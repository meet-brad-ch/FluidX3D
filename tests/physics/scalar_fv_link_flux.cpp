// The conservative finite-volume form with its own density, transported
// by the LBM's own link mass fluxes (Zhao et al. 2020, eqs 18 and C4; Guo et al. 2021, eq 17), on the host, on the GPU's
// flow of the SCALAR_FV heat budget case (heat_conservation.cpp, the plume; the GPU runs SCALAR_FV). Each step the
// host reads the GPU's rho and w_T (cell probe) and the DDFs the step stored (DEBUG_PROBE), takes the 18 link fluxes
// psi_i(n) = f*_i(n) - f*_ibar(n + c_i) from them (LinkFluxFiniteVolumeScalars::link_fluxes) and advances rho_FV,
// rho (T - 1) and rho (Q - 1) (scheme C) passively on that flow.
// Measured every 100 steps (printed every 1000): the code check, the LBM's continuity from the link fluxes
// max |rho_LB(k+1) - rho_LB(k) + sum_i psi_i| over the cells without a solid neighbour; the heat of the GPU and of C
// against the heat added (C as the sum of rho_FV (T - 1), of T - 1 and of rho_LB (T - 1)); C's uniform Q; the drift
// max |rho_FV / rho_LB - 1| in the cells without a solid neighbour and in the cells next to a wall, with its cell.
// A case stops when a drift passes 0.5. Gates: the continuity within 1e-5 (float rounding of the DDFs), C's conserved
// heat within 1e-6 of the heat added, C's uniform Q within 1e-9, the drift away from walls within 1e-3 (the limit for a usable
// density). The drift next to walls (the Esoteric-Pull bounce-back keeps mass in transit, psi is 0 on
// wall links) is reported, not gated.

#include "setup/setup.hpp"
#include "physics_check.hpp"
#include "debug_probe.hpp"
#include "scalar_fv_reference.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

namespace {

/// @brief The progress lines' time stamp: seconds since the program started (portable, no C time functions).
std::string clock_now() {
	static const std::chrono::steady_clock::time_point start{ std::chrono::steady_clock::now() };
	const double seconds{ std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count() };
	return "t+"+std::to_string((long long)seconds)+" s";
}

/// @brief One case of the lab: the heat_conservation case it repeats.
struct LabCase {
	float inv_prandtl_t{}; ///< the eddy heat diffusivity's 1/Pr_t (0: none)
	const char* name{};    ///< the heat_conservation case name
};

/// @brief What one case measured.
struct LabResult {
	double continuity{};         ///< the largest |rho_LB(k+1) - rho_LB(k) + sum psi| away from walls (the code check)
	double gpu_error{};          ///< the GPU kernel's (sum (T - 1) - added) / added at the last check
	double conserved_error{};    ///< C: (sum rho_FV (T - 1) - added) / added, the largest |.| over the checks
	double plain_error{};        ///< C: (sum (T - 1) - added) / added at the last check
	double lbm_weighted_error{}; ///< C: (sum rho_LB (T - 1) - added) / added at the last check
	double uniform_drift{};      ///< C: the largest |Q - Q_start| over the checks
	double inner_drift{};        ///< C: the largest |rho_FV / rho_LB - 1| in the cells without a solid neighbour
	double wall_drift{};         ///< C: the same in the cells with a solid neighbour
	double lowest_T{};           ///< C: the smallest T of a fluid cell over the checks
	ulong stopped_at{};          ///< the step at which a drift passed 0.5 and the case stopped (0: ran to the end)
};

/// @brief The largest |rho_FV / rho_LB - 1| over a set of cells, with the cell.
struct DriftAt {
	double drift{}; ///< the largest |rho_FV / rho_LB - 1|
	ulong cell{};   ///< where
};

/// @brief Runs one case: the GPU's plume with the link-flux scheme on its flow.
/// @param c the case
/// @return the measurements
LabResult run_case(const LabCase& c) {
	const uint Nxy{ 64u }, Nz{ 128u }, r0{ 6u };
	const float nu{ 5.0E-4f }, fz{ -1.0E-6f }, heat_rate{ 1.0E-3f }, beta{ 2000.0f };
	const float alpha{ (float)(5.0E-4/0.71) };
	const ulong steps{ 20000ull }, sample{ 1000ull }, check{ 100ull }; // printed every sample, measured every check
	const double Q_offset{ 0.36 }; // the uniform Q - 1 of scheme C (any constant: it must stay)
	LBM lbm(Nxy, Nxy, Nz, nu, 0.0f, 0.0f, fz, 0.0f, alpha, beta);
	const float cx{ 0.5f*(float)(Nxy-1u) }, cy{ cx };
	const ulong N{ lbm.get_N() };
	std::vector<bool> solid(N, false);
	std::vector<ulong> patch_cells;
	for(ulong n{}; n<N; n++) {
		uint x{}, y{}, z{};
		lbm.coordinates(n, x, y, z);
		lbm.T[n] = 1.0f;
		if(z==0u||z==Nz-1u) { lbm.flags[n] = TYPE_S; solid[n] = true; }
		else if(z==1u&&sq((float)x-cx)+sq((float)y-cy)<=sq((float)r0)) { lbm.flags[n] = TYPE_Y; patch_cells.push_back(n); }
	}
	lbm.set_heat_rate(heat_rate);
	lbm.set_inv_prandtl_t(c.inv_prandtl_t);
	std::printf("\n%s scalar_fv_link_flux, %s: box %u x %u x %u, patch %llu cells, 1/Pr_t %.1f, %llu steps\n", clock_now().c_str(), c.name, Nxy, Nxy, Nz,
	            (unsigned long long)patch_cells.size(), c.inv_prandtl_t, (unsigned long long)steps);
	Memory<float>& probe = lbm.debug_cells();
	Memory<fpxx>& populations = lbm.debug_populations();
	const uint first{ DebugProbe::index(DebugProbe::Cell::rho) }, last{ DebugProbe::index(DebugProbe::Cell::w_T) };
	const float* rho_lbm{ &probe[(ulong)DebugProbe::index(DebugProbe::Cell::rho)*N] };
	const float* w_T{ &probe[(ulong)DebugProbe::index(DebugProbe::Cell::w_T)*N] };
	std::vector<float> D(N, 0.0f);
	std::vector<double> psi, rho_expected(N, 0.0);
	std::unique_ptr<LinkFluxFiniteVolumeScalars> scheme{};
	std::vector<bool> near_wall(N, false); // a fluid cell with a solid cell among its 18 neighbours
	LabResult r{};
	r.lowest_T = 1.0E9;
	std::ofstream csv(std::string("scalar_fv_link_flux_")+c.name+".csv");
	csv << "step,continuity,added,gpu,conserved,plain,lbm_weighted,uniform_drift,inner_drift,wall_drift,min_T" << std::endl;
	for(ulong step{ 1ull }; step<=steps; step++) {
		lbm.run(1ull);
		probe.read_from_device((ulong)first*N, (ulong)(last-first+1u)*N);
		populations.read_from_device();
		if(step==1ull) { // the scheme starts from the GPU's state before the first step: T = 1, rho = the probe's
			std::vector<float> start_rho(rho_lbm, rho_lbm+N);
			scheme = std::make_unique<LinkFluxFiniteVolumeScalars>(Nxy, Nxy, Nz, solid, start_rho, std::vector<std::vector<double>>{ std::vector<double>(N, 0.0), std::vector<double>(N, Q_offset) });
			for(ulong n{}; n<N; n++) if(!solid[n]) for(uint i=1u; i<19u; i++) if(solid[scheme->along(n, i, 1)]) near_wall[n] = true;
		} else if(step%check==1ull) { // the probe's rho of this step is the LBM density after the last step
			for(ulong n{}; n<N; n++) if(!solid[n]&&!near_wall[n]) r.continuity = std::fmax(r.continuity, std::fabs((double)rho_lbm[n]-rho_expected[n]));
			DriftAt inner{}, wall{};
			for(ulong n{}; n<N; n++) {
				if(solid[n]) continue;
				const double drift{ std::fabs(scheme->density_at(n)/(double)rho_lbm[n]-1.0) };
				DriftAt& set = near_wall[n] ? wall : inner;
				if(drift>set.drift) { set.drift = drift; set.cell = n; }
			}
			r.inner_drift = std::fmax(r.inner_drift, inner.drift);
			r.wall_drift = std::fmax(r.wall_drift, wall.drift);
			uint xi{}, yi{}, zi{}, xw{}, yw{}, zw{};
			lbm.coordinates(inner.cell, xi, yi, zi);
			lbm.coordinates(wall.cell, xw, yw, zw);
			if(step<=sample+1ull||step%sample==1ull) std::printf("\n%s %s step %llu: continuity %.2e; rho_FV/rho_LB drift %.3e at (%u, %u, %u) away from walls, %.3e at (%u, %u, %u) next to a wall\n",
			                                                      clock_now().c_str(), c.name, (unsigned long long)step-1ull, r.continuity, inner.drift, xi, yi, zi, wall.drift, xw, yw, zw);
			if(inner.drift>0.5||wall.drift>0.5) { // phi = rho phi / rho_FV means nothing any more
				std::printf("\n%s %s FAILED at step %llu: rho_FV left rho_LB by %.3f; the case stops here\n", clock_now().c_str(), c.name, (unsigned long long)step-1ull, std::fmax(inner.drift, wall.drift));
				r.stopped_at = step-1ull;
				break;
			}
		}
		scheme->link_fluxes(populations.data(), lbm.get_t(), psi);
		if(step%check==0ull) for(ulong n{}; n<N; n++) { // the density the LBM's own fluxes predict for the next probe
			double sum{};
			for(uint i=0u; i<18u; i++) sum += psi[(ulong)i*N+n];
			rho_expected[n] = (double)rho_lbm[n]-sum;
		}
		for(ulong n{}; n<N; n++) D[n] = solid[n] ? 0.0f : 0.25f*(1.0f/w_T[n]-0.5f);
		scheme->step(psi, D.data());
		for(const ulong n : patch_cells) scheme->add(0ull, n, (double)heat_rate); // the kernel's source: heat_rate per step in a TYPE_Y cell
		if(step%check==0ull) {
			lbm.update_fields();
			lbm.T.read_from_device();
			const double added{ (double)heat_rate*(double)patch_cells.size()*(double)step };
			double gpu{}, conserved{}, plain{}, weighted{};
			for(ulong n{}; n<N; n++) {
				if(solid[n]) continue;
				gpu += (double)lbm.T[n]-1.0;
				conserved += scheme->mass_at(0ull, n);
				plain += scheme->at(0ull, n);
				weighted += (double)rho_lbm[n]*scheme->at(0ull, n);
				r.uniform_drift = std::fmax(r.uniform_drift, std::fabs(scheme->at(1ull, n)-Q_offset));
				r.lowest_T = std::fmin(r.lowest_T, 1.0+scheme->at(0ull, n));
			}
			r.gpu_error = (gpu-added)/added;
			r.conserved_error = std::fmax(r.conserved_error, std::fabs((conserved-added)/added));
			r.plain_error = (plain-added)/added;
			r.lbm_weighted_error = (weighted-added)/added;
			csv << step << "," << r.continuity << "," << added << "," << gpu << "," << conserved << "," << plain << "," << weighted << "," << r.uniform_drift << "," << r.inner_drift << "," << r.wall_drift << "," << r.lowest_T << std::endl;
			if(step%sample==0ull) std::printf("\n%s %s step %llu/%llu: heat vs added: GPU %+.2f %%, C conserved %+.2e, C sum(T-1) %+.2f %%, C rho_LB-weighted %+.2f %%; C uniform Q drift %.1e\n",
			                                  clock_now().c_str(), c.name, (unsigned long long)step, (unsigned long long)steps, 100.0*r.gpu_error, (conserved-added)/added,
			                                  100.0*r.plain_error, 100.0*r.lbm_weighted_error, r.uniform_drift);
			std::fflush(stdout);
		}
	}
	return r;
}

} // namespace

void main_setup() { // extensions: VOLUME_FORCE TEMPERATURE THERMAL_DIFFUSIVITY_FIX SUBGRID SCALAR_FV DEBUG_PROBE
	const LabCase matrix[]{
		{ 3.0f, "plume" },         // the advective kernel gains 28.4 %; the 7-point form's rho_FV leaves the LBM density
		{ 0.0f, "plume_no_eddy" }, // step 3: +161 %
	};
	LabResult results[sizeof(matrix)/sizeof(matrix[0])]{};
	for(size_t i{}; i<sizeof(matrix)/sizeof(matrix[0]); i++) results[i] = run_case(matrix[i]);
	std::printf("\nscalar_fv_link_flux: at the last check\n  %-14s %7s %11s %9s %11s %9s %11s %10s %11s %10s %8s\n",
	            "case", "steps", "continuity", "GPU %", "C cons.", "C T-1 %", "C rho_LB %", "uniform Q", "drift inner", "drift wall", "min T");
	for(size_t i{}; i<sizeof(matrix)/sizeof(matrix[0]); i++) {
		const LabResult& r = results[i];
		std::printf("  %-14s %7llu %11.2e %+9.2f %11.2e %+9.2f %+11.2f %10.1e %11.2e %10.2e %8.4f\n", matrix[i].name, (unsigned long long)(r.stopped_at!=0ull ? r.stopped_at : 20000ull),
		            r.continuity, 100.0*r.gpu_error, r.conserved_error, 100.0*r.plain_error, 100.0*r.lbm_weighted_error, r.uniform_drift, r.inner_drift, r.wall_drift, r.lowest_T);
	}
	for(size_t i{}; i<sizeof(matrix)/sizeof(matrix[0]); i++) {
		const LabResult& r = results[i];
		const std::string name{ matrix[i].name };
		PhysicsCheck::report(("link-flux "+name+": the link fluxes give the LBM's density change away from walls (code check)").c_str(), r.continuity, 1.0E-5);
		PhysicsCheck::report(("link-flux "+name+": the conserved heat sum rho_FV (T - 1) against the heat added (over 1e-6)").c_str(), r.conserved_error/1.0E-6, 1.0);
		PhysicsCheck::report(("link-flux "+name+": the uniform Q stays uniform (over 1e-9)").c_str(), r.uniform_drift/1.0E-9, 1.0);
		PhysicsCheck::report(("link-flux "+name+": rho_FV follows rho_LB away from walls (limit 1e-3)").c_str(), r.inner_drift, 1.0E-3);
	}
}
