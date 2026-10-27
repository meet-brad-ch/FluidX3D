// The conservative finite-volume form with its own density (Guo et al. 2021, eq 17) and 7-point face mass fluxes, on
// the host, on the GPU's flow of the SCALAR_FV heat budget case (heat_conservation.cpp, the plume: TYPE_S floor and ceiling, a TYPE_Y patch, beta 2000; the GPU
// runs SCALAR_FV, whose advective T gained 28.4 % heat). Each step the host reads the GPU's rho, u_eq and w_T from the
// cell probe (DEBUG_PROBE) and advances two passive schemes on that flow:
//   A, the control: the advective FV of the kernel (FiniteVolumeScalar, midpoint step); it must show the GPU's gain;
//   B, the 7-point conservative form: rho_FV, rho (T - 1) and rho (Q - 1) by the 7-point face mass fluxes (ConservativeFiniteVolumeScalars).
// Measured every 100 steps (printed every 1000): the heat of A and of B against the heat added (B as the sum of
// rho_FV (T - 1), as the sum of T - 1, and as the sum of rho_LB (T - 1)), the uniform Q of B, and the drift
// max |rho_FV / rho_LB - 1| with its cell. A case stops when the drift passes 0.5 (rho_FV then goes to 0: NaN).
// Gates: B's conserved heat within 1e-6 of the heat added (rounding), B's uniform Q within 1e-9, A within 2 % of the
// GPU's budget (the host twin follows the kernel), and the drift within 1e-3 (the limit for a usable density).

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
	double gpu_error{};          ///< the GPU kernel's (sum (T - 1) - added) / added at the end
	double control_error{};      ///< the same of the host's advective FV (scheme A)
	double conserved_error{};    ///< B: (sum rho_FV (T - 1) - added) / added, the largest |.| over the samples
	double plain_error{};        ///< B: (sum (T - 1) - added) / added at the end
	double lbm_weighted_error{}; ///< B: (sum rho_LB (T - 1) - added) / added at the end
	double uniform_drift{};      ///< B: the largest |Q - Q_start| over the samples
	double density_drift{};      ///< B: the largest |rho_FV / rho_LB - 1| over the samples
	double lowest_T{};           ///< B: the smallest T of a fluid cell over the samples
	ulong stopped_at{};          ///< the step at which rho_FV left rho_LB by more than 0.5 and the case stopped (0: ran to the end)
};

/// @brief Runs one case: the GPU's plume with the two passive host schemes on its flow.
/// @param c the case
/// @return the measurements
LabResult run_case(const LabCase& c) {
	const uint Nxy{ 64u }, Nz{ 128u }, r0{ 6u };
	const float nu{ 5.0E-4f }, fz{ -1.0E-6f }, heat_rate{ 1.0E-3f }, beta{ 2000.0f };
	const float alpha{ (float)(5.0E-4/0.71) };
	const ulong steps{ 20000ull }, sample{ 1000ull }, check{ 100ull }; // printed every sample, measured every check
	const double Q_offset{ 0.36 }; // the uniform Q - 1 of scheme B (any constant: it must stay)
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
	std::printf("\n%s scalar_fv_conservative, %s: box %u x %u x %u, patch %llu cells, 1/Pr_t %.1f, %llu steps\n", clock_now().c_str(), c.name, Nxy, Nxy, Nz,
	            (unsigned long long)patch_cells.size(), c.inv_prandtl_t, (unsigned long long)steps);
	Memory<float>& probe = lbm.debug_cells();
	const uint first{ DebugProbe::index(DebugProbe::Cell::rho) }, last{ DebugProbe::index(DebugProbe::Cell::w_T) };
	const float* rho_lbm{ &probe[(ulong)DebugProbe::index(DebugProbe::Cell::rho)*N] };
	const float* u_eq{ &probe[(ulong)DebugProbe::index(DebugProbe::Cell::ux_eq)*N] };
	const float* w_T{ &probe[(ulong)DebugProbe::index(DebugProbe::Cell::w_T)*N] };
	std::vector<float> D(N, 0.0f);
	std::unique_ptr<FiniteVolumeScalar> control{};
	std::unique_ptr<ConservativeFiniteVolumeScalars> conservative{};
	LabResult r{};
	r.lowest_T = 1.0E9;
	std::ofstream csv(std::string("scalar_fv_conservative_")+c.name+".csv");
	csv << "step,added,gpu,control,conserved,plain,lbm_weighted,uniform_drift,density_drift,min_T" << std::endl;
	for(ulong step{ 1ull }; step<=steps; step++) {
		lbm.run(1ull);
		probe.read_from_device((ulong)first*N, (ulong)(last-first+1u)*N);
		if(step==1ull) { // the schemes start from the GPU's state before the first step: T = 1, rho = the probe's
			std::vector<float> zero(N, 0.0f);
			std::vector<float> start_rho(rho_lbm, rho_lbm+N);
			control = std::make_unique<FiniteVolumeScalar>(Nxy, Nxy, Nz, solid, zero);
			control->use_rk2(true);
			conservative = std::make_unique<ConservativeFiniteVolumeScalars>(Nxy, Nxy, Nz, solid, start_rho, std::vector<std::vector<double>>{ std::vector<double>(N, 0.0), std::vector<double>(N, Q_offset) });
		} else if(step%check==1ull) { // the probe's rho of this step is the LBM density at the end of the last step: compare with rho_FV
			double worst{};
			ulong worst_n{};
			for(ulong n{}; n<N; n++) {
				if(solid[n]) continue;
				const double drift{ std::fabs(conservative->density_at(n)/(double)rho_lbm[n]-1.0) };
				if(drift>worst) { worst = drift; worst_n = n; }
			}
			r.density_drift = std::fmax(r.density_drift, worst);
			uint x{}, y{}, z{};
			lbm.coordinates(worst_n, x, y, z);
			if(step<=sample+1ull||step%sample==1ull) std::printf("\n%s %s step %llu: rho_FV/rho_LB drift %.3e at cell (%u, %u, %u)%s: rho_FV %.6f, rho_LB %.6f\n", clock_now().c_str(), c.name,
			                                                      (unsigned long long)step-1ull, worst, x, y, z, (lbm.flags[worst_n]&TYPE_Y) ? " (patch)" : "", conservative->density_at(worst_n), (double)rho_lbm[worst_n]);
			if(worst>0.5) { // rho_FV is half off rho_LB: phi = rho phi / rho_FV means nothing any more, and rho_FV goes to 0 next (NaN)
				std::printf("\n%s %s FAILED at step %llu: rho_FV left rho_LB by %.3f at cell (%u, %u, %u); the case stops here\n", clock_now().c_str(), c.name, (unsigned long long)step-1ull, worst, x, y, z);
				r.stopped_at = step-1ull;
				break;
			}
		}
		for(ulong n{}; n<N; n++) D[n] = solid[n] ? 0.0f : 0.25f*(1.0f/w_T[n]-0.5f);
		control->step(u_eq, D.data());
		conservative->step(rho_lbm, u_eq, D.data());
		for(const ulong n : patch_cells) { control->add(n, (double)heat_rate); conservative->add(0ull, n, (double)heat_rate); } // the kernel's source: heat_rate per step in a TYPE_Y cell
		if(step%check==0ull) {
			lbm.update_fields();
			lbm.T.read_from_device();
			const double added{ (double)heat_rate*(double)patch_cells.size()*(double)step };
			double gpu{}, ctrl{}, conserved{}, plain{}, weighted{};
			for(ulong n{}; n<N; n++) {
				if(solid[n]) continue;
				gpu += (double)lbm.T[n]-1.0;
				ctrl += control->at(n);
				conserved += conservative->mass_at(0ull, n);
				plain += conservative->at(0ull, n);
				weighted += (double)rho_lbm[n]*conservative->at(0ull, n);
				r.uniform_drift = std::fmax(r.uniform_drift, std::fabs(conservative->at(1ull, n)-Q_offset));
				r.lowest_T = std::fmin(r.lowest_T, 1.0+conservative->at(0ull, n));
			}
			r.gpu_error = (gpu-added)/added;
			r.control_error = (ctrl-added)/added;
			r.conserved_error = std::fmax(r.conserved_error, std::fabs((conserved-added)/added));
			r.plain_error = (plain-added)/added;
			r.lbm_weighted_error = (weighted-added)/added;
			csv << step << "," << added << "," << gpu << "," << ctrl << "," << conserved << "," << plain << "," << weighted << "," << r.uniform_drift << "," << r.density_drift << "," << r.lowest_T << std::endl;
			if(step%sample==0ull) std::printf("\n%s %s step %llu/%llu: heat vs added: GPU %+.2f %%, A advective %+.2f %%, B conserved %+.2e, B sum(T-1) %+.2f %%, B rho_LB-weighted %+.2f %%; B uniform Q drift %.1e, rho_FV/rho_LB drift %.2e\n",
			            clock_now().c_str(), c.name, (unsigned long long)step, (unsigned long long)steps, 100.0*r.gpu_error, 100.0*r.control_error, (conserved-added)/added,
			            100.0*r.plain_error, 100.0*r.lbm_weighted_error, r.uniform_drift, r.density_drift);
			std::fflush(stdout);
		}
	}
	return r;
}

} // namespace

void main_setup() { // extensions: VOLUME_FORCE TEMPERATURE THERMAL_DIFFUSIVITY_FIX SUBGRID SCALAR_FV DEBUG_PROBE
	const LabCase matrix[]{
		{ 3.0f, "plume" },         // step 3: the advective kernel gained 28.4 %
		{ 0.0f, "plume_no_eddy" }, // step 3: +161 %, the largest gain
	};
	LabResult results[sizeof(matrix)/sizeof(matrix[0])]{};
	for(size_t i{}; i<sizeof(matrix)/sizeof(matrix[0]); i++) results[i] = run_case(matrix[i]);
	std::printf("\nscalar_fv_conservative: heat against the heat added at the last check\n  %-14s %7s %9s %9s %11s %9s %11s %10s %10s %8s\n",
	            "case", "steps", "GPU %", "A %", "B cons.", "B T-1 %", "B rho_LB %", "uniform Q", "rho drift", "min T");
	for(size_t i{}; i<sizeof(matrix)/sizeof(matrix[0]); i++) {
		const LabResult& r = results[i];
		std::printf("  %-14s %7llu %+9.2f %+9.2f %11.2e %+9.2f %+11.2f %10.1e %10.2e %8.4f\n", matrix[i].name, (unsigned long long)(r.stopped_at!=0ull ? r.stopped_at : 20000ull), 100.0*r.gpu_error, 100.0*r.control_error, r.conserved_error,
		            100.0*r.plain_error, 100.0*r.lbm_weighted_error, r.uniform_drift, r.density_drift, r.lowest_T);
	}
	for(size_t i{}; i<sizeof(matrix)/sizeof(matrix[0]); i++) {
		const LabResult& r = results[i];
		const std::string name{ matrix[i].name };
		PhysicsCheck::report(("conservative 7-point "+name+": the conserved heat sum rho_FV (T - 1) against the heat added (over 1e-6)").c_str(), r.conserved_error/1.0E-6, 1.0);
		PhysicsCheck::report(("conservative 7-point "+name+": the uniform Q stays uniform (over 1e-9)").c_str(), r.uniform_drift/1.0E-9, 1.0);
		PhysicsCheck::report(("conservative 7-point "+name+": the host's advective control follows the GPU's budget (relative)").c_str(), std::fabs(r.control_error-r.gpu_error)/std::fabs(r.gpu_error), 0.02);
		PhysicsCheck::report(("conservative 7-point "+name+": rho_FV follows rho_LB (limit 1e-3)").c_str(), r.density_drift, 1.0E-3);
	}
}
