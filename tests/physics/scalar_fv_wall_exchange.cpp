// The link-flux transport next to walls: is the difference between its density rho_FV and the LBM density in a wall
// cell the mass in transit on the cell's wall links? The scheme takes no flux on a link to a TYPE_S cell (psi = 0).
// The LBM's Esoteric-Pull bounce-back stores the population f*_i that a fluid cell n sends toward a wall cell w in a
// slot of w and gives it back two steps later. In one step the LBM density of n therefore changes on that link by
// f*_i(step - 1) - f*_i(step), and over steps k1 to k2 by f*_i(k1) - f*_i(k2) (the sum telescopes). So the change of
// rho_FV - rho_LB in n from check k1 to check k2 must equal the change of sum_i f*_i(n) over its wall links.
//
// The case is the plume heat-budget case of heat_conservation.cpp (64 x 64 x 128, TYPE_S floor and ceiling, a TYPE_Y
// patch, beta 2000, SCALAR_FV on the GPU), 1000 steps. The host advances LinkFluxFiniteVolumeScalars on the GPU's flow
// (as scalar_fv_link_flux.cpp) and reads the in-transit populations from the wall slots (incoming() of a solid cell).
// Every 100 steps: per wall cell the change since the first check of rho_FV - rho_LB (measured) and of the in-transit
// sum (predicted). Gate: max |measured - predicted| over the wall cells within 10 % of max |measured|.

#include "setup/setup.hpp"
#include "physics_check.hpp"
#include "debug_probe.hpp"
#include "scalar_fv_reference.hpp"

#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

namespace {

/// @brief The wall cells and the wall links of each.
struct WallLinks {
	std::vector<ulong> cells;             ///< the fluid cells with a solid neighbour among the 18
	std::vector<std::vector<uint>> dirs;  ///< per cell, the directions i with a solid cell n + c_i
};

/// @brief What the run measured.
struct WallResult {
	double worst_mismatch{}; ///< the largest |measured - predicted| change over the wall cells and checks
	double worst_change{};   ///< the largest |measured| change (the scale of the gate)
	ulong worst_cell{};      ///< where the mismatch was largest
};

/// @brief Runs the plume case with the link-flux scheme and compares the wall cells' density difference with the
/// in-transit populations.
/// @return the measurements
WallResult run() {
	const uint Nxy{ 64u }, Nz{ 128u }, r0{ 6u };
	const float nu{ 5.0E-4f }, fz{ -1.0E-6f }, heat_rate{ 1.0E-3f }, beta{ 2000.0f };
	const float alpha{ (float)(5.0E-4/0.71) };
	const ulong steps{ 1000ull }, check{ 100ull };
	LBM lbm(Nxy, Nxy, Nz, nu, 0.0f, 0.0f, fz, 0.0f, alpha, beta);
	const float cx{ 0.5f*(float)(Nxy-1u) }, cy{ cx };
	const ulong N{ lbm.get_N() };
	std::vector<bool> solid(N, false);
	for(ulong n{}; n<N; n++) {
		uint x{}, y{}, z{};
		lbm.coordinates(n, x, y, z);
		lbm.T[n] = 1.0f;
		if(z==0u||z==Nz-1u) { lbm.flags[n] = TYPE_S; solid[n] = true; }
		else if(z==1u&&sq((float)x-cx)+sq((float)y-cy)<=sq((float)r0)) lbm.flags[n] = TYPE_Y;
	}
	lbm.set_heat_rate(heat_rate);
	lbm.set_inv_prandtl_t(3.0f);
	Memory<float>& probe = lbm.debug_cells();
	Memory<fpxx>& populations = lbm.debug_populations();
	const uint first{ DebugProbe::index(DebugProbe::Cell::rho) }, last{ DebugProbe::index(DebugProbe::Cell::w_T) };
	const float* rho_lbm{ &probe[(ulong)DebugProbe::index(DebugProbe::Cell::rho)*N] };
	const float* w_T{ &probe[(ulong)DebugProbe::index(DebugProbe::Cell::w_T)*N] };
	std::vector<float> D(N, 0.0f);
	std::vector<double> psi;
	std::unique_ptr<LinkFluxFiniteVolumeScalars> scheme{};
	WallLinks walls{};
	std::vector<double> transit, transit_first, drift_first;
	WallResult r{};
	bool have_first{};
	for(ulong step{ 1ull }; step<=steps; step++) {
		lbm.run(1ull);
		probe.read_from_device((ulong)first*N, (ulong)(last-first+1u)*N);
		populations.read_from_device();
		if(step==1ull) {
			std::vector<float> start_rho(rho_lbm, rho_lbm+N);
			scheme = std::make_unique<LinkFluxFiniteVolumeScalars>(Nxy, Nxy, Nz, solid, start_rho, std::vector<std::vector<double>>{ std::vector<double>(N, 0.0) });
			for(ulong n{}; n<N; n++) {
				if(solid[n]) continue;
				std::vector<uint> d;
				for(uint i=1u; i<19u; i++) if(solid[scheme->step_to(n, i)]) d.push_back(i);
				if(!d.empty()) { walls.cells.push_back(n); walls.dirs.push_back(d); }
			}
			transit.assign(walls.cells.size(), 0.0);
			std::printf("\nwall exchange: %llu wall cells, %llu steps\n", (unsigned long long)walls.cells.size(), (unsigned long long)steps);
		} else if(step%check==1ull) { // the probe's rho is the LBM density after the last step, the time of transit
			for(size_t c{}; c<walls.cells.size(); c++) {
				const ulong n{ walls.cells[c] };
				const double drift{ scheme->density_at(n)-(double)rho_lbm[n] };
				if(!have_first) { drift_first.push_back(drift); continue; }
				const double measured{ drift-drift_first[c] }, predicted{ transit[c]-transit_first[c] };
				r.worst_change = std::fmax(r.worst_change, std::fabs(measured));
				if(std::fabs(measured-predicted)>r.worst_mismatch) { r.worst_mismatch = std::fabs(measured-predicted); r.worst_cell = n; }
			}
			if(!have_first) { transit_first = transit; have_first = true; }
			uint x{}, y{}, z{};
			lbm.coordinates(r.worst_cell, x, y, z);
			std::printf("step %llu: the largest change of rho_FV - rho_LB in a wall cell %.3e, the largest mismatch to the in-transit change %.3e at (%u, %u, %u)\n",
			            (unsigned long long)step-1ull, r.worst_change, r.worst_mismatch, x, y, z);
		}
		scheme->link_fluxes(populations.data(), lbm.get_t(), psi);
		for(ulong n{}; n<N; n++) D[n] = solid[n] ? 0.0f : 0.25f*(1.0f/w_T[n]-0.5f);
		scheme->step(psi, D.data());
		if(step%check==0ull) for(size_t c{}; c<walls.cells.size(); c++) { // the populations this step sent to the walls
			double sum{};
			for(const uint i : walls.dirs[c]) sum += scheme->incoming(populations.data(), lbm.get_t(), scheme->step_to(walls.cells[c], i), i);
			transit[c] = sum;
		}
	}
	return r;
}

} // namespace

void main_setup() { // extensions: VOLUME_FORCE TEMPERATURE THERMAL_DIFFUSIVITY_FIX SUBGRID SCALAR_FV DEBUG_PROBE
	const WallResult r = run();
	std::printf("\nwall exchange: the largest change %.3e, the largest mismatch %.3e (%.2f %% of the change)\n", r.worst_change, r.worst_mismatch, 100.0*r.worst_mismatch/r.worst_change);
	PhysicsCheck::report("link-flux transport: rho_FV - rho_LB in the wall cells follows the in-transit populations (relative)", r.worst_mismatch/r.worst_change, 0.10);
}
