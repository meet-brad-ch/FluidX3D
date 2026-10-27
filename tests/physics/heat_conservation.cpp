// Heat conservation of the thermal lattice in the plume test's conditions: the plume
// run showed a cold layer (T about 0.03) on the whole floor and a warm layer (1.34) under the ceiling, far more heat
// moved than the patch had added. This test measures the heat budget in the same lattice (TYPE_S floor and ceiling,
// a TYPE_Y patch on the floor, nu 5e-4, alpha nu/0.71 with THERMAL_DIFFUSIVITY_FIX, SUBGRID with Pr_t 1/3, the
// Boussinesq force f = fz (1 - beta (T - 1)) with fz = -1e-6) in a small box, twice: beta = 0 (no buoyancy: the heat
// only diffuses from the patch) and beta = 2000 (the plume). Every 1000 steps: the total heat sum(T - 1) over the fluid
// cells against the heat added (heat_rate x patch cells x steps; esoteric pull keeps one population per wall link in
// the wall's slots, a bounded bookkeeping offset), the minimum and maximum T, the mean T of the floor layer outside the
// patch and of the ceiling layer. Passes when the total heat stays within 2 % of the heat added in both runs and T never
// falls below 1 - 0.01 (nothing cools below the start).

#include "setup/setup.hpp"
#include "physics_check.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

namespace {

/// The heat budget of one sample.
struct HeatSample {
	double total = 0.0;        ///< sum(T - 1) over the fluid cells
	double minimum = 0.0;      ///< the smallest T of a fluid cell
	double maximum = 0.0;      ///< the largest T of a fluid cell
	double floor_mean = 0.0;   ///< the mean T of z = 1 outside the patch
	double ceiling_mean = 0.0; ///< the mean T of z = Nz - 2
};

HeatSample sample_of(LBM& lbm, const uint Nz, const uint patch_z) {
	lbm.update_fields();
	lbm.T.read_from_device();
	HeatSample s;
	s.minimum = 1.0E9; s.maximum = -1.0E9;
	ulong floor_cells = 0ull, ceiling_cells = 0ull;
	for(ulong n=0ull; n<lbm.get_N(); n++) {
		if(lbm.flags[n]&TYPE_S) continue;
		const double T = (double)lbm.T[n];
		s.total += T-1.0;
		s.minimum = std::fmin(s.minimum, T);
		s.maximum = std::fmax(s.maximum, T);
		uint x=0u, y=0u, z=0u;
		lbm.coordinates(n, x, y, z);
		if(z==patch_z&&!(lbm.flags[n]&TYPE_Y)) { s.floor_mean += T; floor_cells++; }
		if(z==Nz-2u) { s.ceiling_mean += T; ceiling_cells++; }
	}
	s.floor_mean /= (double)floor_cells;
	s.ceiling_mean /= (double)ceiling_cells;
	return s;
}

/// One case of the matrix.
struct CaseSettings {
	float beta = 0.0f;         ///< the thermal expansion (0: no buoyancy)
	float inv_prandtl_t = 0.0f;///< the eddy heat diffusivity's 1/Pr_t (0: SUBGRID adds nothing to the thermal relaxation)
	float alpha = 0.0f;        ///< the thermal diffusivity
	bool walls = true;         ///< TYPE_S floor and ceiling (false: periodic in z, the patch at z = 0 as the heat_source test)
	const char* name = "";
};

/// What one case measured at the end.
struct CaseResult {
	double worst_budget = 0.0; ///< the largest |total - added| / added over the samples
	double lowest_T = 0.0;     ///< the smallest T of a fluid cell over the samples
	double final_error = 0.0;  ///< (total - added) / added at the end
	double floor_T = 0.0;      ///< the floor layer's mean T outside the patch at the end
};

CaseResult run_case(const CaseSettings& c) {
	const uint N = 64u, Nz = 128u, r0 = 6u;
	const float nu = 5.0E-4f, fz = -1.0E-6f;
	const float heat_rate = 1.0E-3f;
	const ulong steps = 20000ull, sample = 1000ull;
	LBM lbm(N, N, Nz, nu, 0.0f, 0.0f, fz, 0.0f, c.alpha, c.beta);
	const float cx = 0.5f*(float)(N-1u), cy = cx;
	const uint patch_z = c.walls ? 1u : 0u;
	ulong patch = 0ull;
	for(ulong n=0ull; n<lbm.get_N(); n++) {
		uint x=0u, y=0u, z=0u;
		lbm.coordinates(n, x, y, z);
		lbm.T[n] = 1.0f;
		if(c.walls&&(z==0u||z==Nz-1u)) lbm.flags[n] = TYPE_S;
		else if(z==patch_z&&sq((float)x-cx)+sq((float)y-cy)<=sq((float)r0)) { lbm.flags[n] = TYPE_Y; patch++; }
	}
	lbm.set_heat_rate(heat_rate);
	lbm.set_inv_prandtl_t(c.inv_prandtl_t);
	std::printf("\nheat conservation, %s: beta %.0f, 1/Pr_t %.1f, alpha %.2e, walls %d; box %u x %u x %u, patch %llu cells, heat_rate %.1e\n",
	            c.name, c.beta, c.inv_prandtl_t, c.alpha, (int)c.walls, N, N, Nz, (unsigned long long)patch, heat_rate);
	CaseResult r;
	r.lowest_T = 1.0E9;
	std::ofstream csv(std::string("heat_conservation_")+c.name+".csv"); // the console rows are overwritten by the run's progress bar
	csv << "step,added,total,error,min_T,max_T,floor_T,ceiling_T" << std::endl;
	for(ulong t=0ull; t<steps; t+=sample) {
		lbm.run(sample);
		const HeatSample s = sample_of(lbm, Nz, patch_z);
		const double added = (double)heat_rate*(double)patch*(double)(t+sample);
		r.final_error = (s.total-added)/added;
		r.worst_budget = std::fmax(r.worst_budget, std::fabs(r.final_error));
		r.lowest_T = std::fmin(r.lowest_T, s.minimum);
		r.floor_T = s.floor_mean;
		csv << (t+sample) << "," << added << "," << s.total << "," << r.final_error << "," << s.minimum << "," << s.maximum << "," << s.floor_mean << "," << s.ceiling_mean << std::endl;
	}
	std::printf("  %s: budget error at the end %+.1f %% (worst %.1f %%), min T %.3f, floor T at the end %.3f\n", c.name, 100.0*r.final_error, 100.0*r.worst_budget, r.lowest_T, r.floor_T);
	return r;
}

} // namespace

void main_setup() { // extensions: VOLUME_FORCE TEMPERATURE THERMAL_DIFFUSIVITY_FIX SUBGRID
	const float a1 = (float)(5.0E-4/0.71), a3 = 3.0f*a1, a5 = 5.0f*a1, a10 = 10.0f*a1; // the plume's diffusivity (tauT 0.503), 3, 5 and 10 times it (0.508, 0.514, 0.528)
	const CaseSettings matrix[] = {
		{ 0.0f, 3.0f, a1, true, "still_plume" },           // the plume's lattice without buoyancy
		{ 2000.0f, 3.0f, a1, true, "plume" },              // the plume's lattice
		{ 2000.0f, 0.0f, a1, true, "plume_no_eddy" },      // SUBGRID's eddy heat diffusivity off
		{ 2000.0f, 3.0f, a3, true, "plume_alpha3" },       // three times the diffusivity (tauT 0.508)
		{ 2000.0f, 3.0f, a5, true, "plume_alpha5" },       // five times (tauT 0.514)
		{ 2000.0f, 3.0f, a10, true, "plume_alpha10" },     // ten times the diffusivity
		{ 2000.0f, 0.0f, a10, true, "plume_alpha10_no_eddy" },
		{ 2000.0f, 3.0f, a1, false, "plume_periodic" },    // no walls: periodic in z
		{ 0.0f, 3.0f, a1, false, "still_periodic" },
	};
	CaseResult results[sizeof(matrix)/sizeof(matrix[0])];
	for(size_t i=0; i<sizeof(matrix)/sizeof(matrix[0]); i++) results[i] = run_case(matrix[i]);
	std::printf("\nheat conservation matrix (20000 steps):\n  %-24s %12s %10s %10s %10s\n", "case", "end error %", "worst %", "min T", "floor T");
	for(size_t i=0; i<sizeof(matrix)/sizeof(matrix[0]); i++) std::printf("  %-24s %+12.1f %10.1f %10.3f %10.3f\n", matrix[i].name, 100.0*results[i].final_error, 100.0*results[i].worst_budget, results[i].lowest_T, results[i].floor_T);
	PhysicsCheck::report("heat conservation without buoyancy: total heat against the heat added", results[0].worst_budget, 0.02);
	PhysicsCheck::report("heat conservation with buoyancy (plume): total heat against the heat added", results[1].worst_budget, 0.02);
	PhysicsCheck::report("no cooling below the start without buoyancy (1 - min T)", std::fmax(0.0, 1.0-results[0].lowest_T), 0.01);
	PhysicsCheck::report("no cooling below the start with buoyancy (1 - min T)", std::fmax(0.0, 1.0-results[1].lowest_T), 0.01);
}
