// The heat budget with buoyancy (SCALAR_FV): a heated patch on the floor of a box with TYPE_S floor and ceiling (nu
// 5e-4, alpha nu/0.71 with THERMAL_DIFFUSIVITY_FIX, SUBGRID with Pr_t 1/3, the Boussinesq force f = fz (1 - beta (T - 1))
// with fz = -1e-6). Every 1000 steps of 20000: the total heat sum(T - 1) over the fluid cells against the heat added
// (heat_rate x patch cells x steps), and the lowest T. The cases: without buoyancy (beta 0), the plume (beta 2000), the
// plume without the eddy heat diffusivity, and the plume in a box periodic in z (no walls).
// Passes when in every case the total heat stays within 2 % of the heat added and T never falls below 1 - 0.01
// (nothing cools below the start). The D3Q7 thermal lattice does not pass at this alpha (tauT 0.503): next to walls it
// creates heat, so SCALAR_FV is the transport for such cases (research/convection/README.md).

#include "setup/setup.hpp"
#include "physics_check.hpp"

#include <cmath>
#include <cstdio>

namespace {

/// The heat budget of one sample.
struct HeatSample {
	double total = 0.0;        ///< sum(T - 1) over the fluid cells
	double minimum = 0.0;      ///< the smallest T of a fluid cell
	double maximum = 0.0;      ///< the largest T of a fluid cell
	double floor_mean = 0.0;   ///< the mean T of z = 1 outside the patch
};

HeatSample sample_of(LBM& lbm, const uint patch_z) {
	lbm.update_fields();
	lbm.T.read_from_device();
	HeatSample s;
	s.minimum = 1.0E9; s.maximum = -1.0E9;
	ulong floor_cells = 0ull;
	for(ulong n=0ull; n<lbm.get_N(); n++) {
		if(lbm.flags[n]&TYPE_S) continue;
		const double T = (double)lbm.T[n];
		s.total += T-1.0;
		s.minimum = std::fmin(s.minimum, T);
		s.maximum = std::fmax(s.maximum, T);
		uint x=0u, y=0u, z=0u;
		lbm.coordinates(n, x, y, z);
		if(z==patch_z&&!(lbm.flags[n]&TYPE_Y)) { s.floor_mean += T; floor_cells++; }
	}
	s.floor_mean /= (double)floor_cells;
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
	for(ulong t=0ull; t<steps; t+=sample) {
		lbm.run(sample);
		const HeatSample s = sample_of(lbm, patch_z);
		const double added = (double)heat_rate*(double)patch*(double)(t+sample);
		r.final_error = (s.total-added)/added;
		r.worst_budget = std::fmax(r.worst_budget, std::fabs(r.final_error));
		r.lowest_T = std::fmin(r.lowest_T, s.minimum);
		r.floor_T = s.floor_mean;
	}
	std::printf("  %s: budget error at the end %+.1f %% (worst %.1f %%), min T %.3f, floor T at the end %.3f\n", c.name, 100.0*r.final_error, 100.0*r.worst_budget, r.lowest_T, r.floor_T);
	return r;
}

} // namespace

void main_setup() { // extensions: VOLUME_FORCE TEMPERATURE THERMAL_DIFFUSIVITY_FIX SUBGRID SCALAR_FV
	const float alpha = (float)(5.0E-4/0.71); // the plume's diffusivity (tauT 0.503)
	const CaseSettings matrix[] = {
		{ 0.0f, 3.0f, alpha, true, "without buoyancy" },
		{ 2000.0f, 3.0f, alpha, true, "plume" },
		{ 2000.0f, 0.0f, alpha, true, "plume without the eddy heat diffusivity" },
		{ 2000.0f, 3.0f, alpha, false, "plume, periodic in z" },
	};
	for(const CaseSettings& c : matrix) {
		const CaseResult r = run_case(c);
		char name[160];
		std::snprintf(name, sizeof(name), "heat conservation, %s: total heat against the heat added", c.name);
		PhysicsCheck::report(name, r.worst_budget, 0.02);
		std::snprintf(name, sizeof(name), "heat conservation, %s: no cooling below the start (1 - min T)", c.name);
		PhysicsCheck::report(name, std::fmax(0.0, 1.0-r.lowest_T), 0.01);
	}
}
