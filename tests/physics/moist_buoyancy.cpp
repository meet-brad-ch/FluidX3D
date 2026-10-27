// MOISTURE with LEVEL_FORCING: the buoyancy from the virtual potential temperature after the cell's saturation adjustment
// against the host's moist-air model (src/moist_air.hpp).
// A periodic box of air at rest with one state (thl, qt) everywhere, the level pressure p and the reference theta_v_ref
// on every level: the kernel's force is uniform, F = fz - fz beta (theta_v - theta_v_ref) / def_T_scale (the constant
// volume force fz acts on every cell, plus the buoyancy), so the whole box accelerates uniformly, w = (n + 1/2) F / rho
// after n steps (Guo forcing, exact). Six states cover the dry
// limit, the vapour term (0.61 qv), condensation (the latent heating of the saturation adjustment) and two pressures.
// Passes when every measured w after 100 steps is within 0.5 % of the host's (the kernel works in float, the host in double).

#include "setup/setup.hpp"
#include "physics_check.hpp"
#include "moist_air.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

namespace {

/// One state of the box's air.
struct AirState {
	const char* name = "";
	double thl = 0.0;    ///< K
	double qt = 0.0;     ///< kg/kg
	double pressure = 0.0; ///< Pa
};

/// The mean w of the box after the steps, in lattice units.
double mean_w_after(const AirState& a, const double theta_0, const double theta_v_ref, const float fz, const float beta, const ulong steps) {
	const uint N = 16u, Nz = 16u;
	LBM lbm(N, N, Nz, 0.01f, 0.0f, 0.0f, fz, 0.0f, 0.01f, beta);
	for(ulong n=0ull; n<lbm.get_N(); n++) { lbm.T[n] = (float)(1.0+(a.thl-theta_0)/1.0); lbm.Q[n] = (float)(1.0+a.qt/0.01); }
	lbm.set_theta_0((float)theta_0);
	std::vector<float> fx(Nz, 0.0f), fy(Nz, 0.0f), rate(Nz, 0.0f), T_ref(Nz, 1.0f), p(Nz, (float)a.pressure), thv(Nz, (float)theta_v_ref);
	lbm.set_level_forcing(fx, fy, rate, T_ref);
	lbm.set_level_moist(p, thv);
	lbm.run(steps);
	lbm.update_fields();
	lbm.u.read_from_device();
	double sum = 0.0;
	for(ulong n=0ull; n<lbm.get_N(); n++) sum += (double)lbm.u.z[n];
	return sum/(double)lbm.get_N();
}

} // namespace

void main_setup() { // extensions: VOLUME_FORCE TEMPERATURE THERMAL_DIFFUSIVITY_FIX SUBGRID LEVEL_FORCING MOISTURE (the CBL configuration)
	const double theta_0 = 300.0, theta_v_ref = 300.0;
	const float fz = -1.0E-6f, beta = 100.0f; // |fz| beta = 1e-4 per K of theta_v above the reference
	const ulong steps = 100ull;
	const AirState states[] = {
		{ "dry, 1 K warm", 301.0, 0.0, 90000.0 },
		{ "dry, 2 K cold", 298.0, 0.0, 90000.0 },
		{ "moist unsaturated, 10 g/kg at 300 K", 300.0, 0.010, 90000.0 },
		{ "saturated, 22 g/kg at 300 K and 950 hPa", 300.0, 0.022, 95000.0 }, // ql 1.0 g/kg
		{ "saturated, 17 g/kg at 296 K and 900 hPa", 296.0, 0.017, 90000.0 }, // ql 1.9 g/kg
		{ "moist unsaturated, 6 g/kg at 299 K and 800 hPa", 299.0, 0.006, 80000.0 },
	};
	double worst = 0.0;
	std::printf("\nbuoyancy after %llu steps (w = (n + 1/2) F, F = fz (1 - beta (theta_v - %.0f))):\n", (unsigned long long)steps, theta_v_ref);
	for(const AirState& a : states) {
		const double theta_v = moist_air::virtual_theta(a.thl, a.qt, a.pressure);
		const double expected = ((double)steps+0.5)*(double)fz*(1.0-(double)beta*(theta_v-theta_v_ref)); // the constant fz acts on every cell too
		const double measured = mean_w_after(a, theta_0, theta_v_ref, fz, beta, steps);
		const double error = std::fabs(measured-expected)/std::fmax(std::fabs(expected), 1.0E-12);
		worst = std::fmax(worst, error);
		const moist_air::Adjusted adj = moist_air::saturation_adjust(a.thl, a.qt, a.pressure);
		std::printf("  %-46s theta_v %.4f K (T %.3f K, ql %.5f): w %.6e expected %.6e, %.3f %%\n", a.name, theta_v, adj.kelvin, adj.liquid, measured, expected, 100.0*error);
	}
	PhysicsCheck::report("moist buoyancy: the kernel's virtual potential temperature against the host's", worst, 0.005);
}
