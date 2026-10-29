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

/// @brief One state of the box's air.
struct AirState {
	const char* name{ "" }; ///< the state's name, for the message
	double thl{ 0.0 };      ///< the liquid water potential temperature in K
	double qt{ 0.0 };       ///< the total water in kg/kg
	double pressure{ 0.0 }; ///< the level pressure in Pa
};

/// @brief The buoyancy of one air state in the kernel against the host's moist-air model.
class MoistBuoyancy {
public:
	/// @brief Gives the relative error of the kernel's uniform acceleration of a box of the state against the host's.
	/// @param a the air state
	/// @return the relative error
	static double error_of(const AirState& a) {
		const double theta_v{ moist_air::virtual_theta(a.thl, a.qt, a.pressure) };
		const double half_steps{ static_cast<double>(steps) + 0.5 };
		const double fz_d{ static_cast<double>(fz) };
		const double beta_d{ static_cast<double>(beta) };
		const double expected{ half_steps * fz_d * (1.0 - beta_d * (theta_v - theta_v_ref)) }; // the constant fz acts on every cell too
		const double measured{ mean_w_after(a) };
		const double scale{ std::fmax(std::fabs(expected), smallest_expected) };
		const double error{ std::fabs(measured - expected) / scale };
		const moist_air::Adjusted adj = moist_air::saturation_adjust(a.thl, a.qt, a.pressure);
		const double error_percent{ percent * error };
		std::printf("  %-46s theta_v %.4f K (T %.3f K, ql %.5f): w %.6e expected %.6e, %.3f %%\n", a.name, theta_v, adj.kelvin, adj.liquid, measured, expected, error_percent);
		return error;
	}

	static constexpr double theta_0 = 300.0;     ///< the reference potential temperature in K
	static constexpr double theta_v_ref = 300.0; ///< the reference virtual potential temperature on every level, in K
	static constexpr ulong steps = 100ull;       ///< the steps of each run

private:
	/// @brief Gives the mean w of a box of the state after the steps.
	/// @param a the air state
	/// @return w in lattice units
	static double mean_w_after(const AirState& a) {
		LBM lbm(box_cells, box_cells, box_cells, viscosity, 0.0f, 0.0f, fz, 0.0f, alpha, beta);
		const float start_t{ static_cast<float>(1.0 + (a.thl - theta_0) / 1.0) };
		const float start_q{ static_cast<float>(1.0 + a.qt / q_scale) };
		const ulong cells = lbm.get_N();
		for(ulong n{ 0ull }; n < cells; n++) {
			lbm.T[n] = start_t;
			lbm.Q[n] = start_q;
		}
		lbm.set_theta_0(static_cast<float>(theta_0));
		const std::vector<float> fx(box_cells, 0.0f);
		const std::vector<float> fy(box_cells, 0.0f);
		const std::vector<float> rate(box_cells, 0.0f);
		const std::vector<float> T_ref(box_cells, 1.0f);
		const std::vector<float> p(box_cells, static_cast<float>(a.pressure));
		const std::vector<float> thv(box_cells, static_cast<float>(theta_v_ref));
		lbm.set_level_forcing(fx, fy, rate, T_ref);
		lbm.set_level_moist(p, thv);
		lbm.run(steps);
		lbm.update_fields();
		lbm.u.read_from_device();
		double sum{ 0.0 };
		for(ulong n{ 0ull }; n < cells; n++) {
			sum += static_cast<double>(lbm.u.z[n]);
		}
		const double mean{ sum / static_cast<double>(cells) };
		return mean;
	}

	static constexpr uint box_cells = 16u;           ///< the box's cells along each axis (and its levels)
	static constexpr float viscosity = 0.01f;        ///< the kinematic viscosity in lattice units
	static constexpr float alpha = 0.01f;            ///< the thermal diffusion coefficient in lattice units
	static constexpr float fz = -1.0E-6f;            ///< the constant volume force along z in lattice units
	static constexpr float beta = 100.0f;            ///< |fz| beta = 1e-4 per K of theta_v above the reference
	static constexpr double q_scale = 0.01;          ///< the kernel's water scale def_q_scale in kg/kg: Q = 1 + qt / q_scale
	static constexpr double smallest_expected = 1.0E-12; ///< the smallest |expected| that the error is relative to
	static constexpr double percent = 100.0;         ///< the percent of a fraction of 1, for the printed error
};

} // namespace

void main_setup() { // extensions: VOLUME_FORCE TEMPERATURE THERMAL_DIFFUSIVITY_FIX SUBGRID LEVEL_FORCING MOISTURE (the CBL configuration)
	const AirState states[] = {
		{ "dry, 1 K warm", 301.0, 0.0, 90000.0 },
		{ "dry, 2 K cold", 298.0, 0.0, 90000.0 },
		{ "moist unsaturated, 10 g/kg at 300 K", 300.0, 0.010, 90000.0 },
		{ "saturated, 22 g/kg at 300 K and 950 hPa", 300.0, 0.022, 95000.0 }, // ql 1.0 g/kg
		{ "saturated, 17 g/kg at 296 K and 900 hPa", 296.0, 0.017, 90000.0 }, // ql 1.9 g/kg
		{ "moist unsaturated, 6 g/kg at 299 K and 800 hPa", 299.0, 0.006, 80000.0 },
	};
	double worst{ 0.0 };
	const unsigned long long printed_steps{ static_cast<unsigned long long>(MoistBuoyancy::steps) };
	std::printf("\nbuoyancy after %llu steps (w = (n + 1/2) F, F = fz (1 - beta (theta_v - %.0f))):\n", printed_steps, MoistBuoyancy::theta_v_ref);
	for(const AirState& a : states) {
		const double error{ MoistBuoyancy::error_of(a) };
		worst = std::fmax(worst, error);
	}
	const double error_limit{ 0.005 }; // the kernel works in float, the host in double
	PhysicsCheck::report("moist buoyancy: the kernel's virtual potential temperature against the host's", worst, error_limit);
}
