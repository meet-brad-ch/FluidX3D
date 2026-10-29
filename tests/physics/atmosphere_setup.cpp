// An atmosphere through the Setup API: a Simulation made with a TimeStepRule, lattice values for the viscosity and the
// thermal diffusivity, a temperature scale (theta_0, 1 K) and a BoussinesqBuoyancy. Checks what the core received:
// dt from the rule at the cell size, nu and alpha as given (alpha through THERMAL_DIFFUSIVITY_FIX), the carrier fz and
// beta = g dt^2/(dx theta_0 carrier). Then the physics: a periodic box at rest, 1 K warmer than the reference
// everywhere, accelerates uniformly by the buoyancy g * 1 K / theta_0 less the carrier's pull (|fz| dx/dt^2);
// with Guo forcing the velocity after n steps is (n + 1/2) F / rho (exact, as tests/physics/moist_buoyancy.cpp).
// The run estimate before the LBM exists: the cells and the steps.
// Passes when the received values are as given (float rounding) and w after 100 steps is within 0.5 % of the
// physics' value.

#include "setup/setup.hpp"
#include "physics_check.hpp"

#include <cmath>

namespace {

/// @brief The given set-up values and the checks of what the core received and of the buoyant rise.
class AtmosphereSetupCheck {
public:
	static constexpr double cell_m = 2.5;              ///< the cell size in m
	static constexpr double step_s = 0.04;             ///< the time step of the rule at 2.5 m, in s
	static constexpr double theta_0_k = 300.0;         ///< the reference temperature in K
	static constexpr double viscosity_lattice = 1.0E-5; ///< the given viscosity in lattice units
	static constexpr double alpha_lattice = 1.0E-4;     ///< the given thermal diffusivity in lattice units
	static constexpr double carrier = 1.0E-6;           ///< BoussinesqBuoyancy's default carrier (fz = -carrier)
	static constexpr double gravity_mps2 = 9.81;        ///< the gravity in m/s^2
	static constexpr double box_m = 40.0;               ///< the box's side in m (16 cells)

	/// @brief Checks the run estimate: 16^3 cells, 25 steps in 1 s, one domain.
	/// @param sim the simulation before its LBM exists
	static void check_estimate(Simulation& sim) {
		using namespace si::symbols;
		const si::Duration run_time = 1.0 * s;
		const si::LatticeUpdateRate speed = 2000.0 * MLUPs;
		const RunEstimate estimate = sim.estimate(run_time, speed);
		const std::string estimate_text = estimate.describe();
		std::printf("\nestimate: %s\n", estimate_text.c_str());
		const std::uint64_t side_cells{ 16ull };
		const bool counts_cells{ estimate.cells == side_cells * side_cells * side_cells };
		const bool counts_steps{ estimate.steps == 25ull };
		const bool one_domain{ estimate.domains.x == 1u };
		const bool estimate_right{ counts_cells && counts_steps && one_domain };
		const double estimate_error{ estimate_right ? 0.0 : 1.0 };
		PhysicsCheck::report("atmosphere set-up: the estimate counts 16^3 cells and 25 steps in 1 s", estimate_error, count_limit);
	}

	/// @brief Checks the values that the core received: dt, nu, alpha, fz and beta.
	/// @param sim the simulation after its LBM exists
	static void check_received(Simulation& sim) {
		using namespace si::symbols;
		const LBM& lbm = sim.lbm(); // expert use: the received values
		const double dt{ sim.converter().step().in(s) };
		const double dx{ sim.converter().cell().in(m) };
		const double g_dt2{ gravity_mps2 * dt * dt };
		const double dx_theta_carrier{ dx * theta_0_k * carrier };
		const double beta_expected{ g_dt2 / dx_theta_carrier }; // the derivation, in double
		const double nu{ static_cast<double>(lbm.get_nu()) };
		const double alpha{ static_cast<double>(lbm.get_alpha()) };
		const double fz{ static_cast<double>(lbm.get_fz()) };
		const double beta{ static_cast<double>(lbm.get_beta()) };
		std::printf("\nreceived: dt %.6g s, dx %.6g m, nu %.6g, alpha %.6g, fz %.6g, beta %.8g (expected %.8g)\n",
		            dt, dx, nu, alpha, fz, beta, beta_expected);
		report_ratio("atmosphere set-up: dt = 0.04 s at 2.5 m (the rule)", dt, step_s);
		report_ratio("atmosphere set-up: nu as given (lattice)", nu, viscosity_lattice);
		report_ratio("atmosphere set-up: alpha as given (the fix: alpha = D)", alpha, alpha_lattice);
		report_ratio("atmosphere set-up: the carrier fz = -1e-6", fz, -carrier);
		report_ratio("atmosphere set-up: beta = g dt^2/(dx theta_0 carrier)", beta, beta_expected);
	}

	/// @brief Warms the box 1 K above theta_0, runs it and checks its mean rise against (n + 1/2) dt (g 1 K/theta_0 -
	/// carrier), exact with Guo forcing.
	/// @param sim the simulation after its LBM exists
	static void check_rise(Simulation& sim) {
		using namespace si::symbols;
		LBM& lbm = sim.lbm(); // expert use: the start field
		const double dt{ sim.converter().step().in(s) };
		const double dx{ sim.converter().cell().in(m) };
		const ulong cells = lbm.get_N();
		const si::Temperature one_kelvin_warmer = si::kelvin_point(theta_0_k + 1.0);
		const float warm{ sim.converter().temperature(one_kelvin_warmer) }; // 1 K above theta_0: T = 2
		for(ulong n{ 0ull }; n < cells; n++) {
			lbm.T[n] = warm;
		}
		const ulong steps{ 100ull };
		lbm.run(steps);
		lbm.u.read_from_device();
		double w_sum{ 0.0 };
		for(ulong n{ 0ull }; n < cells; n++) {
			const double uz{ static_cast<double>(lbm.u.z[n]) };
			w_sum += uz;
		}
		const double w_mean{ w_sum / static_cast<double>(cells) };
		const double w_cells{ w_mean * dx };
		const double w{ w_cells / dt }; // m/s
		const double buoyancy_acceleration{ gravity_mps2 * 1.0 / theta_0_k }; // the buoyancy of 1 K, m/s^2
		const double dt2{ dt * dt };
		const double carrier_acceleration{ carrier * dx / dt2 }; // the carrier's pull, m/s^2
		const double acceleration{ buoyancy_acceleration - carrier_acceleration }; // the buoyancy of 1 K less the carrier, m/s^2
		const double half_steps{ static_cast<double>(steps) + 0.5 };
		const double rise_time{ half_steps * dt };
		const double expected{ rise_time * acceleration };
		const unsigned long long printed_steps{ static_cast<unsigned long long>(steps) };
		std::printf("w after %llu steps: %.6e m/s, expected (n + 1/2) dt (g 1 K/theta_0 - carrier) = %.6e m/s\n", printed_steps, w, expected);
		const double w_ratio{ w / expected };
		const double w_error{ std::fabs(w_ratio - 1.0) };
		PhysicsCheck::report("atmosphere set-up: a 1 K warmer box rises as g/theta_0 per K", w_error, rise_limit);
	}

private:
	/// @brief Reports a received value against its given value.
	/// @param check the check's name
	/// @param received the value that the core received
	/// @param given the value that was given
	static void report_ratio(const char* check, double received, double given) {
		const double ratio{ received / given };
		const double error{ std::fabs(ratio - 1.0) };
		PhysicsCheck::report(check, error, received_limit);
	}

	static constexpr double count_limit = 0.5;      ///< a wrong count fails (error 1), a right one passes (error 0)
	static constexpr double received_limit = 1.0E-6; ///< float rounding of a received value
	static constexpr double rise_limit = 0.005;     ///< the largest relative error of the rise that passes (0.5 %)
};

} // namespace

void main_setup() { // extensions: VOLUME_FORCE, TEMPERATURE, THERMAL_DIFFUSIVITY_FIX
	using namespace si::symbols;
	const si::Length cell = AtmosphereSetupCheck::cell_m * m;
	const si::Duration step = AtmosphereSetupCheck::step_s * s;
	const TimeStepRule rule = TimeStepRule::constant_lattice_speed(step, cell);
	const si::Temperature theta_0 = si::kelvin_point(AtmosphereSetupCheck::theta_0_k);
	const si::Length side = AtmosphereSetupCheck::box_m * m;
	Domain domain = Domain::box(side, side, side);
	domain.cell_size(cell);
	Simulation sim(domain, Fluid::AIR, rule);
	const si::LatticeDiffusivity nu{ AtmosphereSetupCheck::viscosity_lattice };
	const Diffusivity viscosity = Diffusivity::lattice(nu);
	sim.set_viscosity(viscosity);
	const si::LatticeDiffusivity alpha{ AtmosphereSetupCheck::alpha_lattice };
	const Diffusivity thermal_diffusivity = Diffusivity::lattice(alpha);
	sim.set_thermal_diffusivity(thermal_diffusivity);
	const si::TemperatureDifference one_kelvin = 1.0 * K;
	sim.set_temperature_scale(theta_0, one_kelvin);
	const si::Acceleration gravity = AtmosphereSetupCheck::gravity_mps2 * m / (s * s);
	const BoussinesqBuoyancy buoyancy = BoussinesqBuoyancy(gravity, theta_0);
	sim.set_boussinesq_buoyancy(buoyancy);
	AtmosphereSetupCheck::check_estimate(sim);
	AtmosphereSetupCheck::check_received(sim);
	AtmosphereSetupCheck::check_rise(sim);
}
