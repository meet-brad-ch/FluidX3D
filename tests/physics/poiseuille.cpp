// Poiseuille flow: water in a pipe driven by a pressure gradient, against the analytic profile u(r) = u_center*(1-r²/R²).
// Passes when the L2 error of the converged velocity profile is below 1 %.

#include "setup/setup.hpp"
#include "physics_check.hpp"

#include <cmath>

namespace {

/// @brief Measures the L2 error of the velocity profile across the pipe against the analytic profile (Krüger p. 138)
/// until it stops falling, and reports the smallest error.
class ProfileCheck {
public:
	/// @brief Makes the check of a pipe along Y.
	/// @param sim the simulation of the pipe (not owned, outlives the check)
	/// @param radius the pipe's inner radius
	/// @param half the axis's distance from the domain's origin corner along X and Z
	/// @param center_speed the analytic profile's speed on the axis
	ProfileCheck(Simulation& sim, si::Length radius, si::Length half, si::Speed center_speed)
		: sim_(sim), radius_(radius), half_(half), center_speed_(center_speed) {}

	/// @brief Measures the error; reports the smallest and stops the run when the error no longer falls or the run
	/// took too long.
	/// @param t the simulated time
	void at(si::Duration t) {
		if(t == si::Duration{}) { // no flow at the start
			return;
		}
		const double error{ profile_error() };
		const bool too_long{ t > time_limit };
		if(error >= error_min_ || too_long) { // converged (or not converging)
			PhysicsCheck::report("Poiseuille flow, velocity profile", error_min_, error_limit);
			sim_.stop();
		}
		error_min_ = std::fmin(error_min_, error);
	}

	static constexpr double error_limit = 0.01; ///< the largest L2 error that passes (1 %)

private:
	/// @brief Gives the L2 error of the simulated velocities across the pipe.
	/// @return the error, a ratio: the root of the summed squared differences over the summed squared analytic speeds
	double profile_error() const {
		using namespace si::symbols;
		double error_dif{ 0.0 };
		double error_sum{ 0.0 };
		const double center_mps{ center_speed_.in(m / s) };
		const FieldReader fields = sim_.fields();
		const std::uint64_t count{ fields.cell_count() };
		for(std::uint64_t n{ 0u }; n < count; ++n) {
			const FieldReader::Cell c{ fields.cell(n) };
			const si::Position across{ c.center.x - half_, si::Length{}, c.center.z - half_ };
			const si::Length r = across.magnitude(); // from the axis
			if(c.solid || r >= radius_) {
				continue;
			}
			const double r_over_radius{ au::as_raw_number(r / radius_) };
			const double u_analytic{ center_mps * (1.0 - sq(r_over_radius)) };
			const double u_simulated{ c.velocity.magnitude().in(m / s) };
			error_dif += sq(u_simulated - u_analytic);
			error_sum += sq(u_analytic);
		}
		const double error{ std::sqrt(error_dif / error_sum) };
		return error;
	}

	static constexpr si::Duration time_limit = au::seconds(2000.0); ///< the time after which the check reports anyway

	Simulation& sim_;                ///< the simulation (not owned)
	si::Length radius_{};            ///< the pipe's inner radius
	si::Length half_{};              ///< the axis's distance from the origin corner along X and Z
	si::Speed center_speed_{};       ///< the analytic speed on the axis
	double error_min_{ max_double }; ///< the smallest error measured so far (a ratio)
};

} // namespace

void main_setup() { // extensions: VOLUME_FORCE
	using namespace si::symbols;
	const si::Length radius = 1.0 * cm;
	const si::Length cell = radius / 63.0;              // 63 cells across the radius
	const si::Length diameter = 2.0 * (radius + cell);  // the domain across the pipe, with its wall
	const si::KinematicViscosity viscosity = Fluid::WATER.kinematic_viscosity;
	const si::Speed center_speed = 0.6 * viscosity / cell; // with the default lattice Mach number: relaxation time 1
	const si::Length half = 0.5 * diameter;

	Domain domain = Domain::box(diameter, cell, diameter); // 128 x 1 x 128 cells, periodic along Y
	domain.cell_size(cell);
	Simulation sim(domain, Fluid::WATER, center_speed);
	const si::Acceleration drive = 4.0 * center_speed * viscosity / (radius * radius); // the pressure gradient per density
	const si::AccelerationVector body_force{ si::Acceleration{}, drive, si::Acceleration{} };
	sim.set_body_force(body_force);
	const si::Position axis_center{ half, 0.5 * cell, half };
	const Shape pipe_wall = !Shape::cylinder(axis_center, Axis::Y, radius, cell);
	sim.boundaries()
		.add_solid(pipe_wall)
		.apply();

	ProfileCheck check(sim, radius, half, center_speed);
	const si::Duration check_interval = 4.0 * s; // about every 1000 time steps
	sim.every(check_interval, [&check](si::Duration t) { check.at(t); });
	sim.run();
}
