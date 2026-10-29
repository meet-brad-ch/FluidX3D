/// @file main.cpp
/// @brief Poiseuille flow validation in physical units: pipe flow (D2Q9: channel flow) driven by a pressure gradient,
/// compared with the analytic profile u(r) = u_center (1 - r²/R²).

#include "setup/setup.hpp"

#include <cmath>

/// @brief The L2 error of the simulated velocity profile across the pipe against the analytic profile
/// u(r) = u_center (1 - r²/R²), and the stop rule on it: the run stops when the error does not fall any more.
class ProfileError {
public:
	/// @brief Makes the check of a pipe (D2Q9: a channel) around the domain's middle.
	/// @param sim the simulation whose velocity field is read; it must live as long as the check
	/// @param radius the pipe's radius (D2Q9: half the channel's width), more than 0 m
	/// @param half the distance of the pipe's axis (D2Q9: the channel's middle) from the domain's origin corner
	/// @param center_speed the analytic profile's speed on the axis
	ProfileError(Simulation& sim, si::Length radius, si::Length half, si::Speed center_speed)
		: sim_(sim), radius_(radius), half_(half), center_speed_(center_speed) {}

	/// @brief Measures the error at a time, prints it, and stops the simulation when the error is not less than the
	/// smallest error measured before.
	/// @param t the simulated time since the start; at 0 s nothing is measured (no flow yet)
	void check(si::Duration t) {
		if(t == si::Duration{}) { // no flow at the start
			return;
		}
		const double error = measure();
		const float time_s = static_cast<float>(t.in(si::symbols::s));
		const std::string time_text = to_string(time_s, 0u);
		const double error_percent = percent * error_min_;
		const std::string error_text = to_string(error_percent, 3u);
		if(error >= error_min_) { // stop when error has converged
			const std::string steps_text = to_string(sim_.lbm().get_t());
			print_info("Poiseuille flow error converged after " + time_text + " s (" + steps_text + " time steps) to " + error_text + "%"); // typical expected L2 errors: 2-5% (Krüger p. 256)
			wait();
			sim_.stop();
			return;
		}
		error_min_ = std::fmin(error_min_, error);
		const double new_error_percent = percent * error_min_;
		const std::string new_error_text = to_string(new_error_percent, 3u);
		print_info("Poiseuille flow error after " + time_text + " s is " + new_error_text + "%"); // typical expected L2 errors: 2-5% (Krüger p. 256)
	}

private:
	/// @brief Gives the distance of a point from the pipe's axis (D2Q9: from the channel's middle plane).
	/// @param p the point, in metres from the domain's origin corner
	/// @return the distance
	si::Length radius_of(const si::Position& p) const {
#ifndef D2Q9
		const si::Position across{ p.x - half_, si::Length{}, p.z - half_ };
#else // D2Q9
		const si::Position across{ si::Length{}, p.y - half_, si::Length{} };
#endif // D2Q9
		const si::Length r = across.magnitude();
		return r;
	}

	/// @brief Measures the L2 error of the fluid cells inside the pipe (Krüger p. 138).
	/// @return the error, a ratio: the root of the summed squared differences over the summed squared analytic speeds
	double measure() {
		using namespace si::symbols;
		double error_dif{ 0.0 };
		double error_sum{ 0.0 };
		const double center_mps{ center_speed_.in(m / s) };
		const FieldReader fields{ sim_.fields() };
		const std::uint64_t count{ fields.cell_count() };
		for(std::uint64_t n{ 0u }; n < count; ++n) {
			const FieldReader::Cell c{ fields.cell(n) };
			const si::Length r{ radius_of(c.center) };
			if(c.solid || r >= radius_) {
				continue;
			}
			const double u_simulated{ c.velocity.magnitude().in(m / s) };
			const double r_over_radius{ au::as_raw_number(r / radius_) };
			const double u_analytic{ center_mps * (1.0 - sq(r_over_radius)) };
			error_dif += sq(u_simulated - u_analytic); // L2 error (Krüger p. 138)
			error_sum += sq(u_analytic);
		}
		const double error{ std::sqrt(error_dif / error_sum) };
		return error;
	}

	static constexpr double percent = 100.0; ///< the percent of a ratio of 1, for the printed error

	Simulation& sim_;             ///< the simulation whose velocity field is read
	si::Length radius_{};         ///< the pipe's radius (D2Q9: half the channel's width)
	si::Length half_{};           ///< the axis's distance from the domain's origin corner
	si::Speed center_speed_{};    ///< the analytic profile's speed on the axis
	double error_min_{ max_double }; ///< the smallest error measured so far (a ratio)
};

/// @brief Drives water through a 1 cm pipe of 63 cells radius with a body force and prints the L2 error against the
/// analytic profile every 4 s of simulated time, until the error stops falling.
///
/// The center speed is the original example's lattice setup: 0.1 cells per time step at a relaxation time of 1.
void main_setup() { // extensions: VOLUME_FORCE
	using namespace si::symbols;
	const si::Length radius = 1.0 * cm;               // the pipe's (D2Q9: half the channel's width)
	const double cells_per_radius = 63.0;             // 63 cells, as the original
	const si::Length cell = radius / cells_per_radius;
	const si::Length diameter = 2.0 * (radius + cell); // the domain across the pipe, with its wall
	const si::KinematicViscosity viscosity = Fluid::WATER.kinematic_viscosity;
	// the original lattice setup: center speed 0.1 at relaxation time 1 (lattice viscosity 1/6), so u = 0.6*nu/cell
	const double lattice_speed_over_viscosity = 0.6;
	const si::Speed center_speed = lattice_speed_over_viscosity * viscosity / cell; // 3.8 mm/s in water
	const si::Length half = 0.5 * diameter;
	const si::Duration check_interval = 4.0 * s; // about every 1000 time steps, as the original

#ifndef D2Q9
	Domain domain = Domain::box(diameter, cell, diameter); // 128 x 1 x 128 cells, periodic along Y
	domain.cell_size(cell);
	Simulation sim(domain, Fluid::WATER, center_speed);
	const double pipe_drive_factor = 4.0; // the pressure gradient per density of pipe flow: 4 u nu / R²
	const si::Acceleration drive = pipe_drive_factor * center_speed * viscosity / (radius * radius);
	const si::AccelerationVector body_force{ si::Acceleration{}, drive, si::Acceleration{} };
	sim.set_body_force(body_force);
	const si::Position axis_center{ half, 0.5 * cell, half };
	const Shape pipe_wall = !Shape::cylinder(axis_center, Axis::Y, radius, cell);
	sim.boundaries()
		.add_solid(pipe_wall)
		.apply();
#else // D2Q9
	Domain domain = Domain::box(cell, diameter, cell); // 1 x 128 x 1 cells, periodic along X
	domain.cell_size(cell);
	Simulation sim(domain, Fluid::WATER, center_speed);
	const double channel_drive_factor = 2.0; // the pressure gradient per density of channel flow: 2 u nu / R²
	const si::Acceleration drive = channel_drive_factor * center_speed * viscosity / (radius * radius);
	const si::AccelerationVector body_force{ drive, si::Acceleration{}, si::Acceleration{} };
	sim.set_body_force(body_force);
	sim.boundaries()
		.set_solid_faces({ Face::Y_MIN, Face::Y_MAX })
		.apply();
#endif // D2Q9

	// the simulated velocities across the pipe against the analytic profile
	ProfileError profile_error(sim, radius, half, center_speed);
	sim.every(check_interval, [&](si::Duration t) {
		profile_error.check(t);
	});
	sim.run();
}
