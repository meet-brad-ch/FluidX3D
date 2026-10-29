/// @file main.cpp
/// @brief Stokes drag validation in physical units: the force on a sphere in creeping flow of water (Reynolds number
/// 0.01), compared with Stokes' law F = 6 pi mu R u in newtons.

#include "setup/setup.hpp"

#include <cmath>

/// @brief Stokes' solution of creeping flow past a resting sphere: the velocity at a point. It is the formula of the
/// core's Units::u_Stokes(), in si:: quantities and double: with x the point from the sphere's centre, r = |x|, x^ = x / r
/// and u0 the far flow, u = u0 - 3/4 (R/r) [u0 (1 + R²/(3 r²)) + x^ (u0 . x^) (1 - R²/r²)] outside the sphere and 0
/// inside it. The result equals the core's float result within float rounding (a relative 1e-6 or less).
class StokesFlow {
public:
	/// @brief Makes the flow around one sphere.
	/// @param center the sphere's centre, in metres from the domain's origin corner
	/// @param radius the sphere's radius, more than 0 m
	/// @param far_flow the flow velocity far from the sphere
	StokesFlow(si::Position center, si::Length radius, si::Velocity far_flow) : center_(center), radius_(radius), far_flow_(far_flow) {}

	/// @brief Gives the flow velocity at a point.
	/// @param p the point, in metres from the domain's origin corner
	/// @return the velocity: Stokes' solution outside the sphere, 0 on and inside its surface
	si::Velocity velocity(si::Position p) const {
		const si::Position x = p - center_;
		const si::Length r = x.magnitude();
		if(r <= radius_) {
			const si::Velocity at_rest{};
			return at_rest;
		}
		const double R_over_r = au::as_raw_number(radius_ / r);
		const double R2_over_r2 = R_over_r * R_over_r;
		const double x_hat = au::as_raw_number(x.x / r);
		const double y_hat = au::as_raw_number(x.y / r);
		const double z_hat = au::as_raw_number(x.z / r);
		const si::Speed along_x_hat = far_flow_.x * x_hat + far_flow_.y * y_hat + far_flow_.z * z_hat; // u0 . x^
		const si::Velocity radial{ along_x_hat * x_hat, along_x_hat * y_hat, along_x_hat * z_hat };   // x^ (u0 . x^)
		const double far_factor = stokes_factor * R_over_r * (1.0 + R2_over_r2 / near_field_divisor);
		const double radial_factor = stokes_factor * R_over_r * (1.0 - R2_over_r2);
		const si::Velocity disturbance = far_flow_ * far_factor + radial * radial_factor;
		const si::Velocity flow = far_flow_ - disturbance;
		return flow;
	}

private:
	static constexpr double stokes_factor = 0.75;      ///< the 3/4 in front of the disturbance (a ratio)
	static constexpr double near_field_divisor = 3.0;  ///< the 3 in the term R²/(3 r²) (a ratio)

	si::Position center_{};   ///< the sphere's centre, in metres from the domain's origin corner
	si::Length radius_{};     ///< the sphere's radius
	si::Velocity far_flow_{}; ///< the flow velocity far from the sphere
};

/// @brief The check of the measured drag force against Stokes' law, and the stop rule on it: the run stops when the
/// relative error has converged (utilities.hpp converged() over the last three errors).
class DragError {
public:
	/// @brief Makes the check of a sphere's drag.
	/// @param sim the simulation; it must live as long as the check
	/// @param drag the force on the measured sphere; it must live as long as the check
	/// @param expected the force of Stokes' law, more than 0 N
	DragError(Simulation& sim, ForceAnalyzer& drag, si::Force expected) : sim_(sim), drag_(drag), expected_(expected) {}

	/// @brief Measures the force at a time, prints it with the expected force, and stops the simulation when the error
	/// has converged.
	/// @param t the simulated time since the start; at 0 s nothing is measured (no flow has developed)
	void check(si::Duration t) {
		using namespace si::symbols;
		if(t == si::Duration{}) { // no flow has developed at the start
			return;
		}
		const double F_theory = expected_.in(N);
		const si::Force simulated_force = drag_.force().magnitude();
		const double F_simulated = simulated_force.in(N);
		const double E0 = std::fabs(F_simulated - F_theory) / F_theory;
		const float time_s = static_cast<float>(t.in(s));
		const std::string time_text = to_string(time_s, 1u);
		const double F_theory_nN = expected_.in(nanonewtons);
		const double F_simulated_nN = simulated_force.in(nanonewtons);
		const std::string expected_text = to_string(F_theory_nN, 6u);
		const std::string measured_text = to_string(F_simulated_nN, 6u);
		const double error_percent = percent * E0;
		const std::string error_text = to_string(static_cast<float>(error_percent), 1u);
		print_info(time_text + " s, expected: " + expected_text + " nN, measured: " + measured_text + " nN, error = " + error_text + "%");
		if(converged(E2_, E1_, E0, convergence_threshold)) { // stop when error has sufficiently converged
			const std::string steps_text = to_string(sim_.lbm().get_t());
			const std::string final_error_text = to_string(error_percent, 1u);
			print_info("Error converged after " + time_text + " s (" + steps_text + " time steps) to " + final_error_text + "%");
			wait();
			sim_.stop();
		}
		E2_ = E1_;
		E1_ = E0;
	}

private:
	static constexpr double start_error = 1000.0;          ///< the two errors before the first measurement: large, so that the first checks do not converge (a ratio, as the original)
	static constexpr double convergence_threshold = 1E-4;  ///< the change of the error that counts as converged (converged()'s threshold, as the original)
	static constexpr auto nanonewtons = au::nano(au::newtons); ///< the printed force's unit: nN
	static constexpr double percent = 100.0;               ///< the percent of a ratio of 1, for the printed error

	Simulation& sim_;           ///< the simulation that the check stops
	ForceAnalyzer& drag_;       ///< the force on the measured sphere
	si::Force expected_{};      ///< the force of Stokes' law
	double E1_{ start_error };  ///< the error of the last check (a ratio)
	double E2_{ start_error };  ///< the error of the check before the last one (a ratio)
};

/// @brief Sets up a 1 mm sphere in a 8 mm box of 256³ cells with Stokes' solution as the start, prints the measured
/// and the expected force every 0.1 s of simulated time, and stops when the error has converged.
///
/// The lattice Mach number gives a lattice viscosity of 1 (relaxation time 3.5), as the original. The start flow is
/// Stokes' solution (StokesFlow: the formula of the core's Units::u_Stokes(), in si:: quantities).
void main_setup() { // extensions: FORCE_FIELD, EQUILIBRIUM_BOUNDARIES
	using namespace si::symbols;
	const si::Length radius = 1.0 * mm;        // the sphere's
	const double cells_per_radius = 32.0;      // 256³ cells, as the original
	const si::Length cell = radius / cells_per_radius;
	const double reynolds = 0.01;              // over the sphere's diameter
	const si::KinematicViscosity viscosity = Fluid::WATER.kinematic_viscosity;
	const si::Density density = Fluid::WATER.density;
	const si::Speed flow_speed = reynolds * viscosity / (2.0 * radius); // 5 micrometres per second in water

	const double box_in_radii = 8.0;           // the box's side, in sphere radii
	const si::Length box = box_in_radii * radius;
	const si::Length box_middle = 0.5 * box;
	const si::Duration check_interval = 0.1 * s; // the error about every 100 time steps, as the original
	Domain domain = Domain::box(box, box, box);
	domain.cell_size(cell);
	// lattice viscosity 1 (relaxation time 3.5), as the original: Mach = sqrt(3) Re / (2 R in cells)
	const double sphere_diameter_cells = 2.0 * cells_per_radius;
	const double lattice_mach = std::sqrt(3.0) * reynolds / sphere_diameter_cells;
	const si::LatticeMach mach(lattice_mach);
	Simulation sim(domain, Fluid::WATER, flow_speed, mach);

	// Stokes' solution around the sphere as the start, for the flow along -X; r from the sphere's center
	const si::Position center{ box_middle, box_middle, box_middle };
	const si::Velocity far_flow{ -flow_speed, si::Speed{}, si::Speed{} };
	const StokesFlow stokes_flow(center, radius, far_flow);
	const auto velocity = [=](si::Position p) {
		const si::Velocity flow = stokes_flow.velocity(p);
		return flow;
	};
	const auto pressure = [=](si::Position p) { // p = -3/2 mu R (u0 . r) / |r|³, mu = rho nu
		const si::Position d = p - center;
		const si::Length distance = d.magnitude();
		const auto u0_dot_r = -flow_speed * d.x;
		const si::Pressure p_flow = -1.5 * density * viscosity * radius * u0_dot_r / (distance * distance * distance);
		return p_flow;
	};
	const Shape sphere = Shape::sphere(center, radius);
	sim.boundaries()
		.set_open_boundaries()
		.add_solid(sphere, Solid::MEASURED)
		.initialize_velocity(velocity)
		.initialize_pressure(pressure)
		.apply();

	ForceAnalyzer& drag = sim.forces();
	const si::Force stokes_force = 6.0 * pi * density * viscosity * radius * flow_speed; // F = 6 pi mu R u
	DragError drag_error(sim, drag, stokes_force);
	sim.every(check_interval, [&](si::Duration t) {
		drag_error.check(t);
	});
	sim.run();
}
