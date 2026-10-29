// Stokes drag: the force on a sphere in creeping flow of water (Re 0.01), against Stokes' law F = 6*pi*mu*R*u in newtons.
// Passes when the converged force is within 5 % of Stokes' law.

#include "setup/setup.hpp"
#include "physics_check.hpp"

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

/// @brief The check of the measured drag force against Stokes' law: it reports the error (PhysicsCheck) and stops the
/// run when the error has converged (utilities.hpp converged()) or the time limit has passed.
class DragCheck {
public:
	/// @brief Makes the check of a sphere's drag.
	/// @param sim the simulation; it must live as long as the check
	/// @param drag the force on the measured sphere; it must live as long as the check
	/// @param expected the force of Stokes' law, more than 0 N
	DragCheck(Simulation& sim, ForceAnalyzer& drag, si::Force expected) : sim_(sim), drag_(drag), expected_(expected) {}

	/// @brief Measures the force at a time, and reports and stops when the error has converged or the time is up.
	/// @param t the simulated time since the start; at 0 s nothing is measured (no flow has developed)
	void check(si::Duration t) {
		using namespace si::symbols;
		if(t == si::Duration{}) { // no flow has developed at the start
			return;
		}
		const double F_stokes = expected_.in(N);
		const double F_simulated = drag_.force().magnitude().in(N);
		const double E0 = std::fabs(F_simulated - F_stokes) / F_stokes;
		const bool too_long = t > time_limit;
		const bool error_converged = converged(E2_, E1_, E0, convergence_threshold);
		if(error_converged || too_long) { // converged (or not converging)
			PhysicsCheck::report("Stokes drag, force on the sphere", E0, pass_limit);
			sim_.stop();
		}
		E2_ = E1_;
		E1_ = E0;
	}

private:
	static constexpr double start_error = 1000.0;          ///< the errors before the first check: large, so that it does not converge (a ratio)
	static constexpr double convergence_threshold = 1E-4;  ///< the change of the error that counts as converged (a ratio)
	static constexpr double pass_limit = 0.05;             ///< the largest relative error that passes (5 %)
	static constexpr si::Duration time_limit = au::seconds(30.0); ///< the check ends here when the error does not converge

	Simulation& sim_;          ///< the simulation that the check stops
	ForceAnalyzer& drag_;      ///< the force on the measured sphere
	si::Force expected_{};     ///< the force of Stokes' law
	double E1_{ start_error }; ///< the error of the last check (a ratio)
	double E2_{ start_error }; ///< the error of the check before the last one (a ratio)
};

void main_setup() { // extensions: FORCE_FIELD, EQUILIBRIUM_BOUNDARIES
	using namespace si::symbols;
	const si::Length radius = 1.0 * mm;
	const si::Length cell = radius / 32.0; // 256³ cells
	const double reynolds = 0.01;          // over the diameter
	const si::KinematicViscosity viscosity = Fluid::WATER.kinematic_viscosity;
	const si::Density density = Fluid::WATER.density;
	const si::Speed flow_speed = reynolds * viscosity / (2.0 * radius);

	const si::Length box = 8.0 * radius;
	Domain domain = Domain::box(box, box, box);
	domain.cell_size(cell);
	const si::LatticeMach mach(std::sqrt(3.0) * reynolds / 64.0); // lattice viscosity 1
	Simulation sim(domain, Fluid::WATER, flow_speed, mach);

	// Stokes' solution around the sphere as the start, for the flow along -X
	const si::Position center{ 4.0 * radius, 4.0 * radius, 4.0 * radius };
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
	sim.boundaries()
		.set_open_boundaries()
		.add_solid(Shape::sphere(center, radius), Solid::MEASURED)
		.initialize_velocity(velocity)
		.initialize_pressure(pressure)
		.apply();

	ForceAnalyzer& drag = sim.forces();
	const si::Force stokes_force = 6.0 * pi * density * viscosity * radius * flow_speed; // F = 6 pi mu R u
	const si::Duration check_interval = 0.1 * s; // about every 100 time steps
	DragCheck drag_check(sim, drag, stokes_force);
	sim.every(check_interval, [&](si::Duration t) {
		drag_check.check(t);
	});
	sim.run();
}
