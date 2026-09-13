// Poiseuille flow validation in physical units: pipe flow (D2Q9: channel flow) driven by a pressure gradient, compared
// with the analytic profile u(r) = u_center*(1-r²/R²)

#include "setup/setup.hpp"

void main_setup() { // extensions: VOLUME_FORCE
	const Length radius = 1.0_cm;         // the pipe's (D2Q9: half the channel's width)
	const Length cell = radius / 63.0f;   // 63 cells, as the original
	const Length diameter = 2.0f * (radius + cell); // the domain across the pipe, with its wall
	const KinematicViscosity viscosity = Fluid::WATER.kinematic_viscosity;
	// the original lattice setup: center speed 0.1 at relaxation time 1 (lattice viscosity 1/6), so u = 0.6*nu/cell
	const Speed center_speed = 0.6f * viscosity / cell; // 3.8 mm/s in water

#ifndef D2Q9
	Simulation sim(Domain::box(diameter, cell, diameter).cell_size(cell), Fluid::WATER, center_speed); // 128 x 1 x 128 cells, periodic along Y
	sim.set_body_force({ Acceleration{}, 4.0f * center_speed * viscosity / (radius * radius), Acceleration{} }); // the pressure gradient per density
	sim.boundaries()
		.add_solid(!Shape::cylinder({ 0.5f * diameter, 0.5f * cell, 0.5f * diameter }, Axis::Y, radius, cell))
		.apply();
	const auto radius_of = [=](const Position& p) { return magnitude(Position{ p.x - 0.5f * diameter, Length{}, p.z - 0.5f * diameter }); }; // from the pipe's axis
#else // D2Q9
	Simulation sim(Domain::box(cell, diameter, cell).cell_size(cell), Fluid::WATER, center_speed); // 1 x 128 x 1 cells, periodic along X
	sim.set_body_force({ 2.0f * center_speed * viscosity / (radius * radius), Acceleration{}, Acceleration{} });
	sim.boundaries()
		.set_solid_faces({ Face::Y_MIN, Face::Y_MAX })
		.apply();
	const auto radius_of = [=](const Position& p) { return magnitude(Position{ Length{}, p.y - 0.5f * diameter, Length{} }); }; // from the channel's center
#endif // D2Q9

	// the simulated velocities across the pipe against the analytic profile
	double error_min = max_double;
	sim.every(4.0_s, [&](Duration t) { // about every 1000 time steps, as the original
		if(t == Duration{}) return; // no flow at the start
		double error_dif = 0.0, error_sum = 0.0;
		sim.fields().for_each_cell([&](const FieldReader::Cell& c) {
			const Length r = radius_of(c.center);
			if(c.solid || r >= radius) return;
			const double u_simulated = magnitude(c.velocity).si();
			const double u_analytic = center_speed.si() * (1.0 - sq(r.si()) / sq(radius.si()));
			error_dif += sq(u_simulated - u_analytic); // L2 error (Krüger p. 138)
			error_sum += sq(u_analytic);
		});
		const double error = sqrt(error_dif / error_sum);
		if(error >= error_min) { // stop when error has converged
			print_info("Poiseuille flow error converged after " + to_string(t.si(), 0u) + " s (" + to_string(sim.lbm().get_t()) + " time steps) to " + to_string(100.0 * error_min, 3u) + "%"); // typical expected L2 errors: 2-5% (Krüger p. 256)
			wait();
			sim.stop();
			return;
		}
		error_min = fmin(error_min, error);
		print_info("Poiseuille flow error after " + to_string(t.si(), 0u) + " s is " + to_string(100.0 * error_min, 3u) + "%"); // typical expected L2 errors: 2-5% (Krüger p. 256)
	});
	sim.run();
}
