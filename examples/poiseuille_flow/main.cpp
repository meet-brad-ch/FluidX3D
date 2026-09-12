// Poiseuille flow validation in physical units, using Setup API: pipe flow (D2Q9: channel flow) driven by a pressure
// gradient, compared with the analytic profile u(r) = u_center*(1-r²/R²)

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() { // Poiseuille flow validation; required extensions: VOLUME_FORCE
	const Length radius = 1.0_cm;         // the pipe's (D2Q9: half the channel's width)
	const Length cell = radius / 63.0f;   // 63 cells, as the original
	const Length diameter = 2.0f * (radius + cell); // the domain across the pipe, with its wall
	const KinematicViscosity viscosity = Fluid::WATER.kinematic_viscosity;
	// the original lattice setup: center speed 0.1 at relaxation time 1 (lattice viscosity 1/6), so u = 0.6*nu/cell
	const Speed center_speed = 0.6f * viscosity / cell; // 3.8 mm/s in water

#ifndef D2Q9
	SimulationSetup sim(Domain::box(diameter, cell, diameter).cell_size(cell)); // 128 x 1 x 128 cells, periodic along Y
#else // D2Q9
	SimulationSetup sim(Domain::box(cell, diameter, cell).cell_size(cell)); // 1 x 128 x 1 cells, periodic along X
#endif // D2Q9
	sim.setup();
	sim.configure_units(center_speed, Fluid::WATER);

#ifndef D2Q9
	const Acceleration drive = 4.0f * center_speed * viscosity / (radius * radius); // pressure gradient per density
	LBM lbm = sim.create_lbm(viscosity, { Acceleration{}, drive, Acceleration{} });
	BoundaryBuilder(lbm)
		.add_solid(!Shape::cylinder({ 0.5f * diameter, 0.5f * cell, 0.5f * diameter }, Axis::Y, radius, cell))
		.apply();
#else // D2Q9
	const Acceleration drive = 2.0f * center_speed * viscosity / (radius * radius);
	LBM lbm = sim.create_lbm(viscosity, { drive, Acceleration{}, Acceleration{} });
	BoundaryBuilder(lbm)
		.set_solid_faces({ Face::Y_MIN, Face::Y_MAX })
		.apply();
#endif // D2Q9

	// the simulated velocities across the pipe against the analytic profile
	const uint Nx = lbm.get_Nx(), Ny = lbm.get_Ny(), Nz = lbm.get_Nz();
	const double R = radius.si(), u_center = center_speed.si(), dx = cell.si();
	double error_min = max_double;
	Runner runner(lbm);
	runner.every(4.0_s, [&](Duration t) { // about every 1000 time steps, as the original
		if(t == Duration{}) return; // no flow at the start
		lbm.u.read_from_device();
		double error_dif = 0.0, error_sum = 0.0;
		for(uint z = 0u; z < Nz; z++) {
			for(uint x = 0u; x < Nx; x++) {
#ifndef D2Q9
				const uint y = Ny / 2u;
				const double r = dx * sqrt(sq(x + 0.5f - 0.5f * (float)Nx) + sq(z + 0.5f - 0.5f * (float)Nz)); // from the pipe's axis, m
				if(r >= R) continue;
				const uint n = x + (y + z * Ny) * Nx;
#else // D2Q9
				for(uint y = 1u; y < Ny - 1u; y++) {
				const double r = dx * (y + 0.5f - 0.5f * (float)Ny); // from the channel's center, m
				const uint n = x + y * Nx;
#endif // D2Q9
				const double u_simulated = units.si_u(sqrt(sq(lbm.u.x[n]) + sq(lbm.u.y[n]) + sq(lbm.u.z[n]))); // m/s
				const double u_analytic = u_center * (1.0 - sq(r) / sq(R));
				error_dif += sq(u_simulated - u_analytic); // L2 error (Krüger p. 138)
				error_sum += sq(u_analytic);
#ifdef D2Q9
				}
#endif // D2Q9
			}
		}
		if(sqrt(error_dif / error_sum) >= error_min) { // stop when error has converged
			print_info("Poiseuille flow error converged after " + to_string(t.si(), 0u) + " s (" + to_string(lbm.get_t()) + " time steps) to " + to_string(100.0 * error_min, 3u) + "%"); // typical expected L2 errors: 2-5% (Krüger p. 256)
			wait();
			runner.stop();
			return;
		}
		error_min = fmin(error_min, sqrt(error_dif / error_sum));
		print_info("Poiseuille flow error after " + to_string(t.si(), 0u) + " s is " + to_string(100.0 * error_min, 3u) + "%"); // typical expected L2 errors: 2-5% (Krüger p. 256)
	});
	runner.run();
} /**/
