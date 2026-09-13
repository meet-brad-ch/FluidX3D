// Poiseuille flow: water in a pipe driven by a pressure gradient, against the analytic profile u(r) = u_center*(1-r²/R²).
// Passes when the L2 error of the converged velocity profile is below 1 %.

#include "setup/setup.hpp"
#include "physics_check.hpp"

void main_setup() { // extensions: VOLUME_FORCE
	const Length radius = 1.0_cm;
	const Length cell = radius / 63.0f;             // 63 cells across the radius
	const Length diameter = 2.0f * (radius + cell); // the domain across the pipe, with its wall
	const KinematicViscosity viscosity = Fluid::WATER.kinematic_viscosity;
	const Speed center_speed = 0.6f * viscosity / cell; // with the default lattice Mach number: relaxation time 1

	Simulation sim(Domain::box(diameter, cell, diameter).cell_size(cell), Fluid::WATER, center_speed); // 128 x 1 x 128 cells, periodic along Y
	const Acceleration drive = 4.0f * center_speed * viscosity / (radius * radius); // the pressure gradient per density
	sim.set_body_force({ Acceleration{}, drive, Acceleration{} });
	sim.boundaries()
		.add_solid(!Shape::cylinder({ 0.5f * diameter, 0.5f * cell, 0.5f * diameter }, Axis::Y, radius, cell))
		.apply();

	// the L2 error of the simulated velocities across the pipe against the analytic profile (Krüger p. 138)
	LBM& lbm = sim.lbm();
	const uint Nx = lbm.get_Nx(), Nz = lbm.get_Nz();
	const double R = radius.si(), u_center = center_speed.si(), dx = cell.si();
	const auto profile_error = [&]() {
		lbm.u.read_from_device();
		double error_dif = 0.0, error_sum = 0.0;
		for(uint z = 0u; z < Nz; z++) {
			for(uint x = 0u; x < Nx; x++) {
				const double r = dx * sqrt(sq(x + 0.5f - 0.5f * (float)Nx) + sq(z + 0.5f - 0.5f * (float)Nz)); // from the axis, m
				if(r >= R) continue;
				const uint n = x + z * Nx;
				const double u_simulated = sim.unit_scale().si_velocity(sqrt(sq(lbm.u.x[n]) + sq(lbm.u.y[n]) + sq(lbm.u.z[n]))).si();
				const double u_analytic = u_center * (1.0 - sq(r) / sq(R));
				error_dif += sq(u_simulated - u_analytic);
				error_sum += sq(u_analytic);
			}
		}
		return sqrt(error_dif / error_sum);
	};

	double error_min = max_double;
	sim.every(4.0_s, [&](Duration t) { // about every 1000 time steps
		if(t == Duration{}) return; // no flow at the start
		const double error = profile_error();
		if(error >= error_min || t > 2000.0_s) { // converged (or not converging)
			PhysicsCheck::report("Poiseuille flow, velocity profile", error_min, 0.01);
			sim.stop();
		}
		error_min = fmin(error_min, error);
	});
	sim.run();
}
