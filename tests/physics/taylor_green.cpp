// 2D Taylor-Green vortices in water: u = A*sin(kx)*cos(ky), v = -A*cos(kx)*sin(ky) is an exact solution of the
// Navier-Stokes equations whose kinetic energy decays as exp(-4*nu*k²*t).
// Passes when the viscosity measured from the decay is within 1 % of water's.

#include "setup/setup.hpp"
#include "physics_check.hpp"

void main_setup() { // extensions: none (D3Q19, one cell high)
	const Length size = 1.0_cm;         // a periodic square, one wavelength
	const Length cell = size / 128.0f;  // 128 x 128 x 1 cells
	const KinematicViscosity viscosity = Fluid::WATER.kinematic_viscosity;
	const Density density = Fluid::WATER.density;
	const Speed amplitude = 100.0f * viscosity / size; // Reynolds number 100
	const float k = 2.0f * pif / size.si();            // wave number, 1/m
	const Duration decay_time = 1.0f / (2.0f * k * k * viscosity.si()) * 1.0_s; // the velocity decays to 1/e

	Simulation sim(Domain::box(size, size, cell).cell_size(cell), Fluid::WATER, amplitude);

	const auto phase = [=](Length position) { return 2.0f * pif * (position / size); };
	sim.boundaries()
		.initialize_velocity([=](Position p) {
			const float x = phase(p.x), y = phase(p.y);
			return Velocity{ amplitude * (sinf(x) * cosf(y)), -amplitude * (cosf(x) * sinf(y)), Speed{} };
		})
		.initialize_pressure([=](Position p) {
			return 0.25f * density * amplitude * amplitude * (cosf(2.0f * phase(p.x)) + cosf(2.0f * phase(p.y)));
		})
		.apply();

	const auto kinetic_energy = [&]() { // per density, summed over the cells: only its decay is compared
		double energy = 0.0;
		sim.fields().for_each_cell([&](const FieldReader::Cell& c) { energy += sq((double)magnitude(c.velocity).si()); });
		return energy;
	};

	double initial_energy = 0.0;
	sim.every(decay_time, [&](Duration t) {
		if(t == Duration{}) {
			initial_energy = kinetic_energy();
			return;
		}
		const double measured_viscosity = -log(kinetic_energy() / initial_energy) / (4.0 * sq((double)k) * (double)t.si()); // m²/s
		PhysicsCheck::report("Taylor-Green vortices, viscosity from the decay", fabs(measured_viscosity / viscosity.si() - 1.0), 0.01);
		sim.stop();
	});
	sim.run();
}
