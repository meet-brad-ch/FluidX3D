// 2D Taylor-Green vortices

#include "setup/setup.hpp"

void main_setup() { // extensions: D2Q9, INTERACTIVE_GRAPHICS
	const Length size = 1.0_m;             // square, periodic
	const Length cell = size / 1024.0f;    // 1024 x 1024 cells, as the original
	const Length wavelength = size / 5.0f; // 5 x 5 vortex pairs
	const float reynolds = 0.2f * 204.8f / 0.02f; // 2048: the original lattice setup (amplitude 0.2, wavelength 204.8 cells, viscosity 0.02)
	const Speed amplitude = reynolds * Fluid::WATER.kinematic_viscosity / wavelength; // 1 cm/s in water
	const Density density = Fluid::WATER.density;

	Simulation sim(Domain::box(size, size, cell).cell_size(cell), Fluid::WATER, amplitude, // one cell high: 2D
	               LatticeMach(0.346f)); // the original's lattice speed 0.2

	// phases from the domain's center
	const auto phase = [=](Length position) { return 2.0f * pif * ((position - 0.5f * size) / wavelength); };
	sim.boundaries()
		.initialize_velocity([=](Position p) {
			const float x = phase(p.x), y = phase(p.y);
			return Velocity{ amplitude * (cosf(x) * sinf(y)), -amplitude * (sinf(x) * cosf(y)), Speed{} };
		})
		.initialize_pressure([=](Position p) {
			return -0.25f * density * amplitude * amplitude * (cosf(2.0f * phase(p.x)) + cosf(2.0f * phase(p.y)));
		})
		.apply();

	sim.graphics()
		.show_velocity_field()
		.set_slice_mode(SliceMode::Z)
		.apply();

	sim.run();
}
