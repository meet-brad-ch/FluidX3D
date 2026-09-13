// 3D Taylor-Green vortices

#include "setup/setup.hpp"

void main_setup() { // extensions: INTERACTIVE_GRAPHICS
	const Length size = 1.0_m;         // cube, periodic: one vortex wavelength
	const float reynolds = 0.25f * 128.0f / 0.01f; // 3200: the original lattice setup (amplitude 0.25, 128 cells, viscosity 0.01)
	const Speed amplitude = reynolds * Fluid::WATER.kinematic_viscosity / size; // 3.2 mm/s in water
	const Density density = Fluid::WATER.density;

	Simulation sim(Domain::box(size, size, size).cell_size(size / 128.0f), Fluid::WATER, amplitude, // 128³ cells, as the original
	               LatticeMach(0.433f)); // the original's lattice speed 0.25

	// the classic Taylor-Green vortex (no flow along Z, its pressure), with phases from the domain's center; the
	// original's Z velocity made the start compressible and its pressure was that of the 2D vortices
	const auto phase = [=](Length position) { return 2.0f * pif * ((position - 0.5f * size) / size); };
	sim.boundaries()
		.initialize_velocity([=](Position p) {
			const float x = phase(p.x), y = phase(p.y), z = phase(p.z);
			return Velocity{ amplitude * (cosf(x) * sinf(y) * sinf(z)), -amplitude * (sinf(x) * cosf(y) * sinf(z)), Speed{} };
		})
		.initialize_pressure([=](Position p) {
			const float x = phase(p.x), y = phase(p.y), z = phase(p.z);
			return -density * amplitude * amplitude * ((cosf(2.0f * x) + cosf(2.0f * y)) * (2.0f - cosf(2.0f * z)) / 16.0f);
		})
		.apply();

	sim.graphics()
		.show_streamlines()
		.apply();

	sim.run();
}
