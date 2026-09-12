// 3D Taylor-Green vortices, using Setup API

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() { // 3D Taylor-Green vortices; required extensions: INTERACTIVE_GRAPHICS
	const Length size = 1.0_m;         // cube, periodic: one vortex wavelength
	const float reynolds = 0.25f * 128.0f / 0.01f; // 3200: the original lattice setup (amplitude 0.25, 128 cells, viscosity 0.01)
	const Speed amplitude = reynolds * Fluid::WATER.kinematic_viscosity / size; // 3.2 mm/s in water
	const Density density = Fluid::WATER.density;

	SimulationSetup sim(Domain::box(size, size, size).cell_size(size / 128.0f)); // 128³ cells, as the original
	sim.setup();
	sim.configure_units(amplitude, Fluid::WATER, 0.25f);

	LBM lbm = sim.create_lbm(Fluid::WATER);

	// the classic Taylor-Green vortex (no flow along Z, its pressure), with phases from the domain's center; the
	// original's Z velocity made the start compressible and its pressure was that of the 2D vortices
	const auto phase = [=](Length position) { return 2.0f * pif * ((position - 0.5f * size) / size); };
	BoundaryBuilder(lbm)
		.initialize_velocity([=](Position p) {
			const float x = phase(p.x), y = phase(p.y), z = phase(p.z);
			return Velocity{ amplitude * (cosf(x) * sinf(y) * sinf(z)), -amplitude * (sinf(x) * cosf(y) * sinf(z)), Speed{} };
		})
		.initialize_pressure([=](Position p) {
			const float x = phase(p.x), y = phase(p.y), z = phase(p.z);
			return -density * amplitude * amplitude * ((cosf(2.0f * x) + cosf(2.0f * y)) * (2.0f - cosf(2.0f * z)) / 16.0f);
		})
		.apply();

	GraphicsConfig(lbm)
		.show_streamlines()
		.apply();

	lbm.run();
} /**/
