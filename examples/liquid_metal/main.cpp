// Liquid metal on a vibrating speaker membrane

#include "setup/setup.hpp"

void main_setup() { // extensions: FP16S, VOLUME_FORCE, MOVING_BOUNDARIES, SURFACE, INTERACTIVE_GRAPHICS
	const Length width = 12.8_cm;          // the container; 128 x 128 x 96 cells, as the original
	const Length cell = width / 128.0f;
	const Length height = 0.75f * width, depth = height / 3.0f; // of the liquid: 3.2 cm
	const Density density = 6440.0_kgpm3;  // galinstan, a metal that is liquid at room temperature
	const Acceleration gravity = 9.81_mps2;
	// the original lattice setup: the membrane's peak speed 0.09 and frequency 0.01 per time step, gravity 0.0005,
	// depth 32 cells, viscosity 0.01, surface tension 0.005
	const float froude = 0.09f / sqrtf(0.0005f * 32.0f);
	const Speed peak_speed = froude * sqrt(gravity * depth);                   // 0.40 m/s
	const Frequency frequency = (0.01f * 32.0f / 0.09f) * peak_speed / depth; // 44 Hz
	const float reynolds = 0.09f * 32.0f / 0.01f;                             // 288 over the depth
	const FluidProperties galinstan { density, peak_speed * depth / reynolds };

	Simulation sim(Domain::box(width, width, height).cell_size(cell), galinstan, peak_speed,
	               LatticeMach(0.156f)); // the original's lattice speed 0.09
	sim.set_gravity(gravity).set_surface_tension(0.630829_Npm); // the original lattice setup's 0.005 at this scale

	sim.surface()
		.set_water_level(depth)
		.initialize_hydrostatic()
		.set_solid_box()
		.apply();

	// the speaker's membrane, the floor inside the walls, oscillates up and down
	sim.boundaries()
		.add_moving_solid(Shape::box({ cell, cell, 0_m }, { width - cell, width - cell, cell }), [=](Position, Duration t) {
			return Velocity{ Speed{}, Speed{}, peak_speed * sinf(2.0f * pif * (frequency * t)) };
		})
		.apply();

	sim.graphics()
		.show_free_surface()
		.apply();

	sim.run();
}
