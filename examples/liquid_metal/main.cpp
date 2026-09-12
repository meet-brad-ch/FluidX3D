// Liquid metal on a vibrating speaker membrane, using Setup API

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() { // liquid metal on a speaker; required extensions: FP16S, VOLUME_FORCE, MOVING_BOUNDARIES, SURFACE, INTERACTIVE_GRAPHICS
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

	SimulationSetup sim(Domain::box(width, width, height).cell_size(cell));
	sim.setup();
	sim.configure_units(peak_speed, density, 0.09f);

	LBM lbm = sim.create_lbm_surface(peak_speed * depth / reynolds, gravity, sim.unit_scale().si_surface_tension(0.005f)); // the original lattice setup's

	SurfaceBuilder(lbm)
		.set_water_level(depth)
		.initialize_hydrostatic()
		.set_solid_walls()
		.apply();

	// the speaker's membrane, the floor inside the walls: a speed above zero makes it a moving wall from the start
	BoundaryBuilder(lbm)
		.add_moving_solid(Shape::box({ cell, cell, 0_m }, { width - cell, width - cell, cell }),
		                  [](Position) { return Velocity{ Speed{}, Speed{}, Speed::from_si(1E-12f) }; })
		.apply();

	GraphicsConfig(lbm)
		.show_free_surface()
		.apply();

	// the membrane oscillates up and down
	const uint Nx = lbm.get_Nx(), Ny = lbm.get_Ny();
	const float lbm_peak_speed = sim.to_lbm_velocity(peak_speed);
	const float lbm_frequency = frequency * sim.unit_scale().time_step(); // per time step
	lbm.run(0u); // initialize simulation
	while(true) { // main simulation loop
		lbm.u.read_from_device();
		const float uz = lbm_peak_speed * sinf(2.0f * pif * lbm_frequency * (float)lbm.get_t());
		for(uint y = 1u; y < Ny - 1u; y++) {
			for(uint x = 1u; x < Nx - 1u; x++) lbm.u.z[x + y * Nx] = uz;
		}
		lbm.u.write_to_device();
		lbm.run(1u);
	}
} /**/
