// A cube of water under turning gravity

#include "setup/setup.hpp"

void main_setup() { // extensions: FP16S, VOLUME_FORCE, SURFACE, INTERACTIVE_GRAPHICS
	const Length size = 10.0_cm; // 96³ cells, as the original
	const Acceleration g = 9.81_mps2;
	// the original lattice setup: gravity 0.001, viscosity 0.02, surface tension 0.001; the speed sqrt(g*size) sets the scale
	const float lbm_speed = sqrtf(0.001f * 96.0f);
	const Speed speed = sqrt(g * size);               // 0.99 m/s
	const float reynolds = lbm_speed * 96.0f / 0.02f; // 1490

	Simulation sim(Domain::box(size, size, size).cell_size(size / 96.0f),
	               Fluid::WATER.with_viscosity(speed * size / reynolds), speed,
	               LatticeMach(sqrtf(3.0f) * lbm_speed)); // the original's lattice speed
	sim.set_gravity(g).set_surface_tension(0.0106254_Npm); // the original lattice setup's 0.001 at this scale

	sim.surface()
		.add_water(Shape::box({ 0_m, 0_m, 0_m }, { 2.0f / 3.0f * size, 2.0f / 3.0f * size, size }))
		.set_solid_box()
		.apply();

	sim.graphics()
		.show_free_surface()
		.apply();

	// gravity turns every 0.8 s: down, toward +Y, up, toward -Y, then it is off for 1 s (the original's 2500 to 3000 time steps)
	const Acceleration none {};
	while(true) { // main simulation loop
		sim.set_body_force({ none, none, -g });
		sim.run_for(0.8_s);
		sim.set_body_force({ none, g, none });
		sim.run_for(0.8_s);
		sim.set_body_force({ none, none, g });
		sim.run_for(0.8_s);
		sim.set_body_force({ none, -g, none });
		sim.run_for(0.65_s);
		sim.set_body_force({ none, none, none });
		sim.run_for(1.0_s);
	}
}
