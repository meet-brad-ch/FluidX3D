// A cube of water under turning gravity, using Setup API

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() { // cube with changing gravity; required extensions: FP16S, VOLUME_FORCE, SURFACE, INTERACTIVE_GRAPHICS
	const Length size = 10.0_cm; // 96³ cells, as the original
	const Acceleration g = 9.81_mps2;
	// the original lattice setup: gravity 0.001, viscosity 0.02, surface tension 0.001; the speed sqrt(g*size) sets the scale
	const float lbm_speed = sqrtf(0.001f * 96.0f);
	const Speed speed = sqrt(g * size);               // 0.99 m/s
	const float reynolds = lbm_speed * 96.0f / 0.02f; // 1490

	SimulationSetup sim(Domain::box(size, size, size).cell_size(size / 96.0f));
	sim.setup();
	sim.configure_units(speed, Fluid::WATER, lbm_speed);

	LBM lbm = sim.create_lbm_surface(speed * size / reynolds, g, sim.unit_scale().si_surface_tension(0.001f)); // the original lattice setup's

	SurfaceBuilder(lbm)
		.add_water(Shape::box({ 0_m, 0_m, 0_m }, { 2.0f / 3.0f * size, 2.0f / 3.0f * size, size }))
		.set_solid_walls()
		.apply();

	GraphicsConfig(lbm)
		.show_free_surface()
		.apply();

	// gravity turns every 0.8 s: down, toward +Y, up, toward -Y, then it is off for 1 s (the original's 2500 to 3000 time steps)
	const Acceleration none {};
	const auto set_gravity = [&](const AccelerationVector& a) {
		const float3 f = sim.to_lbm_acceleration(a);
		lbm.set_f(f.x, f.y, f.z);
	};
	lbm.run(0u); // initialize simulation
	while(true) { // main simulation loop
		set_gravity({ none, none, -g });
		lbm.run(sim.to_lbm_timesteps(0.8_s));
		set_gravity({ none, g, none });
		lbm.run(sim.to_lbm_timesteps(0.8_s));
		set_gravity({ none, none, g });
		lbm.run(sim.to_lbm_timesteps(0.8_s));
		set_gravity({ none, -g, none });
		lbm.run(sim.to_lbm_timesteps(0.65_s));
		set_gravity({ none, none, none });
		lbm.run(sim.to_lbm_timesteps(1.0_s));
	}
} /**/
