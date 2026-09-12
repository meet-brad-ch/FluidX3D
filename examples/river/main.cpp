// A river flowing around two pillars, using Setup API

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() { // river; required extensions: FP16S, VOLUME_FORCE, SURFACE, INTERACTIVE_GRAPHICS
	const Length width = 1.0_m;
	const Length cell = width / 128.0f; // 128 x 384 x 96 cells, as the original
	const Length length = 3.0f * width, height = 0.75f * width;
	const Length depth = 32.0f * cell;  // 0.25 m of water
	const Acceleration gravity = 9.81_mps2;
	const float slope = 0.14f;          // of the river's bed: 14 % of gravity pulls downstream (-Y), as the original
	// the original lattice setup: flow speed 0.1, gravity 0.0005, depth 32 cells, viscosity 0.02, surface tension 0.01
	const float froude = 0.1f / sqrtf(0.0005f * 32.0f);
	const Speed flow_speed = froude * sqrt(gravity * depth); // 1.2 m/s
	const float reynolds = 0.1f * 32.0f / 0.02f; // 160 over the depth; water's own viscosity would need a much finer grid

	SimulationSetup sim(Domain::box(width, length, height).cell_size(cell));
	sim.setup();
	sim.configure_units(flow_speed, Fluid::WATER, 0.1f);

	LBM lbm = sim.create_lbm_surface(flow_speed * depth / reynolds, { Acceleration{}, -slope * gravity, -gravity },
	                                 sim.unit_scale().si_surface_tension(0.01f)); // the original lattice setup's

	const Length pillar = 20.0f * cell; // radius
	SurfaceBuilder(lbm)
		.set_water_level(depth, { Speed{}, -flow_speed, Speed{} })
		.set_solid_faces({ Face::X_MIN, Face::X_MAX, Face::Z_MIN }) // periodic along Y: the river flows around
		.add_solid(Shape::cylinder({ 2.0f / 3.0f * width, 2.0f / 3.0f * length, 0.5f * height }, Axis::Z, pillar, height))
		.add_solid(Shape::box({ width / 3.0f - pillar, length / 3.0f - pillar, 0_m }, { width / 3.0f + pillar, length / 3.0f + pillar, height }))
		.apply();

	GraphicsConfig(lbm)
		.show_free_surface()
		.apply();

	lbm.run();
} /**/
