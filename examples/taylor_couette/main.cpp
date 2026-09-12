// Taylor-Couette flow between a turning inner and a resting outer cylinder, using Setup API

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() { // Taylor-Couette flow; required extensions: MOVING_BOUNDARIES, INTERACTIVE_GRAPHICS
	const Length width = 10.0_cm;       // the outer cylinder with its wall
	const Length cell = width / 96.0f;  // 96 x 96 x 192 cells, as the original
	const Length height = 2.0f * width;
	const Length outer_radius = 0.5f * width - cell, inner_radius = 0.25f * width;
	const Length gap = outer_radius - inner_radius;
	const float reynolds = 0.25f * 23.0f / 0.04f; // 144: the original lattice setup (surface speed 0.25, gap 23 cells, viscosity 0.04)
	const Speed surface_speed = reynolds * Fluid::WATER.kinematic_viscosity / gap; // of the inner cylinder: 6 mm/s in water

	SimulationSetup sim(Domain::box(width, width, height).cell_size(cell));
	sim.setup();
	sim.configure_units(surface_speed, Fluid::WATER, LatticeMach(0.433f)); // the original's lattice speed 0.25

	LBM lbm = sim.create_lbm(Fluid::WATER);

	const Position center { 0.5f * width, 0.5f * width, 0.5f * height };
	const Frequency turn_rate = surface_speed / inner_radius; // radians per second
	BoundaryBuilder(lbm)
		.add_solid(!Shape::cylinder(center, Axis::Z, outer_radius, height))
		.add_moving_solid(Shape::cylinder(center, Axis::Z, inner_radius, height), [=](Position p) {
			// clockwise about Z; a small axial wave, 2 gaps long, starts the Taylor vortices (the original: random noise)
			const Speed wave = 0.004f * surface_speed * sinf(pif * (p.z / gap));
			return Velocity{ turn_rate * (p.y - center.y), -turn_rate * (p.x - center.x), wave };
		})
		.apply();

	GraphicsConfig(lbm)
		.show_flags()
		.show_streamlines()
		.apply();

	lbm.run();
} /**/
