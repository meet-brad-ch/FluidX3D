/// @file main.cpp
/// @brief A cube of water under turning gravity: gravity points down, sideways, up and sideways in turn, then is off
/// (free surface, interactive graphics).

#include "setup/setup.hpp"

#include <cmath>

/// @brief Sets up a 10 cm box of 96³ cells with a block of water and turns gravity in a loop until the window is
/// closed.
///
/// The speed sqrt(g size) sets the scale; the Reynolds number, the lattice speed and the surface tension are the
/// original example's lattice setup (gravity 0.001, a lattice viscosity of 0.02, a lattice surface tension of 0.001).
void main_setup() { // extensions: FP16S, VOLUME_FORCE, SURFACE, INTERACTIVE_GRAPHICS
	using namespace si::symbols;
	const si::Length size = 10.0 * cm; // 96³ cells, as the original
	const si::Acceleration g = 9.81 * m / (s * s);
	const double cells = 96.0;
	const double lbm_speed = std::sqrt(0.001 * 96.0);
	const si::Speed speed = au::sqrt(g * size);      // 0.99 m/s
	const double reynolds = lbm_speed * 96.0 / 0.02; // 1490

	Domain domain = Domain::box(size, size, size);
	const si::Length cell_size = size / cells;
	domain.cell_size(cell_size);
	const FluidProperties fluid = Fluid::WATER.with_viscosity(speed * size / reynolds);
	const si::LatticeMach mach(std::sqrt(3.0) * lbm_speed); // the original's lattice speed
	Simulation sim(domain, fluid, speed, mach);
	const si::SurfaceTension surface_tension = 0.0106254 * N / m; // the original lattice setup's 0.001 at this scale
	sim.set_gravity(g).set_surface_tension(surface_tension);

	const si::Position block_from{ 0.0 * m, 0.0 * m, 0.0 * m };
	const si::Position block_to{ 2.0 / 3.0 * size, 2.0 / 3.0 * size, size };
	sim.surface()
		.add_water(Shape::box(block_from, block_to))
		.set_solid_box()
		.apply();

	sim.graphics()
		.show_free_surface()
		.apply();

	// gravity turns every 0.8 s: down, toward +Y, up, toward -Y, then it is off for 1 s (the original's 2500 to 3000 time steps)
	const si::Acceleration none{};
	const si::AccelerationVector down{ none, none, -g };
	const si::AccelerationVector forward{ none, g, none };
	const si::AccelerationVector up{ none, none, g };
	const si::AccelerationVector backward{ none, -g, none };
	const si::AccelerationVector off{ none, none, none };
	const si::Duration turn_time = 0.8 * s;
	const si::Duration backward_time = 0.65 * s;
	const si::Duration off_time = 1.0 * s;
	// an event loop, as the original: closing the window ends the program (the core's window handler calls exit())
	while(true) {
		sim.set_body_force(down);
		sim.run_for(turn_time);
		sim.set_body_force(forward);
		sim.run_for(turn_time);
		sim.set_body_force(up);
		sim.run_for(turn_time);
		sim.set_body_force(backward);
		sim.run_for(backward_time);
		sim.set_body_force(off);
		sim.run_for(off_time);
	}
}
