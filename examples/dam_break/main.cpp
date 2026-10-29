/// @file main.cpp
/// @brief Dam break: a column of water collapses in a box (free surface, interactive graphics).

#include "setup/setup.hpp"

#include <cmath>

/// @brief Sets up a 0.5 x 1 x 1 m box of 128 x 256 x 256 cells with a water column 0.75 m high at one wall, and runs
/// until the window is closed.
///
/// The Reynolds number, the lattice speed and the surface tension are the original example's lattice setup (the
/// front speed, a water height of 192 cells, a lattice viscosity of 0.005, a lattice surface tension of 0.0001);
/// water's own viscosity (Reynolds number about 3E6) would need a much finer grid.
void main_setup() { // extensions: FP16S, VOLUME_FORCE, SURFACE, INTERACTIVE_GRAPHICS
	using namespace si::symbols;
	const si::Length domain_x = 0.5 * m;
	const si::Length domain_y = 1.0 * m;
	const si::Length domain_z = 1.0 * m;
	const si::Length water_height = 0.75 * domain_z; // water column: 6/8 of the height and 1/8 of the length
	const si::Acceleration gravity = 9.81 * m / (s * s);
	const si::Speed front_speed = au::sqrt(2.0 * gravity * water_height); // dam-break front speed, the fastest velocity in the flow

	// Reynolds number of the original lattice setup (front speed, water height 192 cells, viscosity 0.005)
	const double reynolds = std::sqrt(2.0 * 0.0002 * 192.0) * 192.0 / 0.005; // about 10600
	const si::KinematicViscosity kinematic_viscosity = front_speed * water_height / reynolds;

	const double cells_y = 256.0; // 128 x 256 x 256 cells, as the original
	const si::Length cell_size = domain_y / cells_y;
	Domain domain = Domain::box(domain_x, domain_y, domain_z);
	domain.cell_size(cell_size);
	const FluidProperties fluid = Fluid::WATER.with_viscosity(kinematic_viscosity);
	const si::LatticeMach mach(0.48); // the front speed as in the original lattice setup (0.28 cells per time step)
	Simulation sim(domain, fluid, front_speed, mach);
	const si::SurfaceTension surface_tension = 0.0747098 * N / m; // the original lattice setup's 0.0001 at this scale
	sim.set_gravity(gravity).set_surface_tension(surface_tension);

	// water column at the y = 0 wall; uniform density, as the original
	const si::Position column_from{ 0.0 * m, 0.0 * m, 0.0 * m };
	const si::Position column_to{ domain_x, domain_y / 8.0, water_height };
	sim.surface()
		.add_water(Shape::box(column_from, column_to))
		.set_solid_box()
		.apply();

	sim.graphics()
		.show_free_surface()
		.apply();

	sim.run();
}
