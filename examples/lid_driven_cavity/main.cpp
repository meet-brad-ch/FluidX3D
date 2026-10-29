/// @file main.cpp
/// @brief Lid-driven cavity: a cube of fluid whose top face moves at a constant speed, at Reynolds number 1000
/// (moving boundaries, interactive graphics).

#include "setup/setup.hpp"

/// @brief Sets up a 1 m cavity of 128³ cells with a lid at 1 m/s and runs until the window is closed.
///
/// The viscosity is chosen so that the Reynolds number over the cavity inside its walls is 1000.
void main_setup() { // extensions: MOVING_BOUNDARIES, INTERACTIVE_GRAPHICS
	using namespace si::symbols;
	const si::Length cavity_size = 1.0 * m;
	const si::Length cell = cavity_size / 128.0; // 128³ cells, as the original
	const si::Speed lid_speed = 1.0 * m / s;
	const double reynolds = 1000.0; // over the cavity inside its walls
	const si::Length inside = cavity_size - 2.0 * cell;
	const si::KinematicViscosity viscosity = lid_speed * inside / reynolds;

	Domain domain = Domain::box(cavity_size, cavity_size, cavity_size);
	domain.cell_size(cell);
	const FluidProperties fluid = Fluid::WATER.with_viscosity(viscosity);
	Simulation sim(domain, fluid, lid_speed);

	sim.boundaries()
		.preset_lid_driven_cavity(Face::Z_MAX, lid_speed)
		.apply();

	sim.graphics()
		.show_flags()
		.show_streamlines()
		.apply();

	sim.run();
}
