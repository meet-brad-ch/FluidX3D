/// @file main.cpp
/// @brief 2D Kármán vortex street: water flows past a cylinder at Reynolds number 250 and sheds vortices (D2Q9,
/// interactive graphics).

#include "setup/setup.hpp"

/// @brief Sets up a 1 cm cylinder in an 8 x 16 cm channel of 256 x 512 cells with an inflow at 2.5 cm/s and runs
/// until the window is closed.
void main_setup() { // extensions: D2Q9, FP16S, EQUILIBRIUM_BOUNDARIES, INTERACTIVE_GRAPHICS
	using namespace si::symbols;
	const si::Length diameter = 1.0 * cm;      // the cylinder's
	const si::Length cell = diameter / 32.0;   // 256 x 512 cells, as the original
	const double reynolds = 250.0;
	const si::Speed flow_speed = reynolds * Fluid::WATER.kinematic_viscosity / diameter; // 2.5 cm/s in water

	const double width_in_diameters = 8.0;   // the channel's width across the flow, in cylinder diameters
	const double length_in_diameters = 16.0; // the channel's length along the flow, in cylinder diameters
	const si::Length width = width_in_diameters * diameter;
	const si::Length length = length_in_diameters * diameter;

	Domain domain = Domain::box(width, length, cell); // one cell high: 2D
	domain.cell_size(cell);
	Simulation sim(domain, Fluid::WATER, flow_speed);

	const si::Position cylinder_center{ 4.0 * diameter, 4.0 * diameter, 0.5 * cell };
	const Shape cylinder = Shape::cylinder(cylinder_center, Axis::Z, 0.5 * diameter, cell);
	sim.boundaries()
		.add_solid(cylinder)
		.set_open_boundaries()
		.set_periodic(Axis::Z)
		.initialize_velocity_y(flow_speed)
		.apply();

	sim.graphics()
		.show_flags()
		.show_velocity_field()
		.set_slice_mode(SliceMode::Z)
		.apply();

	sim.run();
}
