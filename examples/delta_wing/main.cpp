/// @file main.cpp
/// @brief Delta wing: the airflow over a flat triangular wing at Reynolds number 100000, with its leading-edge
/// vortices (interactive graphics).

#include "setup/setup.hpp"

/// @brief Sets up the wing in a 1 x 4 x 1 m box of 128 x 512 x 128 cells and runs until the window is closed.
///
/// The width is the reference length, as in the original; the lattice Mach number gives the original's lattice speed
/// 0.075.
void main_setup() { // extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS
	using namespace si::symbols;
	const si::Length width = 1.0 * m;      // the domain's, the reference length as in the original
	const double reynolds = 100000.0;
	const si::Speed flow_speed = reynolds * Fluid::AIR.kinematic_viscosity / width; // 1.48 m/s in air
	const double lengths_per_width = 4.0;  // 128 x 512 x 128 cells
	const double cells_across = 128.0;
	const si::Length length = lengths_per_width * width;
	const si::Length cell = width / cells_across;

	Domain domain = Domain::box(width, length, width);
	domain.cell_size(cell);
	const si::LatticeMach mach(0.13); // the original's lattice speed 0.075
	Simulation sim(domain, Fluid::AIR, flow_speed, mach);

	// a flat triangular wing 1.33 widths long and 0.63 wide, centered in X, its tip high at the front; the corners in
	// 64ths of the width, as the original
	const double units_per_width = 64.0;
	const si::Length w = width / units_per_width;
	const si::Length center = 0.5 * width;
	const double tip_y = 5.0;         // the tip's distance from the inlet, in 64ths of the width
	const double tip_z = 20.0;        // the tip's height above the middle, in 64ths of the width
	const double corner_y = 90.0;     // the rear corners' distance from the inlet, in 64ths of the width
	const double corner_x = 20.0;     // the rear corners' distance from the middle across, in 64ths of the width
	const double corner_z = 10.0;     // the rear corners' depth below the middle, in 64ths of the width
	const si::Position tip{ center, tip_y * w, center + tip_z * w };
	const si::Position left_corner{ center - corner_x * w, corner_y * w, center - corner_z * w };
	const si::Position right_corner{ center + corner_x * w, corner_y * w, center - corner_z * w };
	const Shape wing = Shape::triangle(tip, left_corner, right_corner);
	sim.boundaries()
		.add_solid(wing)
		.set_open_boundaries()
		.initialize_velocity_y(flow_speed)
		.apply();

	sim.graphics()
		.show_surface()
		.show_vortices()
		.apply();

	sim.run();
}
