/// @file main.cpp
/// @brief A bubble bursting at a water surface: a 4 mm air bubble just below the surface of water at rest (free
/// surface, interactive graphics).

#include "setup/setup.hpp"

#include <cmath>

/// @brief Sets up a 16 x 16 x 12 mm box of 275 x 275 x 206 cells with water 2 bubble diameters deep and the bubble
/// at its surface, and runs until the window is closed.
///
/// No flow sets the units, but the bubble's capillary speed sqrt(sigma/(rho d)); on the lattice it is the original's,
/// sqrt(sigma/d) with the lattice surface tension 0.0003 and d in cells.
void main_setup() { // extensions: FP16C, VOLUME_FORCE, SURFACE, INTERACTIVE_GRAPHICS
	using namespace si::symbols;
	const si::Length d = 4.0 * mm;                // the bubble's diameter
	const double cells_per_diameter = 68.75;      // 275 x 275 x 206 cells, as the original
	const FluidProperties& water = Fluid::WATER;
	const si::SurfaceTension surface_tension = 0.072 * N / m;
	const si::Speed capillary_speed = au::sqrt(surface_tension / (water.density * d));
	const si::Acceleration gravity = 9.81 * m / (s * s);
	const si::Length box_width = 4.0 * d;
	const si::Length box_height = 3.0 * d;

	Domain domain = Domain::box(box_width, box_width, box_height);
	domain.cell_size(d / cells_per_diameter);
	const si::LatticeMach mach(std::sqrt(3.0 * 0.0003 / cells_per_diameter));
	Simulation sim(domain, water, capillary_speed, mach);
	sim.set_gravity(gravity).set_surface_tension(surface_tension);

	// water 2 diameters deep, the bubble just below its surface
	const si::Length depth = 2.0 * d;
	const si::Position bubble_center{ 2.0 * d, 2.0 * d, depth - 0.5 * d };
	sim.surface()
		.set_water_level(depth)
		.add_gas(Shape::sphere(bubble_center, 0.5 * d))
		.set_solid_faces({ Face::Z_MIN })
		.apply();

	sim.graphics()
		.show_free_surface()
		.apply();

	sim.run();
}
