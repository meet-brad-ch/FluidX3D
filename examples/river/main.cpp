/// @file main.cpp
/// @brief A river flowing around two pillars: a periodic channel of water driven down a 14 % slope (free surface,
/// interactive graphics).

#include "setup/setup.hpp"

#include <cmath>

/// @brief Sets up a 1 x 3 x 0.75 m channel of 128 x 384 x 96 cells with 0.25 m of water flowing at 1.2 m/s, a round
/// and a square pillar, and runs until the window is closed.
///
/// The Froude and Reynolds numbers and the surface tension are the original example's lattice setup (flow speed 0.1,
/// gravity 0.0005, a depth of 32 cells, a lattice viscosity of 0.02, a lattice surface tension of 0.01); water's own
/// viscosity would need a much finer grid.
void main_setup() { // extensions: FP16S, VOLUME_FORCE, SURFACE, INTERACTIVE_GRAPHICS
	using namespace si::symbols;
	const si::Length width = 1.0 * m;
	const si::Length cell = width / 128.0; // 128 x 384 x 96 cells, as the original
	const si::Length length = 3.0 * width;
	const si::Length height = 0.75 * width;
	const si::Length depth = 32.0 * cell;  // 0.25 m of water
	const si::Acceleration gravity = 9.81 * m / (s * s);
	const double slope = 0.14;             // of the river's bed: 14 % of gravity pulls downstream (-Y), as the original
	const double froude = 0.1 / std::sqrt(0.0005 * 32.0);
	const si::Speed flow_speed = froude * au::sqrt(gravity * depth); // 1.2 m/s
	const double reynolds = 0.1 * 32.0 / 0.02; // 160 over the depth

	Domain domain = Domain::box(width, length, height);
	domain.cell_size(cell);
	const FluidProperties fluid = Fluid::WATER.with_viscosity(flow_speed * depth / reynolds);
	Simulation sim(domain, fluid, flow_speed);
	const si::AccelerationVector downstream{ si::Acceleration{}, -slope * gravity, -gravity };
	const si::SurfaceTension surface_tension = 11.9535 * N / m; // the original lattice setup's 0.01 at this scale
	sim.set_body_force(downstream)
	   .set_surface_tension(surface_tension);

	const si::Length pillar = 20.0 * cell; // radius
	const si::Velocity river_velocity{ si::Speed{}, -flow_speed, si::Speed{} };
	const si::Position round_center{ 2.0 / 3.0 * width, 2.0 / 3.0 * length, 0.5 * height };
	const si::Position square_from{ width / 3.0 - pillar, length / 3.0 - pillar, 0.0 * m };
	const si::Position square_to{ width / 3.0 + pillar, length / 3.0 + pillar, height };
	sim.surface()
		.set_water_level(depth, river_velocity)
		.set_solid_faces({ Face::X_MIN, Face::X_MAX, Face::Z_MIN }) // periodic along Y: the river flows around
		.add_solid(Shape::cylinder(round_center, Axis::Z, pillar, height))
		.add_solid(Shape::box(square_from, square_to))
		.apply();

	sim.graphics()
		.show_free_surface()
		.apply();

	sim.run();
}
