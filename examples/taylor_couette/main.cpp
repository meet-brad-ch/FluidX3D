/// @file main.cpp
/// @brief Taylor-Couette flow between a turning inner and a resting outer cylinder: the Taylor vortices form in the
/// gap (moving boundaries, interactive graphics).

#include "setup/setup.hpp"

#include <cmath>

/// @brief Sets up a 10 cm wide, 20 cm high pair of cylinders in 96 x 96 x 192 cells, the inner one turning with a
/// surface speed of 6 mm/s, and runs until the window is closed.
///
/// The Reynolds number and the lattice speed are the original example's lattice setup (surface speed 0.25 cells per
/// time step, a gap of 23 cells, a lattice viscosity of 0.04).
void main_setup() { // extensions: MOVING_BOUNDARIES, INTERACTIVE_GRAPHICS
	using namespace si::symbols;
	const si::Length width = 10.0 * cm;      // the outer cylinder with its wall
	const si::Length cell = width / 96.0;    // 96 x 96 x 192 cells, as the original
	const si::Length height = 2.0 * width;
	const si::Length outer_radius = 0.5 * width - cell;
	const si::Length inner_radius = 0.25 * width;
	const si::Length gap = outer_radius - inner_radius;
	const double reynolds = 0.25 * 23.0 / 0.04; // 144: the original lattice setup (surface speed 0.25, gap 23 cells, viscosity 0.04)
	const si::Speed surface_speed = reynolds * Fluid::WATER.kinematic_viscosity / gap; // of the inner cylinder: 6 mm/s in water

	Domain domain = Domain::box(width, width, height);
	domain.cell_size(cell);
	const si::LatticeMach mach(0.433); // the original's lattice speed 0.25
	Simulation sim(domain, Fluid::WATER, surface_speed, mach);

	const si::Position center{ 0.5 * width, 0.5 * width, 0.5 * height };
	const si::Frequency turn_rate = surface_speed / inner_radius; // radians per second
	const auto wall_velocity = [=](si::Position p) {
		// clockwise about Z; a small axial wave, 2 gaps long, starts the Taylor vortices (the original: random noise)
		const double gaps = au::as_raw_number(p.z / gap);
		const si::Speed wave = 0.004 * surface_speed * std::sin(pi * gaps);
		const si::Speed u = turn_rate * (p.y - center.y);
		const si::Speed v = -turn_rate * (p.x - center.x);
		const si::Velocity velocity{ u, v, wave };
		return velocity;
	};
	const Shape outer_wall = !Shape::cylinder(center, Axis::Z, outer_radius, height);
	const Shape inner_cylinder = Shape::cylinder(center, Axis::Z, inner_radius, height);
	sim.boundaries()
		.add_solid(outer_wall)
		.add_moving_solid(inner_cylinder, wall_velocity)
		.apply();

	sim.graphics()
		.show_flags()
		.show_streamlines()
		.apply();

	sim.run();
}
