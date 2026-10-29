/// @file main.cpp
/// @brief Periodic faucet: a mass conservation test with water falling through the domain again and again (free
/// surface, interactive graphics).

#include "setup/setup.hpp"

#include <cmath>

/// @brief Sets up a 12.8 cm high box of 96 x 192 x 128 cells, periodic along Z, with water in its far sixth that runs
/// through a bent pipe to a hole in the floor and falls in again from the hole in the ceiling, and runs until the
/// window is closed.
///
/// The speed of a fall through the height sets the scale; the Reynolds number and the lattice speed are the
/// original example's lattice setup (gravity 0.00025, a lattice viscosity of 0.02).
void main_setup() { // extensions: FP16S, VOLUME_FORCE, SURFACE, INTERACTIVE_GRAPHICS
	using namespace si::symbols;
	const si::Length height = 12.8 * cm;
	const si::Length cell = height / 128.0; // 96 x 192 x 128 cells, as the original
	const si::Length width = 96.0 * cell;
	const si::Length length = 192.0 * cell;
	const si::Acceleration gravity = 9.81 * m / (s * s);
	const double lbm_fall_speed = std::sqrt(2.0 * 0.00025 * 128.0);
	const si::Speed fall_speed = au::sqrt(2.0 * gravity * height); // 1.6 m/s
	const double reynolds = lbm_fall_speed * 128.0 / 0.02;         // 1600 over the height

	Domain domain = Domain::box(width, length, height);
	domain.cell_size(cell);
	const FluidProperties fluid = Fluid::WATER.with_viscosity(fall_speed * height / reynolds);
	const si::LatticeMach mach(std::sqrt(3.0) * lbm_fall_speed); // the original's lattice speed
	Simulation sim(domain, fluid, fall_speed, mach);
	sim.set_gravity(gravity);

	// water in the far sixth runs through a bent pipe hanging from the ceiling to the hole in the floor; periodic along Z,
	// it falls in again from the hole in the ceiling
	const si::Length radius = height / 6.0; // of the holes and of the pipe
	const si::Length half_width = 0.5 * width;
	const Shape floor_plate = Shape::box({ 0.0 * m, 0.0 * m, 0.0 * m }, { width, length, cell });
	const Shape ceiling_plate = Shape::box({ 0.0 * m, 0.0 * m, height - cell }, { width, length, height });
	const Shape floor_and_ceiling = floor_plate | ceiling_plate;
	const Shape hole = Shape::cylinder({ half_width, half_width, 0.5 * height }, Axis::Z, radius, height);
	const Shape ring = Shape::torus({ half_width, half_width + radius, height }, Axis::X, radius, radius);
	const Shape pipe_half = Shape::box({ 0.0 * m, 0.0 * m, 0.0 * m }, { width, half_width + 2.0 * radius, height });
	const Shape pipe = ring & pipe_half;
	const Shape water = Shape::box({ 0.0 * m, 5.0 / 6.0 * length, 0.0 * m }, { width, length, height });
	sim.surface()
		.add_water(water)
		.set_solid_faces({ Face::X_MIN, Face::X_MAX, Face::Y_MIN, Face::Y_MAX })
		.add_solid(floor_and_ceiling & !hole)
		.add_solid(pipe)
		.apply();

	sim.graphics()
		.show_flags()
		.show_free_surface_mesh()
		.apply();

	sim.run();
}
