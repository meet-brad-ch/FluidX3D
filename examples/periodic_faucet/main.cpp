// Periodic faucet: a mass conservation test with water falling through the domain again and again

#include "setup/setup.hpp"

void main_setup() { // extensions: FP16S, VOLUME_FORCE, SURFACE, INTERACTIVE_GRAPHICS
	const Length height = 12.8_cm;
	const Length cell = height / 128.0f; // 96 x 192 x 128 cells, as the original
	const Length width = 96.0f * cell, length = 192.0f * cell;
	const Acceleration gravity = 9.81_mps2;
	// the original lattice setup: gravity 0.00025, viscosity 0.02; the speed of a fall through the height sets the scale
	const float lbm_fall_speed = sqrtf(2.0f * 0.00025f * 128.0f);
	const Speed fall_speed = sqrt(2.0f * gravity * height); // 1.6 m/s
	const float reynolds = lbm_fall_speed * 128.0f / 0.02f; // 1600 over the height

	Simulation sim(Domain::box(width, length, height).cell_size(cell),
	               Fluid::WATER.with_viscosity(fall_speed * height / reynolds), fall_speed,
	               LatticeMach(sqrtf(3.0f) * lbm_fall_speed)); // the original's lattice speed
	sim.set_gravity(gravity);

	// water in the far sixth runs through a bent pipe hanging from the ceiling to the hole in the floor; periodic along Z,
	// it falls in again from the hole in the ceiling
	const Length radius = height / 6.0f; // of the holes and of the pipe
	const Shape floor_and_ceiling = Shape::box({ 0_m, 0_m, 0_m }, { width, length, cell }) |
	                                Shape::box({ 0_m, 0_m, height - cell }, { width, length, height });
	const Shape hole = Shape::cylinder({ 0.5f * width, 0.5f * width, 0.5f * height }, Axis::Z, radius, height);
	const Shape pipe = Shape::torus({ 0.5f * width, 0.5f * width + radius, height }, Axis::X, radius, radius) &
	                   Shape::box({ 0_m, 0_m, 0_m }, { width, 0.5f * width + 2.0f * radius, height });
	sim.surface()
		.add_water(Shape::box({ 0_m, 5.0f / 6.0f * length, 0_m }, { width, length, height }))
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
