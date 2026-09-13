// Two colliding droplets held together by a force field

#include "setup/setup.hpp"

void main_setup() { // extensions: FP16S, VOLUME_FORCE, FORCE_FIELD, SURFACE, INTERACTIVE_GRAPHICS
	const Length width = 2.56_mm;
	const Length cell = width / 256.0f; // 10 micrometres: 256 x 256 x 128 cells, as the original
	const Length big_radius = 0.32_mm, small_radius = 0.12_mm;
	const Speed small_speed = 1.0_mps;  // of the small drop: 0.2 in lattice units, as the original
	const float reynolds = 0.2f * 24.0f / 0.014f; // 343: the original lattice setup, over the small drop

	Simulation sim(Domain::box(width, width, 0.5f * width).cell_size(cell),
	               Fluid::WATER.with_viscosity(small_speed * (2.0f * small_radius) / reynolds), small_speed,
	               LatticeMach(0.346f)); // the original's lattice speed 0.2
	sim.set_surface_tension(2.50142E-5_Npm); // the original lattice setup's 0.0001 at this scale

	const Position center { 0.5f * width, 0.5f * width, 0.25f * width };
	sim.surface()
		.add_water(Shape::sphere(center + Position{ 0_m, -0.1_mm, 0_m }, big_radius), { Speed{}, 0.125f * small_speed, Speed{} })
		.add_water(Shape::sphere(center + Position{ 0.3_mm, 0.4_mm, 0_m }, small_radius), { Speed{}, -small_speed, Speed{} })
		.set_solid_box()
		.apply();

	// a pull toward the center, growing with the distance: the original's force field, 0.001 per domain width (256 cells)
	const Frequency spring = sqrtf(0.001f / 256.0f) / sim.unit_scale().time_step();
	sim.boundaries()
		.set_force_field([=](Position p) {
			const Position r = p - center;
			return AccelerationVector{ -spring * spring * r.x, -spring * spring * r.y, -spring * spring * r.z };
		})
		.apply();

	sim.graphics()
		.show_free_surface()
		.apply();

	sim.run();
}
