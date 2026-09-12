// Two colliding droplets held together by a force field, using Setup API

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() { // two colliding droplets in force field; required extensions: FP16S, VOLUME_FORCE, FORCE_FIELD, SURFACE, INTERACTIVE_GRAPHICS
	const Length width = 2.56_mm;
	const Length cell = width / 256.0f; // 10 micrometres: 256 x 256 x 128 cells, as the original
	const Length big_radius = 0.32_mm, small_radius = 0.12_mm;
	const Speed small_speed = 1.0_mps;  // of the small drop: 0.2 in lattice units, as the original
	const float reynolds = 0.2f * 24.0f / 0.014f; // 343: the original lattice setup, over the small drop

	SimulationSetup sim(Domain::box(width, width, 0.5f * width).cell_size(cell));
	sim.setup();
	sim.configure_units(small_speed, Fluid::WATER, 0.2f);

	LBM lbm = sim.create_lbm_surface(small_speed * (2.0f * small_radius) / reynolds, 0_mps2,
	                                 sim.unit_scale().si_surface_tension(0.0001f)); // the original lattice setup's

	const Position center { 0.5f * width, 0.5f * width, 0.25f * width };
	SurfaceBuilder(lbm)
		.add_water(Shape::sphere(center + Position{ 0_m, -0.1_mm, 0_m }, big_radius), { Speed{}, 0.125f * small_speed, Speed{} })
		.add_water(Shape::sphere(center + Position{ 0.3_mm, 0.4_mm, 0_m }, small_radius), { Speed{}, -small_speed, Speed{} })
		.set_solid_walls()
		.apply();

	// a pull toward the center, growing with the distance: the original's force field, 0.001 per domain width (256 cells)
	const Frequency spring = sqrtf(0.001f / 256.0f) / sim.unit_scale().time_step();
	BoundaryBuilder(lbm)
		.set_force_field([=](Position p) {
			const Position r = p - center;
			return AccelerationVector{ -spring * spring * r.x, -spring * spring * r.y, -spring * spring * r.z };
		})
		.apply();

	GraphicsConfig(lbm)
		.show_free_surface()
		.apply();

	lbm.run();
} /**/
