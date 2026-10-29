/// @file main.cpp
/// @brief Two colliding droplets of water held together by a force field that pulls toward the centre (free surface,
/// interactive graphics).

#include "setup/setup.hpp"

#include <cmath>

/// @brief Sets up a 2.56 x 2.56 x 1.28 mm box of 256 x 256 x 128 cells with a big and a small drop moving toward each
/// other, and runs until the window is closed.
///
/// The Reynolds number, the lattice speed, the surface tension and the force field are the original example's
/// lattice setup (the small drop at 0.2 cells per time step, a lattice viscosity of 0.014, a lattice surface tension
/// of 0.0001, a pull of 0.001 per domain width).
void main_setup() { // extensions: FP16S, VOLUME_FORCE, FORCE_FIELD, SURFACE, INTERACTIVE_GRAPHICS
	using namespace si::symbols;
	const si::Length width = 2.56 * mm;
	const si::Length cell = width / 256.0;       // 10 micrometres: 256 x 256 x 128 cells, as the original
	const si::Length big_radius = 0.32 * mm;
	const si::Length small_radius = 0.12 * mm;
	const si::Speed small_speed = 1.0 * m / s;   // of the small drop: 0.2 in lattice units, as the original
	const double reynolds = 0.2 * 24.0 / 0.014;  // 343: the original lattice setup, over the small drop

	Domain domain = Domain::box(width, width, 0.5 * width);
	domain.cell_size(cell);
	const FluidProperties fluid = Fluid::WATER.with_viscosity(small_speed * (2.0 * small_radius) / reynolds);
	const si::LatticeMach mach(0.346); // the original's lattice speed 0.2
	Simulation sim(domain, fluid, small_speed, mach);
	const si::SurfaceTension surface_tension = 2.50142E-5 * N / m; // the original lattice setup's 0.0001 at this scale
	sim.set_surface_tension(surface_tension);

	const si::Position center{ 0.5 * width, 0.5 * width, 0.25 * width };
	const si::Position big_center = center + si::Position{ 0.0 * m, -0.1 * mm, 0.0 * m };
	const si::Position small_center = center + si::Position{ 0.3 * mm, 0.4 * mm, 0.0 * m };
	const si::Velocity big_velocity{ si::Speed{}, 0.125 * small_speed, si::Speed{} };
	const si::Velocity small_velocity{ si::Speed{}, -small_speed, si::Speed{} };
	sim.surface()
		.add_water(Shape::sphere(big_center, big_radius), big_velocity)
		.add_water(Shape::sphere(small_center, small_radius), small_velocity)
		.set_solid_box()
		.apply();

	// a pull toward the center, growing with the distance: the original's force field, 0.001 per domain width (256 cells)
	const si::Frequency spring = std::sqrt(0.001 / 256.0) / sim.converter().step();
	const auto spring_squared = spring * spring;
	const auto pull = [=](si::Position p) {
		const si::Position r = p - center;
		const si::AccelerationVector force{ -spring_squared * r.x, -spring_squared * r.y, -spring_squared * r.z };
		return force;
	};
	sim.boundaries()
		.set_force_field(pull)
		.apply();

	sim.graphics()
		.show_free_surface()
		.apply();

	sim.run();
}
