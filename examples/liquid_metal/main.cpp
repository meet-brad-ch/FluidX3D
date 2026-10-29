/// @file main.cpp
/// @brief Liquid metal on a vibrating speaker membrane: a 3.2 cm deep layer of galinstan on a floor that oscillates
/// up and down at 44 Hz (free surface, moving boundaries, interactive graphics).

#include "setup/setup.hpp"

#include <cmath>

/// @brief Sets up a 12.8 cm container of 128 x 128 x 96 cells with the liquid at rest and the membrane moving with a
/// peak speed of 0.40 m/s, and runs until the window is closed.
///
/// The Froude and Reynolds numbers, the frequency and the surface tension are the original example's lattice setup:
/// the membrane's peak speed 0.09 and frequency 0.01 per time step, gravity 0.0005, a depth of 32 cells, a lattice
/// viscosity of 0.01, a lattice surface tension of 0.005.
void main_setup() { // extensions: FP16S, VOLUME_FORCE, MOVING_BOUNDARIES, SURFACE, INTERACTIVE_GRAPHICS
	using namespace si::symbols;
	const si::Length width = 12.8 * cm;          // the container; 128 x 128 x 96 cells, as the original
	const si::Length cell = width / 128.0;
	const si::Length height = 0.75 * width;
	const si::Length depth = height / 3.0;       // of the liquid: 3.2 cm
	const si::Density density = 6440.0 * kg / (m * m * m); // galinstan, a metal that is liquid at room temperature
	const si::Acceleration gravity = 9.81 * m / (s * s);
	const double froude = 0.09 / std::sqrt(0.0005 * 32.0);
	const si::Speed peak_speed = froude * au::sqrt(gravity * depth);              // 0.40 m/s
	const si::Frequency frequency = (0.01 * 32.0 / 0.09) * peak_speed / depth;    // 44 Hz
	const double reynolds = 0.09 * 32.0 / 0.01;                                   // 288 over the depth
	const FluidProperties galinstan{ .density = density, .kinematic_viscosity = peak_speed * depth / reynolds };

	Domain domain = Domain::box(width, width, height);
	domain.cell_size(cell);
	const si::LatticeMach mach(0.156); // the original's lattice speed 0.09
	Simulation sim(domain, galinstan, peak_speed, mach);
	const si::SurfaceTension surface_tension = 0.630829 * N / m; // the original lattice setup's 0.005 at this scale
	sim.set_gravity(gravity).set_surface_tension(surface_tension);

	sim.surface()
		.set_water_level(depth)
		.initialize_hydrostatic()
		.set_solid_box()
		.apply();

	// the speaker's membrane, the floor inside the walls, oscillates up and down
	const si::Position membrane_from{ cell, cell, 0.0 * m };
	const si::Position membrane_to{ width - cell, width - cell, cell };
	const auto membrane_velocity = [=](si::Position, si::Duration t) {
		const double cycles = au::as_raw_number(frequency * t);
		const si::Speed w = peak_speed * std::sin(2.0 * pi * cycles);
		const si::Velocity velocity{ si::Speed{}, si::Speed{}, w };
		return velocity;
	};
	sim.boundaries()
		.add_moving_solid(Shape::box(membrane_from, membrane_to), membrane_velocity)
		.apply();

	sim.graphics()
		.show_free_surface()
		.apply();

	sim.run();
}
