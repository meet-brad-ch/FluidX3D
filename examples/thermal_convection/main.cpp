/// @file main.cpp
/// @brief Thermal convection: air in a closed box between a hot and a cold wall (temperature, interactive graphics).

#include "setup/setup.hpp"

/// @brief Sets up a 10 x 60 x 20 cm box in a 2000 MiB grid with the walls at 350 K and 300 K, and runs until the
/// window is closed.
///
/// The grid is finer than the original's 32 x 196 x 60 cells: real air there would have a lattice viscosity of
/// 0.00085 (the original's lattice fluid: 0.02) and blows up.
void main_setup() { // extensions: FP16S, VOLUME_FORCE, TEMPERATURE, INTERACTIVE_GRAPHICS
	using namespace si::symbols;
	const si::Length domain_x = 0.1 * m;     // wide
	const si::Length domain_y = 0.6 * m;     // long (between the walls)
	const si::Length domain_z = 0.2 * m;     // tall
	const si::Acceleration gravity = 9.81 * m / (s * s);

	const si::Temperature T_hot = si::kelvin_point(350.0);  // hot wall (77°C)
	const si::Temperature T_cold = si::kelvin_point(300.0); // cold wall (27°C)
	const si::TemperatureDifference delta_T = T_hot - T_cold;

	// buoyancy velocity scale for natural convection: u ~ sqrt(g * beta * dT * L)
	const si::Speed u_buoyancy = au::sqrt(gravity * Fluid::AIR.thermal_expansion * delta_T * domain_z);

	const si::MemoryAmount vram = 2000.0 * MiB;
	Domain domain = Domain::box(domain_x, domain_y, domain_z);
	domain.vram(vram);
	Simulation sim(domain, Fluid::AIR, u_buoyancy);
	sim.set_temperatures(T_cold, T_hot).set_gravity(gravity); // 0.5 and 1.5 in lattice units

	sim.thermal()
		.set_hot_wall(Face::Y_MIN, T_hot)
		.set_cold_wall(Face::Y_MAX, T_cold)
		.initialize_hydrostatic()
		.apply();

	// all six faces solid, as the original
	sim.boundaries()
		.set_solid_box()
		.apply();

	sim.graphics()
		.show_flags()
		.show_streamlines()
		.apply();

	sim.run();
}
