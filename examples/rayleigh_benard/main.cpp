/// @file main.cpp
/// @brief Rayleigh-Bénard convection: a layer of air heated from below and cooled from above forms convection cells
/// (temperature, interactive graphics).

#include "setup/setup.hpp"

/// @brief Sets up a 20 x 20 x 5 cm layer of 256 x 256 x 64 cells between a floor at 330 K and a ceiling at 300 K,
/// periodic at the sides, with a random start, and runs until the window is closed.
///
/// The buoyancy speed sqrt(g beta dT h) sets the scale; the perturbation is 0.015 in lattice units, as the original.
void main_setup() { // extensions: FP16S, VOLUME_FORCE, TEMPERATURE, INTERACTIVE_GRAPHICS
	using namespace si::symbols;
	const si::Length domain_size = 0.2 * m;  // side of the square domain
	const si::Length height = 0.05 * m;      // height (also for the Rayleigh number)
	const si::Acceleration gravity = 9.81 * m / (s * s);

	const si::Temperature T_hot = si::kelvin_point(330.0);  // hot bottom (57°C)
	const si::Temperature T_cold = si::kelvin_point(300.0); // cold top (27°C)
	const si::TemperatureDifference delta_T = T_hot - T_cold;

	// buoyancy velocity scale for natural convection: u ~ sqrt(g * beta * dT * L)
	const si::Speed u_buoyancy = au::sqrt(gravity * Fluid::AIR.thermal_expansion * delta_T * height);

	const double cells_wide = 256.0; // 256 x 256 x 64 cells, as the original
	const si::Length cell_size = domain_size / cells_wide;
	Domain domain = Domain::box(domain_size, domain_size, height);
	domain.cell_size(cell_size);
	Simulation sim(domain, Fluid::AIR, u_buoyancy);
	sim.set_temperatures(T_cold, T_hot).set_gravity(gravity); // 0.5 and 1.5 in lattice units

	// thermal boundary conditions with random perturbation to trigger instability
	const si::Speed perturbation = 0.15 * u_buoyancy; // 0.015 in lattice units, as the original
	sim.thermal()
		.set_hot_wall(Face::Z_MIN, T_hot)
		.set_cold_wall(Face::Z_MAX, T_cold)
		.initialize_hydrostatic()
		.initialize_random_perturbation(perturbation)
		.apply();

	// solid floor and ceiling only (lateral walls periodic)
	sim.boundaries()
		.set_solid_floor()
		.set_solid_ceiling()
		.apply();

	sim.graphics()
		.show_flags()
		.show_streamlines()
		.apply();

	sim.run();
}
