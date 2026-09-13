// Rayleigh-Benard convection: air heated from below

#include "setup/setup.hpp"

void main_setup() { // extensions: FP16S, VOLUME_FORCE, TEMPERATURE, INTERACTIVE_GRAPHICS
	// physical parameters (SI units)
	const Length domain_size = 0.2_m;  // side of the square domain
	const Length height = 0.05_m;      // height (also for the Rayleigh number)

	const Temperature T_hot = 330.0_K;  // hot bottom (57°C)
	const Temperature T_cold = 300.0_K; // cold top (27°C)
	const Temperature delta_T = T_hot - T_cold;

	// buoyancy velocity scale for natural convection: u ~ sqrt(g * beta * dT * L)
	const Speed u_buoyancy = sqrt(9.81_mps2 * Fluid::AIR.thermal_expansion * delta_T * height);

	Simulation sim(Domain::box(domain_size, domain_size, height).cell_size(domain_size / 256.0f), // 256 x 256 x 64 cells, as the original
	               Fluid::AIR, u_buoyancy);
	sim.set_temperatures(T_cold, T_hot).set_gravity(9.81_mps2); // 0.5 and 1.5 in lattice units

	// thermal boundary conditions with random perturbation to trigger instability
	sim.thermal()
		.set_hot_wall(Face::Z_MIN, T_hot)
		.set_cold_wall(Face::Z_MAX, T_cold)
		.initialize_hydrostatic()
		.initialize_random_perturbation(0.15f*u_buoyancy) // 0.015 in lattice units, as the original
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
