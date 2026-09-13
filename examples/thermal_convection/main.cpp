// Thermal convection: air between a hot and a cold wall

#include "setup/setup.hpp"

void main_setup() { // extensions: FP16S, VOLUME_FORCE, TEMPERATURE, INTERACTIVE_GRAPHICS
	// physical parameters (SI units)
	const Length domain_x = 0.1_m;     // wide
	const Length domain_y = 0.6_m;     // long (flow direction)
	const Length domain_z = 0.2_m;     // tall

	const Temperature T_hot = 350.0_K;  // hot wall (77°C)
	const Temperature T_cold = 300.0_K; // cold wall (27°C)
	const Temperature delta_T = T_hot - T_cold;

	// buoyancy velocity scale for natural convection: u ~ sqrt(g * beta * dT * L)
	const Speed u_buoyancy = sqrt(9.81_mps2 * Fluid::AIR.thermal_expansion * delta_T * domain_z);

	// finer than the original's 32 x 196 x 60 cells: real air there would have a lattice viscosity of 0.00085 (the
	// original's lattice fluid: 0.02) and blows up
	Simulation sim(Domain::box(domain_x, domain_y, domain_z).vram(2000_mb), Fluid::AIR, u_buoyancy);
	sim.set_temperatures(T_cold, T_hot).set_gravity(9.81_mps2); // 0.5 and 1.5 in lattice units

	// thermal boundary conditions (SI temperatures in Kelvin)
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
