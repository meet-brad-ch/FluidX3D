// Particles in a lid-driven cavity

#include "setup/setup.hpp"

void main_setup() { // extensions: VOLUME_FORCE, FORCE_FIELD, MOVING_BOUNDARIES, PARTICLES, INTERACTIVE_GRAPHICS
	// physical parameters (SI units)
	const Length domain_size = 1.0_m;   // cube domain
	const Speed lid_velocity = 1.0_mps;
	const float Re = 1000.0f;           // Reynolds number over the domain
	const uint particle_count = 32768u;
	const float particle_density = 2.0f; // relative to the fluid's
	const Acceleration settling_acceleration = 0.128_mps2; // makes the heavy particles settle: 1E-5 in LBM units, as the original

	Simulation sim(Domain::box(domain_size, domain_size, domain_size).cell_size(domain_size / 128.0f), // 128³ cells, as the original
	               Fluid::WATER.with_viscosity(lid_velocity * domain_size / Re), lid_velocity);
	sim.set_particles(particle_count, particle_density).set_gravity(settling_acceleration);

	// particle seeding: a cube a quarter of the domain wide at its center, as the original
	sim.particles().seed()
		.cube({ 0.5f * domain_size, 0.5f * domain_size, 0.5f * domain_size }, domain_size / 8.0f, particle_count);
	sim.particles().show().initialize();

	sim.boundaries()
		.preset_lid_driven_cavity(Face::Z_MAX, lid_velocity)
		.apply();

	// graphics (with the particles' mode)
	sim.graphics()
		.inherit_modes()
		.show_flags()
		.show_streamlines()
		.apply();

	sim.run();
}
