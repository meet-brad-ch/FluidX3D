/// @file main.cpp
/// @brief Particles in a lid-driven cavity: heavy particles, seeded in a cube at the centre, settle and are carried
/// by the flow under a moving lid (particles with 2-way coupling, interactive graphics).

#include "setup/setup.hpp"

/// @brief Sets up a 1 m cavity of 128³ cells at Reynolds number 1000 with 32768 particles twice as dense as the
/// fluid, and runs until the window is closed.
///
/// The settling acceleration is 1e-5 in lattice units, as the original.
void main_setup() { // extensions: VOLUME_FORCE, FORCE_FIELD, MOVING_BOUNDARIES, PARTICLES, INTERACTIVE_GRAPHICS
	using namespace si::symbols;
	const si::Length domain_size = 1.0 * m;     // cube domain
	const si::Speed lid_velocity = 1.0 * m / s;
	const double reynolds = 1000.0;             // over the domain
	const uint particle_count = 32768u;
	const float particle_density = 2.0f;        // relative to the fluid's (a ratio, as the core takes it)
	const si::Acceleration settling_acceleration = 0.128 * m / (s * s); // makes the heavy particles settle: 1E-5 in LBM units, as the original
	const si::KinematicViscosity viscosity = lid_velocity * domain_size / reynolds;
	const double cells_per_side = 128.0; // 128³ cells, as the original
	const si::Length cell = domain_size / cells_per_side;

	Domain domain = Domain::box(domain_size, domain_size, domain_size);
	domain.cell_size(cell);
	const FluidProperties fluid = Fluid::WATER.with_viscosity(viscosity);
	Simulation sim(domain, fluid, lid_velocity);
	sim.set_particles(particle_count, particle_density);
	sim.set_gravity(settling_acceleration);

	// particle seeding: a cube a quarter of the domain wide at its center, as the original
	const si::Position center{ 0.5 * domain_size, 0.5 * domain_size, 0.5 * domain_size };
	const si::Length half_side = domain_size / 8.0;
	sim.particles().seed().cube(center, half_side, particle_count);
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
