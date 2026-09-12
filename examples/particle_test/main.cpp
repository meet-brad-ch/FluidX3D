#include "defines.hpp"
#include "info.hpp"
#include "lbm.hpp"
#include "graphics.hpp"
#include "setup.hpp"
#include "shapes.hpp"

void main_setup() { // particle test; required extensions: VOLUME_FORCE, FORCE_FIELD, MOVING_BOUNDARIES, PARTICLES, INTERACTIVE_GRAPHICS
	// physical parameters (SI units)
	const Length domain_size = 1.0_m;   // cube domain
	const Speed lid_velocity = 1.0_mps;
	const float Re = 1000.0f;           // Reynolds number
	const uint particle_count = 32768u;
	const float particle_density = 2.0f;
	const Acceleration settling_acceleration = 0.128_mps2; // makes the heavy particles settle: 1E-5 in LBM units, as the original

	// simulation setup
	SimulationSetup sim(Domain::box(domain_size, domain_size, domain_size).cell_size(domain_size / 128.0f)); // 128³ cells, as the original

	sim.setup();
	sim.configure_units(lid_velocity, Fluid::WATER);

	// create LBM with particles (Reynolds-based viscosity)
	LBM lbm = sim.create_lbm_particles_reynolds(Re, particle_count, particle_density, settling_acceleration);

	// particle seeding (SI units): a cube a quarter of the domain wide at its center, as the original
	const float center = domain_size.si() / 2.0f;
	const float half_side = domain_size.si() / 8.0f;

	ParticleManager particles(lbm);
	particles.seed()
		.cube(float3(center, center, center), half_side, particle_count);
	particles.set_visualization(true);
	particles.initialize();

	// boundary conditions (SI units)
	BoundaryBuilder(lbm)
		.preset_lid_driven_cavity(Face::Z_MAX, lid_velocity)
		.apply();

	// graphics (with the particles' mode)
	GraphicsConfig(lbm)
		.inherit_modes()
		.show_flags()
		.show_streamlines()
		.apply();

	lbm.run();
} /**/
