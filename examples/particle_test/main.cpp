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
	const Acceleration settling_acceleration = 0.27_mps2; // makes the heavy particles settle; the original lattice setup used 1E-5 (LBM units)

	// simulation setup
	SimulationSetup sim(Domain::box(domain_size, domain_size, domain_size).vram(2000_mb));

	sim.setup();
	sim.configure_units(lid_velocity, Fluid::WATER);

	// create LBM with particles (Reynolds-based viscosity)
	LBM lbm = sim.create_lbm_particles_reynolds(Re, particle_count, particle_density, settling_acceleration);

	// particle seeding (SI units)
	const float center = domain_size.si() / 2.0f;
	const float radius = domain_size.si() / 8.0f;

	ParticleManager particles(lbm);
	particles.seed()
		.sphere(float3(center, center, center), radius, particle_count);
	particles.set_visualization(true);
	particles.initialize();

	// boundary conditions (SI units)
	BoundaryBuilder(lbm)
		.preset_lid_driven_cavity(Face::Z_MAX, lid_velocity)
		.apply();

	// graphics
	GraphicsConfig(lbm)
		.show_surface()
		.show_streamlines()
		.apply();

	lbm.run();
} /**/
