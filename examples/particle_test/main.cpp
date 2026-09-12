#include "defines.hpp"
#include "info.hpp"
#include "lbm.hpp"
#include "graphics.hpp"
#include "setup.hpp"
#include "shapes.hpp"

void main_setup() { // particle test; required extensions: VOLUME_FORCE, FORCE_FIELD, MOVING_BOUNDARIES, PARTICLES, INTERACTIVE_GRAPHICS
	// physical parameters (SI units)
	const float domain_size_m = 1.0f;   // 1 meter cube domain
	const float velocity_mps = 1.0f;    // 1 m/s lid velocity
	const float Re = 1000.0f;           // Reynolds number
	const uint particle_count = 32768u;
	const float particle_density = 2.0f;
	const float settling_acceleration_mps2 = 0.27f; // makes the heavy particles settle; the original lattice setup used 1E-5 (LBM units)

	// simulation setup
	SimulationSetup sim(SimulationConfig()
		.set_domain_size_m(domain_size_m, domain_size_m, domain_size_m)
		.set_vram_mb(2000u));

	sim.setup();
	sim.configure_units(velocity_mps, Fluid::WATER);

	// create LBM with particles (Reynolds-based viscosity)
	LBM lbm = sim.create_lbm_particles_reynolds(Re, particle_count, particle_density, settling_acceleration_mps2);

	// particle seeding (SI units)
	const float center = domain_size_m / 2.0f;
	const float radius = domain_size_m / 8.0f;

	ParticleManager particles(lbm);
	particles.seed()
		.sphere(float3(center, center, center), radius, particle_count);
	particles.set_visualization(true);
	particles.initialize();

	// boundary conditions (SI units)
	BoundaryBuilder(lbm)
		.preset_lid_driven_cavity(Face::Z_MAX, velocity_mps)
		.apply();

	// graphics
	GraphicsConfig(lbm)
		.show_surface()
		.show_streamlines()
		.apply();

	lbm.run();
} /**/
