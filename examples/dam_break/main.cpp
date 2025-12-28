#include "defines.hpp"
#include "info.hpp"
#include "lbm.hpp"
#include "graphics.hpp"
#include "setup.hpp"
#include "shapes.hpp"

void main_setup() { // dam break; required extensions: FP16S, VOLUME_FORCE, SURFACE, INTERACTIVE_GRAPHICS
	// physical parameters (SI units)
	const float domain_x_m = 0.5f;
	const float domain_y_m = 1.0f;
	const float domain_z_m = 1.0f;

	// simulation setup
	SimulationSetup sim(SimulationConfig()
		.set_domain_size_m(domain_x_m, domain_y_m, domain_z_m)
		.set_vram_mb(2000u));

	sim.setup();
	sim.configure_units(1.0f, Fluid::WATER);

	// create LBM for free surface simulation
	LBM lbm = sim.create_lbm_surface(Fluid::WATER, 9.81f);

	// free surface configuration: water column at y=0, 3/4 height
	const uint Ny = lbm.get_Ny();
	const uint Nz = lbm.get_Nz();

	SurfaceBuilder(lbm)
		.set_fluid_region([=](uint, uint y, uint z) {
			return z < Nz * 6u / 8u && y < Ny / 8u;
		})
		.set_gravity_lbm(sim.to_lbm_force_per_volume(9.81f))
		.set_solid_walls()
		.initialize_hydrostatic()
		.apply();

	SurfaceBuilder(lbm).configure_visualization();

	lbm.run();
} /**/
