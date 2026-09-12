#include "defines.hpp"
#include "info.hpp"
#include "lbm.hpp"
#include "graphics.hpp"
#include "setup.hpp"
#include "shapes.hpp"

void main_setup() { // dam break; required extensions: FP16S, VOLUME_FORCE, SURFACE, INTERACTIVE_GRAPHICS
	// physical parameters (SI units)
	const Length domain_x = 0.5_m;
	const Length domain_y = 1.0_m;
	const Length domain_z = 1.0_m;

	// simulation setup
	SimulationSetup sim(Domain::box(domain_x, domain_y, domain_z).vram(2000_mb));

	sim.setup();
	const float water_height_m = 0.75f*domain_z.si(); // water column: 6/8 of the height
	const float front_speed_mps = sqrt(2.0f*9.81f*water_height_m); // dam-break front speed, the fastest velocity in the flow
	sim.configure_units(front_speed_mps, Fluid::WATER, 0.28f); // front speed in LBM units as in the original lattice setup: sqrt(2*0.0002*192)

	// create LBM for free surface simulation
	// Reynolds number of the original lattice setup (front speed, water height 192 cells, viscosity 0.005);
	// water's own viscosity (Reynolds number about 3E6) would need a much finer grid
	const float reynolds = sqrt(2.0f*0.0002f*192.0f)*192.0f/0.005f; // about 10600
	const float kinematic_viscosity_m2ps = front_speed_mps*water_height_m/reynolds;
	LBM lbm = sim.create_lbm_surface(kinematic_viscosity_m2ps, 9.81f);

	// free surface configuration: water column at y=0, 3/4 height
	const uint Ny = lbm.get_Ny();
	const uint Nz = lbm.get_Nz();

	SurfaceBuilder(lbm)
		.set_fluid_region([=](uint, uint y, uint z) {
			return z < Nz * 6u / 8u && y < Ny / 8u;
		})
		.set_gravity_lbm(sim.to_lbm_acceleration(9.81f))
		.set_solid_walls()
		.initialize_hydrostatic()
		.apply();

	SurfaceBuilder(lbm).configure_visualization();

	lbm.run();
} /**/
