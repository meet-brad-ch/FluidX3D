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
	const Length water_height = 0.75f*domain_z; // water column: 6/8 of the height and 1/8 of the length

	// simulation setup
	SimulationSetup sim(Domain::box(domain_x, domain_y, domain_z).vram(2000_mb));

	sim.setup();
	const float front_speed_mps = sqrt(2.0f*9.81f*water_height.si()); // dam-break front speed, the fastest velocity in the flow
	sim.configure_units(front_speed_mps, Fluid::WATER, 0.28f); // front speed in LBM units as in the original lattice setup: sqrt(2*0.0002*192)

	// create LBM for free surface simulation
	// Reynolds number of the original lattice setup (front speed, water height 192 cells, viscosity 0.005);
	// water's own viscosity (Reynolds number about 3E6) would need a much finer grid
	const float reynolds = sqrt(2.0f*0.0002f*192.0f)*192.0f/0.005f; // about 10600
	const float kinematic_viscosity_m2ps = front_speed_mps*water_height.si()/reynolds;
	LBM lbm = sim.create_lbm_surface(kinematic_viscosity_m2ps, 9.81f);

	// water column at the y = 0 wall
	SurfaceBuilder(lbm)
		.add_water_box({0_m, 0_m, 0_m}, {domain_x, domain_y/8.0f, water_height})
		.set_solid_walls()
		.initialize_hydrostatic()
		.apply();

	SurfaceBuilder(lbm).configure_visualization();

	lbm.run();
} /**/
