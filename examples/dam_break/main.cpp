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
	SimulationSetup sim(Domain::box(domain_x, domain_y, domain_z).cell_size(domain_y / 256.0f)); // 128 x 256 x 256 cells, as the original

	sim.setup();
	const Speed front_speed = sqrt(2.0f*9.81_mps2*water_height); // dam-break front speed, the fastest velocity in the flow
	sim.configure_units(front_speed, Fluid::WATER, LatticeMach(0.48f)); // the front speed as in the original lattice setup (0.28 cells per time step)

	// create LBM for free surface simulation
	// Reynolds number of the original lattice setup (front speed, water height 192 cells, viscosity 0.005);
	// water's own viscosity (Reynolds number about 3E6) would need a much finer grid
	const float reynolds = sqrt(2.0f*0.0002f*192.0f)*192.0f/0.005f; // about 10600
	const KinematicViscosity kinematic_viscosity = front_speed*water_height/reynolds;
	const SurfaceTension surface_tension = sim.unit_scale().si_surface_tension(0.0001f); // the original lattice setup's
	LBM lbm = sim.create_lbm_surface(kinematic_viscosity, 9.81_mps2, surface_tension);

	// water column at the y = 0 wall
	SurfaceBuilder(lbm)
		.add_water(Shape::box({0_m, 0_m, 0_m}, {domain_x, domain_y/8.0f, water_height}))
		.set_solid_walls()
		.apply(); // uniform density, as the original

	GraphicsConfig(lbm)
		.show_free_surface()
		.apply();

	lbm.run();
} /**/
