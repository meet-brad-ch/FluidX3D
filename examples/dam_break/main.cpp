// Dam break: a column of water collapsing in a box

#include "setup/setup.hpp"

void main_setup() { // extensions: FP16S, VOLUME_FORCE, SURFACE, INTERACTIVE_GRAPHICS
	// physical parameters (SI units)
	const Length domain_x = 0.5_m;
	const Length domain_y = 1.0_m;
	const Length domain_z = 1.0_m;
	const Length water_height = 0.75f*domain_z; // water column: 6/8 of the height and 1/8 of the length
	const Speed front_speed = sqrt(2.0f*9.81_mps2*water_height); // dam-break front speed, the fastest velocity in the flow

	// Reynolds number of the original lattice setup (front speed, water height 192 cells, viscosity 0.005);
	// water's own viscosity (Reynolds number about 3E6) would need a much finer grid
	const float reynolds = sqrt(2.0f*0.0002f*192.0f)*192.0f/0.005f; // about 10600
	const KinematicViscosity kinematic_viscosity = front_speed*water_height/reynolds;

	Simulation sim(Domain::box(domain_x, domain_y, domain_z).cell_size(domain_y / 256.0f), // 128 x 256 x 256 cells, as the original
	               Fluid::WATER.with_viscosity(kinematic_viscosity), front_speed,
	               LatticeMach(0.48f)); // the front speed as in the original lattice setup (0.28 cells per time step)
	sim.set_gravity(9.81_mps2).set_surface_tension(0.0747098_Npm); // the original lattice setup's 0.0001 at this scale

	// water column at the y = 0 wall; uniform density, as the original
	sim.surface()
		.add_water(Shape::box({0_m, 0_m, 0_m}, {domain_x, domain_y/8.0f, water_height}))
		.set_solid_box()
		.apply();

	sim.graphics()
		.show_free_surface()
		.apply();

	sim.run();
}
