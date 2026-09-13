// Hydraulic jump: water flowing over a socket into a slower, deeper stream

#include "setup/setup.hpp"

void main_setup() { // extensions: FP16S, VOLUME_FORCE, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SURFACE, SUBGRID, INTERACTIVE_GRAPHICS
	// physical parameters (SI units)
	const Length domain_x = 0.96_m;
	const Length domain_y = 3.52_m;
	const Length domain_z = 0.96_m;

	const Length socket_length = domain_y*3.0f/20.0f;
	const Length socket_height = domain_z*2.0f/5.0f;
	const Length water_height = domain_z*3.0f/5.0f;

	const VolumeFlowRate flow_rate = 0.25_m3ps;
	const Speed inlet_velocity = flow_rate/(domain_x*(water_height - socket_height)); // above the socket
	const Speed outlet_velocity = flow_rate/(domain_x*socket_height);

	Simulation sim(Domain::box(domain_x, domain_y, domain_z).vram(208_mb),
	               Fluid::WATER, inlet_velocity, LatticeMach(0.13f)); // the original's lattice speed 0.075
	sim.set_gravity(9.81_mps2);

	sim.surface()
		.set_water_level(water_height)
		.initialize_hydrostatic()
		.set_solid_faces({Face::X_MIN, Face::X_MAX, Face::Y_MIN, Face::Z_MIN}) // sides and bottom, the top is open
		.add_solid(Shape::box({0_m, 0_m, 0_m}, {domain_x, socket_length, socket_height})) // socket at the inlet
		.add_inflow(Face::Y_MIN, inlet_velocity, socket_height, water_height)
		.add_outflow(Face::Y_MAX, outlet_velocity)
		.apply();

	sim.graphics()
		.show_free_surface()
		.apply();

	sim.run();
}
