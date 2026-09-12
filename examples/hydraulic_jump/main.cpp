#include "defines.hpp"
#include "info.hpp"
#include "lbm.hpp"
#include "graphics.hpp"
#include "setup.hpp"
#include "shapes.hpp"

void main_setup() { // hydraulic jump; required extensions: FP16S, VOLUME_FORCE, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SURFACE, SUBGRID, INTERACTIVE_GRAPHICS
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

	// simulation setup
	SimulationSetup sim(Domain::box(domain_x, domain_y, domain_z).vram(208_mb));

	sim.setup();
	sim.configure_units(inlet_velocity, Fluid::WATER, LatticeMach(0.13f)); // the original's lattice speed 0.075

	// create LBM for free surface simulation
	LBM lbm = sim.create_lbm_surface(Fluid::WATER, 9.81_mps2);

	SurfaceBuilder(lbm)
		.set_water_level(water_height)
		.initialize_hydrostatic()
		.set_solid_faces({Face::X_MIN, Face::X_MAX, Face::Y_MIN, Face::Z_MIN}) // sides and bottom, the top is open
		.add_solid(Shape::box({0_m, 0_m, 0_m}, {domain_x, socket_length, socket_height})) // socket at the inlet
		.add_inflow(Face::Y_MIN, inlet_velocity, socket_height, water_height)
		.add_outflow(Face::Y_MAX, outlet_velocity)
		.apply();

	GraphicsConfig(lbm)
		.show_free_surface()
		.apply();

	lbm.run();
} /**/
