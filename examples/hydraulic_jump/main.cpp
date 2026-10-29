/// @file main.cpp
/// @brief Hydraulic jump: water flows over a socket at the inlet into a slower, deeper stream (free surface,
/// interactive graphics).

#include "setup/setup.hpp"

/// @brief Sets up a 0.96 x 3.52 x 0.96 m channel in a 208 MiB grid with an inflow of 0.25 m³/s above the socket and
/// an outflow at the far end, and runs until the window is closed.
///
/// The lattice Mach number gives the original's lattice speed 0.075.
void main_setup() { // extensions: FP16S, VOLUME_FORCE, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SURFACE, SUBGRID, INTERACTIVE_GRAPHICS
	using namespace si::symbols;
	const si::Length domain_x = 0.96 * m;
	const si::Length domain_y = 3.52 * m;
	const si::Length domain_z = 0.96 * m;

	const si::Length socket_length = domain_y * 3.0 / 20.0;
	const si::Length socket_height = domain_z * 2.0 / 5.0;
	const si::Length water_height = domain_z * 3.0 / 5.0;

	const si::VolumeFlowRate flow_rate = 0.25 * m * m * m / s;
	const si::Speed inlet_velocity = flow_rate / (domain_x * (water_height - socket_height)); // above the socket
	const si::Speed outlet_velocity = flow_rate / (domain_x * socket_height);

	const si::MemoryAmount vram = 208.0 * MiB;
	const si::Acceleration gravity = 9.81 * m / (s * s);
	Domain domain = Domain::box(domain_x, domain_y, domain_z);
	domain.vram(vram);
	const si::LatticeMach mach(0.13); // the original's lattice speed 0.075
	Simulation sim(domain, Fluid::WATER, inlet_velocity, mach);
	sim.set_gravity(gravity);

	const si::Position socket_from{ 0.0 * m, 0.0 * m, 0.0 * m };
	const si::Position socket_to{ domain_x, socket_length, socket_height };
	sim.surface()
		.set_water_level(water_height)
		.initialize_hydrostatic()
		.set_solid_faces({ Face::X_MIN, Face::X_MAX, Face::Y_MIN, Face::Z_MIN }) // sides and bottom, the top is open
		.add_solid(Shape::box(socket_from, socket_to)) // socket at the inlet
		.add_inflow(Face::Y_MIN, inlet_velocity, socket_height, water_height)
		.add_outflow(Face::Y_MAX, outlet_velocity)
		.apply();

	sim.graphics()
		.show_free_surface()
		.apply();

	sim.run();
}
