// Lid-driven cavity

#include "setup/setup.hpp"

void main_setup() { // extensions: MOVING_BOUNDARIES, INTERACTIVE_GRAPHICS
	const Length cavity_size = 1.0_m;
	const Length cell = cavity_size / 128.0f; // 128³ cells, as the original
	const Speed lid_speed = 1.0_mps;
	const float reynolds = 1000.0f; // over the cavity inside its walls

	Simulation sim(Domain::box(cavity_size, cavity_size, cavity_size).cell_size(cell),
	               Fluid::WATER.with_viscosity(lid_speed * (cavity_size - 2.0f * cell) / reynolds), lid_speed);

	sim.boundaries()
		.preset_lid_driven_cavity(Face::Z_MAX, lid_speed)
		.apply();

	sim.graphics()
		.show_flags()
		.show_streamlines()
		.apply();

	sim.run();
}
