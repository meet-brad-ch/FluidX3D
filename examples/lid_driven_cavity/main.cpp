// Lid-driven cavity, using Setup API

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() { // lid-driven cavity; required extensions: MOVING_BOUNDARIES, INTERACTIVE_GRAPHICS
	const Length cavity_size = 1.0_m;
	const Length cell = cavity_size / 128.0f; // 128³ cells, as the original
	const Speed lid_speed = 1.0_mps;
	const float reynolds = 1000.0f; // over the cavity inside its walls

	SimulationSetup sim(Domain::box(cavity_size, cavity_size, cavity_size).cell_size(cell));
	sim.setup();
	sim.configure_units(lid_speed, Fluid::WATER, 0.1f);

	LBM lbm = sim.create_lbm(lid_speed * (cavity_size - 2.0f * cell) / reynolds);

	BoundaryBuilder(lbm)
		.preset_lid_driven_cavity(Face::Z_MAX, lid_speed)
		.apply();

	GraphicsConfig(lbm)
		.show_flags()
		.show_streamlines()
		.apply();

	lbm.run();
} /**/
