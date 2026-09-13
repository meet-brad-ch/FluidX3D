// Delta wing

#include "setup/setup.hpp"

void main_setup() { // extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS
	const Length width = 1.0_m;        // the domain's, the reference length as in the original
	const float reynolds = 100000.0f;
	const Speed flow_speed = reynolds * Fluid::AIR.kinematic_viscosity / width; // 1.48 m/s in air

	Simulation sim(Domain::box(width, 4.0f * width, width).cell_size(width / 128.0f), // 128 x 512 x 128 cells
	               Fluid::AIR, flow_speed, LatticeMach(0.13f)); // the original's lattice speed 0.075

	// a flat triangular wing 1.33 widths long and 0.63 wide, centered in X, its tip high at the front
	const Length w = width / 64.0f;
	const Length center = 0.5f * width;
	sim.boundaries()
		.add_solid(Shape::triangle({ center, 5.0f * w, center + 20.0f * w },
		                           { center - 20.0f * w, 90.0f * w, center - 10.0f * w },
		                           { center + 20.0f * w, 90.0f * w, center - 10.0f * w }))
		.set_open_boundaries()
		.initialize_velocity_y(flow_speed)
		.apply();

	sim.graphics()
		.show_surface()
		.show_vortices()
		.apply();

	sim.run();
}
