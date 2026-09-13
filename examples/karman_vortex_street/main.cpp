// 2D Karman vortex street behind a cylinder

#include "setup/setup.hpp"

void main_setup() { // extensions: D2Q9, FP16S, EQUILIBRIUM_BOUNDARIES, INTERACTIVE_GRAPHICS
	const Length diameter = 1.0_cm;       // the cylinder's
	const Length cell = diameter / 32.0f; // 256 x 512 cells, as the original
	const float reynolds = 250.0f;
	const Speed flow_speed = reynolds * Fluid::WATER.kinematic_viscosity / diameter; // 2.5 cm/s in water

	Simulation sim(Domain::box(8.0f * diameter, 16.0f * diameter, cell).cell_size(cell), // one cell high: 2D
	               Fluid::WATER, flow_speed);

	sim.boundaries()
		.add_solid(Shape::cylinder({ 4.0f * diameter, 4.0f * diameter, 0.5f * cell }, Axis::Z, 0.5f * diameter, cell))
		.set_open_boundaries()
		.set_periodic(Axis::Z)
		.initialize_velocity_y(flow_speed)
		.apply();

	sim.graphics()
		.show_flags()
		.show_velocity_field()
		.set_slice_mode(SliceMode::Z)
		.apply();

	sim.run();
}
