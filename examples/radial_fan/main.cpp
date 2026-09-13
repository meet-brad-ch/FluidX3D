// Radial Fan
//
// Extensions: FP16S, MOVING_BOUNDARIES, SUBGRID
// STL from: https://www.thingiverse.com/thing:6113/files
//
// A rotating fan in an open-top enclosure: the domain's model is the fan itself, which turns as a moving part.

#include "setup/setup.hpp"

void main_setup() {
	const Length fan_diameter = 0.3_m;      // 30 cm fan
	const Speed tip_speed = 30.0_mps;       // blade tip speed
	const Duration simulation_time = 1.0_s; // 1 second of rotation
	const Duration update_interval = 90_us; // 10 time steps, as the original

	// the fan fills half of the enclosure's width, near the floor
	Simulation sim(Domain::around(Model("FAN_Solid_Bottom.stl").length(fan_diameter))
		.size(2.0f * fan_diameter, 2.0f * fan_diameter, 2.0f / 3.0f * fan_diameter)
		.model_offset(0_m, 0_m, -0.154f * fan_diameter)
		.vram(181_mb),
		Fluid::AIR, tip_speed);
	sim.skip_model_voxelization(); // the fan turns: a moving part, not a static solid

	// solid floor and walls, open top
	sim.boundaries()
		.set_solid_floor()
		.set_solid_sides()
		.apply();

	sim.parts()
		.add(MovingPart(sim.model_file())
			.set_rotation_axis(Axis::Z)
			.set_tip_speed(tip_speed)
			.set_update_interval(update_interval))
		.initialize();

	sim.graphics()
		.show_flags()
		.show_surface()
		.show_vortices()
		.apply();

	const Length d = fan_diameter; // camera position from the domain's origin corner
	sim.video()
		.add(CameraView::at({ 1.70702f * d, 0.699348f * d, 1.42929f * d }, -25_deg, 61_deg))
		.set_length(30.0_s);
	sim.run_for(simulation_time);
}
