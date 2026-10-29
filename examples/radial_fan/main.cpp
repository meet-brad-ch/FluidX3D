/// @file main.cpp
/// @brief Radial fan: a 30 cm fan turns in an open-top enclosure; the domain's model is the fan, which turns as a
/// moving part. Recorded as a video.
///
/// Extensions: FP16S, MOVING_BOUNDARIES, SUBGRID
/// STL from: https://www.thingiverse.com/thing:6113/files

#include "setup/setup.hpp"

/// @brief Sets up the fan (half of the enclosure's width, near the floor) in a 181 MiB grid with a blade tip speed
/// of 30 m/s, and records a 30 s video of 1 s of simulated time.
///
/// The part is turned every 90 us (10 time steps), as the original.
void main_setup() {
	using namespace si::symbols;
	const si::Length fan_diameter = 0.3 * m;        // 30 cm fan
	const si::Speed tip_speed = 30.0 * m / s;       // blade tip speed
	const si::Duration simulation_time = 1.0 * s;   // 1 second of rotation
	const si::Duration update_interval = 90.0 * us; // 10 time steps, as the original

	// the fan fills half of the enclosure's width, near the floor
	const si::Length enclosure_width = 2.0 * fan_diameter;
	const si::Length enclosure_height = 2.0 / 3.0 * fan_diameter;
	const si::Length no_offset = 0.0 * m;
	const si::Length fan_offset_z = -0.154 * fan_diameter; // near the floor
	const si::MemoryAmount vram = 181.0 * MiB;
	Model model("FAN_Solid_Bottom.stl");
	model.length(fan_diameter);
	Domain domain = Domain::around(model);
	domain.size(enclosure_width, enclosure_width, enclosure_height);
	domain.model_offset(no_offset, no_offset, fan_offset_z);
	domain.vram(vram);
	Simulation sim(domain, Fluid::AIR, tip_speed);
	sim.skip_model_voxelization(); // the fan turns: a moving part, not a static solid

	// solid floor and walls, open top
	sim.boundaries()
		.set_solid_floor()
		.set_solid_sides()
		.apply();

	MovingPart fan(sim.model_file());
	fan.set_rotation_axis(Axis::Z).set_tip_speed(tip_speed).set_update_interval(update_interval);
	sim.parts()
		.add(fan)
		.initialize();

	sim.graphics()
		.show_flags()
		.show_surface()
		.show_vortices()
		.apply();

	const si::Length d = fan_diameter; // camera position from the domain's origin corner
	const si::Position camera_position{ 1.70702 * d, 0.699348 * d, 1.42929 * d };
	const si::Angle camera_azimuth = -25.0 * deg;
	const si::Angle camera_elevation = 61.0 * deg;
	const si::Duration video_length = 30.0 * s;
	sim.video()
		.add(CameraView::at(camera_position, camera_azimuth, camera_elevation))
		.set_length(video_length);
	sim.run_for(simulation_time);
}
