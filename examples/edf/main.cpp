/// @file main.cpp
/// @brief Electric ducted fan (EDF): a 90 mm rotor turns inside its static stator with a 30 m/s inflow, recorded as
/// a video that pans around the fan.
///
/// Extensions: FP16S, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SUBGRID
/// STL from: https://www.thingiverse.com/thing:3014759/files
///
/// The rotor is a moving part (sim.parts()) inside the static stator.

#include "setup/setup.hpp"

/// @brief Sets up the stator (98 % of the domain's width) in an 8000 MiB grid, turns the rotor with a 100 m/s tip
/// speed, and records a 30 s video of 0.5 s of simulated time.
///
/// The rotor is turned every 0.8 us (4 time steps), as the original.
void main_setup() {
	using namespace si::symbols;
	const si::Length fan_diameter = 0.09 * m;        // 90 mm EDF
	const si::Speed tip_speed = 100.0 * m / s;       // blade tip speed
	const si::Speed inlet_velocity = 30.0 * m / s;   // 30 % of the tip speed
	const si::Duration simulation_time = 0.5 * s;
	const si::Duration update_interval = 0.8 * us;   // 4 time steps, as the original

	// the stator's diameter (X and Z) is 98 % of the domain width; the domain is 1.5 widths long (Y)
	const si::Length domain_width = fan_diameter / 0.98;
	const si::Length domain_length = 1.5 * domain_width;
	const si::Angle no_turn = 0.0 * deg;
	const si::Angle stator_turn_z = 180.0 * deg;
	const si::Length stator_offset_y = -0.2 * fan_diameter; // stator position
	const si::Length no_offset = 0.0 * m;
	const si::MemoryAmount vram = 8000.0 * MiB;
	Model stator("edf_v39.stl");
	stator.rotation(no_turn, no_turn, stator_turn_z).length(fan_diameter, Axis::X);
	Domain domain = Domain::around(stator);
	domain.size(domain_width, domain_length, domain_width);
	domain.model_offset(no_offset, stator_offset_y, no_offset);
	domain.vram(vram);
	Simulation sim(domain, Fluid::AIR, tip_speed);

	sim.boundaries()
		.set_open_boundaries()
		.initialize_velocity_y(inlet_velocity)
		.apply();

	// the rotor's STL is not in the stator's coordinates (the duct's axis is off its origin): centered on the duct's
	// axis, 0.21 diameters behind the stator's center, as in the original
	const si::Position rotor_offset{ 0.0 * m, -0.21 * fan_diameter, 0.0 * m };
	MovingPart rotor("edf_v391.stl");
	rotor.set_rotation_axis(Axis::Y).set_tip_speed(tip_speed).centered_on_model(rotor_offset).set_update_interval(update_interval);
	sim.parts()
		.add(rotor)
		.initialize();

	sim.graphics()
		.show_surface()
		.show_vortices()
		.apply();

	const si::Length view_height = 1.5 * domain_width / 1.284025;
	const si::Duration video_length = 30.0 * s;
	const VideoRecorder::CameraPath panning = [view_height](float progress) { // pans around the fan during the video
		const si::Angle azimuth = -70.0 * deg + static_cast<double>(progress) * (100.0 * deg);
		const si::Angle elevation = 2.0 * deg;
		const si::Angle field = 60.0 * deg;
		const CameraView view = CameraView::orbit(azimuth, elevation).field_of_view(field).view_height(view_height);
		return view;
	};
	sim.video()
		.add(panning)
		.set_length(video_length);
	sim.run_for(simulation_time);
}
