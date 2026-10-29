/// @file main.cpp
/// @brief Boeing 747 aerodynamics: the airflow around a full-scale 747 at 250 m/s and -15 degrees angle of attack,
/// recorded as a video of the surface and the vortices.
///
/// Extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID
/// STL from: https://www.thingiverse.com/thing:2772812/files

#include "setup/setup.hpp"

/// @brief Sets up the aircraft (its fuselage 70.7 m long) in an 880 MiB grid and records 10 s of simulated time.
///
/// The lattice Mach number gives the original's lattice speed 0.075.
void main_setup() {
	using namespace si::symbols;
	const si::Length fuselage_length = 70.7 * m;
	const si::Speed cruise_speed = 250.0 * m / s;
	const si::Angle angle_of_attack = -15.0 * deg;
	const si::Duration simulation_time = 10.0 * s;
	const si::Duration video_length = 10.0 * s;

	// the domain is as wide as the fuselage (Y) is long; the wingspan (X) is 64 m
	const si::Length domain_width = fuselage_length;
	const si::Length domain_length = 2.0 * fuselage_length;
	const si::Length domain_height = 0.5 * fuselage_length;
	const double offset_in_lengths = -0.45; // the model's offset along Y from the domain's middle, in fuselage lengths
	const si::Length model_offset_y = offset_in_lengths * fuselage_length;
	const si::MemoryAmount vram = 880.0 * MiB;
	Model model("techtris_airplane.stl");
	model.angle_of_attack(angle_of_attack).length(fuselage_length).repair_mesh();
	Domain domain = Domain::around(model);
	domain.size(domain_width, domain_length, domain_height);
	domain.model_offset(0.0 * m, model_offset_y, 0.0 * m);
	domain.vram(vram);
	const si::LatticeMach mach(0.13); // the original's lattice speed 0.075
	Simulation sim(domain, Fluid::AIR, cruise_speed, mach);

	sim.boundaries()
		.set_open_boundaries()
		.initialize_velocity_y(cruise_speed)
		.apply();

	sim.graphics()
		.show_surface()
		.show_vortices()
		.apply();

	const si::Length L = fuselage_length; // camera position from the domain's origin corner
	const si::Position camera_position{ 1.5 * L, 0.2 * L, 1.25 * L };
	const si::Angle camera_azimuth = -33.0 * deg;
	const si::Angle camera_elevation = 42.0 * deg;
	const si::Angle camera_field_of_view = 68.0 * deg;
	const CameraView camera_view = CameraView::at(camera_position, camera_azimuth, camera_elevation).field_of_view(camera_field_of_view);
	sim.video()
		.add(camera_view)
		.set_length(video_length);
	sim.run_for(simulation_time);
}
