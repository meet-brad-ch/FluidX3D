/// @file main.cpp
/// @brief Star Wars X-wing aerodynamics: the airflow around a T-65 X-wing at 100 m/s, recorded from four cameras.
///
/// Extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID
/// STL from: https://www.thingiverse.com/thing:353276/files

#include "setup/setup.hpp"

/// @brief Sets up the X-wing (its fuselage 13.4 m long) in an 880 MiB grid and records a 30 s video of 10 s of
/// simulated time.
///
/// The lattice Mach number gives the original's lattice speed 0.075.
void main_setup() {
	using namespace si::symbols;
	const si::Length fuselage_length = 13.4 * m; // T-65 X-wing; the model's longest side
	const si::Speed flight_speed = 100.0 * m / s;
	const si::Angle yaw = 180.0 * deg;           // turns the model to fly along +Y
	const si::Duration simulation_time = 10.0 * s;
	const si::Duration video_length = 30.0 * s;

	// the fuselage (Y) is as long as the domain is wide, half its length
	const si::Length domain_width = fuselage_length;
	const si::Length domain_length = 2.0 * fuselage_length;
	const si::Length domain_height = 0.5 * fuselage_length;
	const double offset_in_lengths = -0.45; // the model's offset along Y from the domain's middle, in fuselage lengths
	const si::Length model_offset_y = offset_in_lengths * fuselage_length;
	const si::MemoryAmount vram = 880.0 * MiB;
	Model model("X-Wing.stl");
	model.rotation(0.0 * deg, 0.0 * deg, yaw).length(fuselage_length);
	Domain domain = Domain::around(model);
	domain.size(domain_width, domain_length, domain_height);
	domain.model_offset(0.0 * m, model_offset_y, 0.0 * m);
	domain.vram(vram);
	const si::LatticeMach mach(0.13); // the original's lattice speed 0.075
	Simulation sim(domain, Fluid::AIR, flight_speed, mach);

	sim.boundaries()
		.set_open_boundaries()
		.initialize_velocity_y(flight_speed)
		.apply();

	sim.graphics()
		.show_surface()
		.show_vortices()
		.apply();

	const si::Length L = fuselage_length; // camera positions from the domain's origin corner
	const si::Position top{ 1.5 * L, 0.2 * L, 1.25 * L };
	const si::Position bottom{ 1.0 * L, 0.3 * L, -0.1 * L };
	const si::Position front{ 0.5 * L, 2.02 * L, 0.625 * L };
	const si::Position side{ 1.2 * L, 0.7 * L, 0.28 * L };
	const si::Angle top_azimuth = -33.0 * deg;
	const si::Angle top_elevation = 42.0 * deg;
	const si::Angle top_field_of_view = 68.0 * deg;
	const si::Angle bottom_azimuth = -33.0 * deg;
	const si::Angle bottom_elevation = -40.0 * deg;
	const si::Angle front_azimuth = 90.0 * deg;
	const si::Angle front_elevation = 28.0 * deg;
	const si::Angle front_field_of_view = 80.0 * deg;
	const CameraView top_view = CameraView::at(top, top_azimuth, top_elevation).field_of_view(top_field_of_view);
	const CameraView bottom_view = CameraView::at(bottom, bottom_azimuth, bottom_elevation);
	const CameraView front_view = CameraView::at(front, front_azimuth, front_elevation).field_of_view(front_field_of_view);
	const CameraView side_view = CameraView::at(side);
	sim.video()
		.add("t", top_view)
		.add("b", bottom_view)
		.add("f", front_view)
		.add("s", side_view)
		.set_length(video_length);
	sim.run_for(simulation_time);
}
