/// @file main.cpp
/// @brief Space Shuttle aerodynamics on 8 GPUs: the airflow around the orbiter at 100 m/s and -20 degrees angle of
/// attack, the grid split among 2 x 4 x 1 GPUs, recorded from two cameras.
///
/// Extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID
/// STL from: https://www.thingiverse.com/thing:4975964/files

#include "setup/setup.hpp"

/// @brief Sets up the orbiter (37.24 m long, 30 % of the domain) in a 1000 MiB grid and records a 30 s video of
/// 10 s of simulated time.
///
/// The lattice Mach number gives the original's lattice speed 0.075.
void main_setup() {
	using namespace si::symbols;
	const si::Length orbiter_length = 37.24 * m; // NASA: 37.24 m long, 23.79 m wingspan
	const si::Speed flight_speed = 100.0 * m / s;
	const si::Angle yaw = 270.0 * deg;           // turns the model's length along Y
	const si::Angle angle_of_attack = -20.0 * deg;
	const si::Duration simulation_time = 10.0 * s;
	const si::Duration video_length = 30.0 * s;

	// the orbiter (Y, before its angle of attack) is 30 % of the domain length; the grid is split among 2 x 4 x 1 GPUs
	const double orbiter_in_length = 0.3;
	const double width_in_lengths = 0.25;
	const double height_in_lengths = 0.2;
	const si::Length domain_length = orbiter_length / orbiter_in_length;
	const si::Length domain_width = width_in_lengths * domain_length;
	const si::Length domain_height = height_in_lengths * domain_length;
	const double offset_y_in_lengths = -1.1;  // the model's offset along Y from the domain's middle, in orbiter lengths
	const double offset_z_in_lengths = 0.05;  // the model's offset along Z from the domain's middle, in orbiter lengths
	const si::Length model_offset_y = offset_y_in_lengths * orbiter_length;
	const si::Length model_offset_z = offset_z_in_lengths * orbiter_length;
	const si::MemoryAmount vram = 1000.0 * MiB;
	Model model("Full_Shuttle.stl");
	model.rotation(0.0 * deg, 0.0 * deg, yaw).angle_of_attack(angle_of_attack).length(orbiter_length);
	Domain domain = Domain::around(model);
	domain.size(domain_width, domain_length, domain_height);
	domain.model_offset(0.0 * m, model_offset_y, model_offset_z);
	domain.vram(vram);
	domain.gpus(2u, 4u, 1u);
	const si::LatticeMach mach(0.13); // the original's lattice speed 0.075
	Simulation sim(domain, Fluid::AIR, flight_speed, mach);

	sim.boundaries()
		.set_open_boundaries()
		.initialize_velocity_y(flight_speed)
		.apply();

	sim.graphics()
		.show_flags()
		.show_surface()
		.show_vortices()
		.apply();

	const si::Length D = domain_length; // camera positions from the domain's origin corner
	const si::Position top{ -0.23399 * D, 0.864331 * D, 0.368885 * D };
	const si::Position bottom{ -0.130302 * D, -0.018006 * D, 0.1 * D };
	const si::Angle top_azimuth = -205.0 * deg;
	const si::Angle top_elevation = 36.0 * deg;
	const si::Angle bottom_azimuth = -137.0 * deg;
	const si::Angle bottom_elevation = 0.0 * deg;
	const si::Angle field_of_view = 74.0 * deg; // both cameras'
	const CameraView top_view = CameraView::at(top, top_azimuth, top_elevation).field_of_view(field_of_view);
	const CameraView bottom_view = CameraView::at(bottom, bottom_azimuth, bottom_elevation).field_of_view(field_of_view);
	sim.video()
		.add("top", top_view)
		.add("bottom", bottom_view)
		.set_length(video_length);
	sim.run_for(simulation_time);
}
