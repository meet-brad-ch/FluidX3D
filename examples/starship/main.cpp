/// @file main.cpp
/// @brief SpaceX Starship aerodynamics: the air flows up past the upper stage at 100 m/s, as in a belly-first
/// reentry, recorded from three cameras.
///
/// Extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID
/// STL from: https://www.thingiverse.com/thing:4912729/files

#include "setup/setup.hpp"

/// @brief Sets up the ship (50 m long, 80 % of the domain) in a 1000 MiB grid and records a 20 s video of 10 s of
/// simulated time.
///
/// The lattice Mach number gives the original's lattice speed 0.05.
void main_setup() {
	using namespace si::symbols;
	const si::Length ship_length = 50.0 * m; // Starship upper stage; the model's longest side
	const si::Speed reentry_speed = 100.0 * m / s;
	const si::Duration simulation_time = 10.0 * s;
	const si::Duration video_length = 20.0 * s;

	// the ship (Y) is 80 % of the domain length; the air flows up (+z) past it, as in a belly-first reentry
	const double ship_in_length = 0.8;
	const si::Length domain_length = ship_length / ship_in_length;
	const si::Length domain_width = 0.5 * domain_length;
	const double offset_y_in_lengths = 0.05;   // the model's offset along Y from the domain's middle, in ship lengths
	const double offset_z_in_lengths = -0.445; // the model's offset along Z from the domain's middle, in ship lengths
	const si::Length model_offset_y = offset_y_in_lengths * ship_length;
	const si::Length model_offset_z = offset_z_in_lengths * ship_length;
	const si::MemoryAmount vram = 1000.0 * MiB;
	Model model("StarShipV2.stl");
	model.length(ship_length).repair_mesh();
	Domain domain = Domain::around(model);
	domain.size(domain_width, domain_length, domain_length);
	domain.model_offset(0.0 * m, model_offset_y, model_offset_z);
	domain.vram(vram);
	const si::LatticeMach mach(0.0866); // the original's lattice speed 0.05
	Simulation sim(domain, Fluid::AIR, reentry_speed, mach);

	sim.boundaries()
		.set_open_boundaries()
		.initialize_velocity_z(reentry_speed)
		.apply();

	sim.graphics()
		.show_flags()
		.show_surface()
		.show_vortices()
		.apply();

	const si::Length D = domain_length; // camera positions from the domain's origin corner
	const si::Position top{ 1.30837 * D, -0.275261 * D, 1.52658 * D };
	const si::Position bottom{ 0.609471 * D, 0.811263 * D, 0.001634 * D };
	const si::Position side{ 1.12406 * D, 0.942782 * D, 0.587945 * D };
	const si::Angle top_azimuth = -38.0 * deg;
	const si::Angle top_elevation = 37.0 * deg;
	const si::Angle top_field_of_view = 60.0 * deg;
	const si::Angle bottom_azimuth = 32.0 * deg;
	const si::Angle bottom_elevation = -40.0 * deg;
	const si::Angle bottom_field_of_view = 104.0 * deg;
	const si::Angle side_azimuth = 24.0 * deg;
	const si::Angle side_elevation = 2.0 * deg;
	const si::Angle side_field_of_view = 92.0 * deg;
	const CameraView top_view = CameraView::at(top, top_azimuth, top_elevation).field_of_view(top_field_of_view);
	const CameraView bottom_view = CameraView::at(bottom, bottom_azimuth, bottom_elevation).field_of_view(bottom_field_of_view);
	const CameraView side_view = CameraView::at(side, side_azimuth, side_elevation).field_of_view(side_field_of_view);
	sim.video()
		.add("top", top_view)
		.add("bottom", bottom_view)
		.add("side", side_view)
		.set_length(video_length);
	sim.run_for(simulation_time);
}
