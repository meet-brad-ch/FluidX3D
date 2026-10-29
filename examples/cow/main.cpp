/// @file main.cpp
/// @brief Aerodynamics of a cow: the wind at 1 m/s around a 2.4 m cow standing on the floor, recorded as a video.

#include "setup/setup.hpp"

/// @brief Sets up the cow (65 % of the domain's length, its nose 0.24 m from the inlet) in a 1000 MiB grid and
/// records 10 s of simulated time.
///
/// The lattice Mach number gives the original's lattice speed 0.075.
void main_setup() { // extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS or GRAPHICS
	using namespace si::symbols;
	const si::Speed flow_velocity = 1.0 * m / s;
	const si::Length cow_length = 2.4 * m;
	const si::Duration simulation_time = 10.0 * s;
	const si::Duration video_length = 10.0 * s;
	const double cow_in_length = 0.65; // the cow is 65 % of the domain length
	const si::Length domain_length = cow_length / cow_in_length;
	const si::Length domain_width = 0.5 * domain_length;
	const si::Length domain_height = 0.5 * domain_length;
	const double nose_gap_in_lengths = 0.1; // the gap from the inlet to the cow's nose, in cow lengths
	const si::Length nose_gap = nose_gap_in_lengths * cow_length;
	const si::Angle roll = 180.0 * deg; // the model's turn about X, then about Z
	const si::Angle yaw = 180.0 * deg;
	const si::MemoryAmount vram = 1000.0 * MiB;

	Model model("Cow_t.stl");
	model.rotation(roll, 0.0 * deg, yaw).length(cow_length);
	Domain domain = Domain::around(model);
	domain.size(domain_width, domain_length, domain_height);
	domain.gap_to_inlet(nose_gap);
	domain.on_floor();
	domain.vram(vram);
	const si::LatticeMach mach(0.13); // the original's lattice speed 0.075
	Simulation sim(domain, Fluid::AIR, flow_velocity, mach);

	sim.boundaries()
		.set_solid_floor()
		.set_open_boundaries()
		.initialize_velocity_y(flow_velocity)
		.apply();

	sim.graphics()
		.show_surface()
		.show_vortices()
		.apply();

	const double view_height_divisor = 1.25; // the camera's view height is the domain length over this
	const si::Length view_height = domain_length / view_height_divisor;
	const si::Angle camera_azimuth = -40.0 * deg;
	const si::Angle camera_elevation = 20.0 * deg;
	const si::Angle camera_field_of_view = 78.0 * deg;
	const CameraView camera_view = CameraView::orbit(camera_azimuth, camera_elevation).field_of_view(camera_field_of_view).view_height(view_height);
	sim.video()
		.add(camera_view)
		.set_length(video_length);
	sim.run_for(simulation_time);
}
