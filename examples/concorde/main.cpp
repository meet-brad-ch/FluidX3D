/// @file main.cpp
/// @brief Concorde aerodynamics: the airflow around the supersonic aircraft at 300 km/h and -10 degrees angle of
/// attack (a landing approach), recorded from six cameras.
///
/// Extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID
/// STL from: https://www.thingiverse.com/thing:1176931/files

#include "setup/setup.hpp"

/// @brief The video's cameras: their positions, as parts of the domain's length from the domain's origin
/// corner, and their directions, as the original.
class Cameras {
public:
	/// @brief Adds the cameras to the video.
	/// @param video the simulation's video recorder
	/// @param reference the domain's length: the camera positions are parts of it
	static void add_to(VideoRecorder& video, si::Length reference) {
		using namespace si::symbols;
		const si::Length D = reference; // camera positions from the domain's origin corner
		const double orbit_height_divisor = 1.648722; // the orbit cameras' view height is the domain length over this
		const si::Length orbit_height = D / orbit_height_divisor;
		const si::Position front{ 0.330448 * D, -0.382147 * D, 0.17739 * D };
		const si::Position back{ 0.544454 * D, 1.90708 * D, 0.364068 * D };
		const si::Position wing{ 0.256454 * D, 0.32028 * D, 0.134165 * D };
		const si::Position follow{ 0.2348 * D, 0.840055 * D, 0.353484 * D };
		const si::Angle front_azimuth = -78.0 * deg;
		const si::Angle front_elevation = 6.0 * deg;
		const si::Angle front_field_of_view = 22.0 * deg;
		const si::Angle back_azimuth = 72.0 * deg;
		const si::Angle back_elevation = 12.0 * deg;
		const si::Angle back_field_of_view = 20.0 * deg;
		const si::Angle side_azimuth = 0.0 * deg;
		const si::Angle side_elevation = 0.0 * deg;
		const si::Angle top_azimuth = 0.0 * deg;
		const si::Angle top_elevation = 90.0 * deg;
		const si::Angle orbit_field_of_view = 25.0 * deg; // the side and the top camera's
		const si::Angle wing_azimuth = -56.0 * deg;
		const si::Angle wing_elevation = 31.6 * deg;
		const si::Angle follow_azimuth = 80.0 * deg;
		const si::Angle follow_elevation = 35.6 * deg;
		const si::Angle follow_field_of_view = 34.0 * deg;
		const CameraView front_view = CameraView::at(front, front_azimuth, front_elevation).field_of_view(front_field_of_view);
		const CameraView back_view = CameraView::at(back, back_azimuth, back_elevation).field_of_view(back_field_of_view);
		const CameraView side_view = CameraView::orbit(side_azimuth, side_elevation).field_of_view(orbit_field_of_view).view_height(orbit_height);
		const CameraView top_view = CameraView::orbit(top_azimuth, top_elevation).field_of_view(orbit_field_of_view).view_height(orbit_height);
		const CameraView wing_view = CameraView::at(wing, wing_azimuth, wing_elevation);
		const CameraView follow_view = CameraView::at(follow, follow_azimuth, follow_elevation).field_of_view(follow_field_of_view);
		video.add("front", front_view);
		video.add("back", back_view);
		video.add("side", side_view);
		video.add("top", top_view);
		video.add("wing", wing_view);
		video.add("follow", follow_view);
	}
};

/// @brief Sets up the aircraft (its fuselage 62 m long, 56 % of the domain) in a 2084 MiB grid and records 10 s of
/// simulated time.
///
/// The lattice Mach number gives the original's lattice speed 0.075; the camera positions are the original's, as
/// parts of the domain's length.
void main_setup() {
	using namespace si::symbols;
	const si::Length fuselage_length = 62.0 * m;
	const si::Speed cruise_speed = 300.0 * kmh;
	const si::Angle angle_of_attack = -10.0 * deg;
	const si::Duration simulation_time = 10.0 * s;
	const si::Duration video_length = 10.0 * s;

	// the fuselage (Y) is 56 % of the domain length
	const double fuselage_in_length = 0.56;
	const double lengths_per_width = 3.0;  // the domain is 3 times as long as it is wide
	const double lengths_per_height = 6.0; // the domain is 6 times as long as it is high
	const si::Length domain_length = fuselage_length / fuselage_in_length;
	const si::Length domain_width = domain_length / lengths_per_width;
	const si::Length domain_height = domain_length / lengths_per_height;
	const double offset_y_in_lengths = -0.373; // the model's offset along Y from the domain's middle, in fuselage lengths
	const double offset_z_in_lengths = 0.03;   // the model's offset along Z from the domain's middle, in fuselage lengths
	const si::Length model_offset_y = offset_y_in_lengths * fuselage_length;
	const si::Length model_offset_z = offset_z_in_lengths * fuselage_length;
	const si::Angle roll = 90.0 * deg; // the model's turn about X, then about Z, to its length along Y
	const si::Angle yaw = 90.0 * deg;
	const si::MemoryAmount vram = 2084.0 * MiB;
	Model model("concord_cut_large.stl");
	model.rotation(roll, 0.0 * deg, yaw).angle_of_attack(angle_of_attack).length(fuselage_length);
	Domain domain = Domain::around(model);
	domain.size(domain_width, domain_length, domain_height);
	domain.model_offset(0.0 * m, model_offset_y, model_offset_z);
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

	VideoRecorder& video = sim.video();
	Cameras::add_to(video, domain_length);
	video.set_length(video_length);
	sim.run_for(simulation_time);
}
