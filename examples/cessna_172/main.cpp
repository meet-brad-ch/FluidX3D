/// @file main.cpp
/// @brief Cessna 172 propeller aircraft: the airflow at 226 km/h around the aircraft, with a turning propeller,
/// recorded from three cameras.
///
/// Extensions: FP16S, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SUBGRID
/// STL from: https://www.thingiverse.com/thing:814319/files
/// Note: Requires manually splitting Airplane.stl into body and rotor components.

#include "setup/setup.hpp"

/// @brief The video's cameras: their positions, as parts of the domain's width from the domain's origin
/// corner, and their directions, as the original.
class Cameras {
public:
	/// @brief Adds the cameras to the video.
	/// @param video the simulation's video recorder
	/// @param reference the domain's width: the camera positions are parts of it
	static void add_to(VideoRecorder& video, si::Length reference) {
		using namespace si::symbols;
		const si::Length W = reference; // camera positions from the domain's origin corner
		const si::Position front{ 0.692778 * W, -0.135346 * W, 0.289396 * W };
		const si::Position bottom{ 0.724926 * W, -0.0754656 * W, 0.0555265 * W };
		const si::Position back{ 0.5 * W, 0.920151 * W, 0.490262 * W };
		const si::Angle front_azimuth = -77.0 * deg;
		const si::Angle front_elevation = 27.0 * deg;
		const si::Angle bottom_azimuth = -65.0 * deg;
		const si::Angle bottom_elevation = -14.0 * deg;
		const si::Angle back_azimuth = 90.0 * deg;
		const si::Angle back_elevation = 40.0 * deg;
		const CameraView front_view = CameraView::at(front, front_azimuth, front_elevation);
		const CameraView bottom_view = CameraView::at(bottom, bottom_azimuth, bottom_elevation);
		const CameraView back_view = CameraView::at(back, back_azimuth, back_elevation);
		video.add("front", front_view);
		video.add("bottom", bottom_view);
		video.add("back", back_view);
	}
};

/// @brief Sets up the aircraft (an 11 m wingspan, 95 % of the domain's width) in an 8000 MiB grid, turns the
/// propeller with a tip speed equal to the flight speed, and records a 5 s video of 1 s of simulated time.
///
/// Ends the program with print_error() and the splitting instructions when a part's STL is missing from resources/.
void main_setup() {
	using namespace si::symbols;
	const si::Length wingspan = 11.0 * m;
	const si::Speed flight_speed = 226.0 * kmh;
	const si::Duration simulation_time = 1.0 * s;
	const si::Duration video_length = 5.0 * s;

	// static body: the wingspan (X) is 95 % of the domain width
	const double wingspan_in_width = 0.95;
	const si::Length domain_width = wingspan / wingspan_in_width;
	const si::Length domain_length = 0.8 * domain_width;
	const si::Length domain_height = 0.25 * domain_width;
	const si::MemoryAmount vram = 8000.0 * MiB;
	Model body("Cessna-172-Skyhawk-body.stl");
	body.length(wingspan, Axis::X);
	body.needs({ "Cessna-172-Skyhawk-rotor.stl" });
	body.instructions({ "This example requires manually splitting Airplane.stl into body and rotor components.",
	                    "Steps:",
	                    "  1. Download Airplane.stl: cd resources && python download_all_thingiverse_stl.py",
	                    "  2. Open Airplane.stl in Microsoft 3D Builder",
	                    "  3. Separate body and propeller into 2 meshes",
	                    "  4. Save as Cessna-172-Skyhawk-body.stl and Cessna-172-Skyhawk-rotor.stl",
	                    "  5. Place both files in resources/" });
	Domain domain = Domain::around(body);
	domain.size(domain_width, domain_length, domain_height);
	domain.vram(vram);
	Simulation sim(domain, Fluid::AIR, flight_speed);

	sim.boundaries()
		.set_open_boundaries()
		.initialize_velocity_y(flight_speed)
		.apply();

	// the propeller
	MovingPart propeller("Cessna-172-Skyhawk-rotor.stl");
	propeller.set_rotation_axis(Axis::Y).set_tip_speed(flight_speed).reverse_direction();
	sim.parts()
		.add(propeller)
		.initialize();

	sim.graphics()
		.show_surface()
		.show_vortices()
		.apply();

	VideoRecorder& video = sim.video();
	Cameras::add_to(video, domain_width);
	video.set_length(video_length);
	sim.run_for(simulation_time);
}
