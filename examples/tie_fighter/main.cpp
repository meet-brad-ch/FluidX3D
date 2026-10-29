/// @file main.cpp
/// @brief Star Wars TIE Fighter, tumbling: the fighter turns as a moving part (MovingPart::set_tumble()) while the
/// air flows past it at 50 m/s, recorded from four cameras.
///
/// Extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID
/// STL from: https://www.thingiverse.com/thing:2919109/files

#include "setup/setup.hpp"

namespace {

/// @brief The video's four cameras, placed as in the original: above (t), below (b), in front (f) and at the side (s)
/// of the fighter.
class TieFighterCameras {
public:
	/// @brief Adds the four cameras to the video and sets the video's length to 30 s.
	/// @param video the simulation's video (Simulation::video())
	/// @param W the domain's width (X); the camera positions are multiples of it from the domain's origin corner
	static void add(VideoRecorder& video, si::Length W) {
		using namespace si::symbols;
		const si::Position top{ 1.5 * W, 0.2 * W, 1.13 * W };
		const si::Position bottom{ 0.8 * W, -2.0 * W, 0.05 * W };
		const si::Position front{ 0.5 * W, 2.14 * W, 1.2 * W };
		const si::Position side{ 3.0 * W, 1.0 * W, 0.5 * W };
		const si::Angle top_azimuth = -33.0 * deg;
		const si::Angle top_elevation = 33.0 * deg;
		const si::Angle top_field = 80.0 * deg;
		const si::Angle bottom_azimuth = -83.0 * deg;
		const si::Angle bottom_elevation = -10.0 * deg;
		const si::Angle bottom_field = 40.0 * deg;
		const si::Angle front_azimuth = 90.0 * deg;
		const si::Angle front_elevation = 29.5 * deg;
		const si::Angle front_field = 80.0 * deg;
		const si::Angle side_field = 50.0 * deg;
		const si::Duration video_length = 30.0 * s;
		video.add("t", CameraView::at(top, top_azimuth, top_elevation).field_of_view(top_field))
			.add("b", CameraView::at(bottom, bottom_azimuth, bottom_elevation).field_of_view(bottom_field))
			.add("f", CameraView::at(front, front_azimuth, front_elevation).field_of_view(front_field))
			.add("s", CameraView::at(side).field_of_view(side_field))
			.set_length(video_length);
	}
};

} // namespace

/// @brief Sets up the fighter (8.8 m high, 65 % of the domain's width) in a 1760 MiB grid, tumbles it half a turn
/// per second, and records a 30 s video of 5 s of simulated time.
///
/// The lattice Mach number gives the original's lattice speed 0.075; the part is turned every 2.2 ms (28 time steps),
/// as the original.
void main_setup() {
	using namespace si::symbols;
	const si::Length height = 8.8 * m;              // TIE/LN (Wookieepedia): the wing panels' height (Z), the longest side
	const si::Speed flow_velocity = 50.0 * m / s;
	const si::Duration simulation_time = 5.0 * s;   // 5 seconds of tumbling
	const si::Duration update_interval = 2.2 * ms;  // 28 time steps, as the original
	const si::Direction tumble_axis{ 0.2, 1.0, 0.1 };
	const si::AngularSpeed tumble_rate = 180.0 * deg / s; // half a turn per second (the original: 0.4° per 28 time steps)

	// the height (Z) is 65 % of the domain width; the fighter's center is about 0.6 heights from the inlet
	const si::Length domain_width = height / 0.65;
	const si::Length domain_length = 2.0 * domain_width;
	const si::Angle model_turn_x = 90.0 * deg;
	const si::Angle no_turn = 0.0 * deg;
	const si::Length model_offset_y = -0.94 * height;
	const si::Length no_offset = 0.0 * m;
	const si::MemoryAmount vram = 1760.0 * MiB;
	Model model("DWG_Tie_Fighter_Assembled_02.stl");
	model.rotation(model_turn_x, no_turn, no_turn).length(height, Axis::Z);
	Domain domain = Domain::around(model);
	domain.size(domain_width, domain_length, domain_width);
	domain.model_offset(no_offset, model_offset_y, no_offset);
	domain.vram(vram);
	const si::LatticeMach mach(0.13); // the original's lattice speed 0.075
	Simulation sim(domain, Fluid::AIR, flow_velocity, mach);
	sim.skip_model_voxelization(); // the fighter tumbles: a moving part, not a static solid

	MovingPart fighter(sim.model_file());
	fighter.set_tumble(tumble_axis, tumble_rate).set_update_interval(update_interval);
	sim.parts()
		.add(fighter)
		.initialize();

	sim.boundaries()
		.set_open_boundaries()
		.initialize_velocity_y(flow_velocity)
		.apply();

	sim.graphics()
		.show_flags()
		.show_surface()
		.show_vortices()
		.apply();

	TieFighterCameras::add(sim.video(), domain_width);
	sim.run_for(simulation_time);
}
