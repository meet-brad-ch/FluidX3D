/// @file main.cpp
/// @brief Bell 222 helicopter: forward flight with a slight descent, with a turning main rotor and tail rotor,
/// recorded from six cameras.
///
/// Extensions: FP16C, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SUBGRID
/// STL from: https://www.thingiverse.com/thing:1625155/files
/// Note: Requires manually splitting BELL222__FIXED.stl into body and rotor components.

#include "setup/setup.hpp"

namespace {

/// @brief The video's six cameras, placed as in the original: three views from given positions (a, c, f), an orbit
/// from the side (d), a view from above (e), and a view that turns with the main rotor (b).
class Bell222Cameras {
public:
	/// @brief Adds the six cameras to the video and sets the video's length to 10 s.
	/// @param video the simulation's video (Simulation::video())
	/// @param W the domain's width (X); the camera positions are multiples of it from the domain's origin corner
	static void add(VideoRecorder& video, si::Length W) {
		using namespace si::symbols;
		const si::Position camera_a{ 1.02851 * W, 0.722514 * W, 0.540685 * W };
		const si::Position camera_c{ 1.05772 * W, -0.0040656 * W, -0.0275928 * W };
		const si::Position camera_f{ 0.501612 * W, 1.22862 * W, 0.447784 * W };
		const si::Length side_height = 1.2 * W / 1.648722;
		const si::Length top_height = 1.2 * W / 1.1;
		const si::Angle a_azimuth = 16.0 * deg;
		const si::Angle a_elevation = 47.0 * deg;
		const si::Angle a_field = 96.0 * deg;
		const si::Angle c_azimuth = -43.0 * deg;
		const si::Angle c_elevation = -21.0 * deg;
		const si::Angle c_field = 75.0 * deg;
		const si::Angle d_azimuth = 58.0 * deg;
		const si::Angle d_elevation = 9.0 * deg;
		const si::Angle d_field = 88.0 * deg;
		const si::Angle e_azimuth = 0.0 * deg;
		const si::Angle e_elevation = 90.0 * deg; // straight down
		const si::Angle f_azimuth = 90.0 * deg;
		const si::Angle f_elevation = 37.0 * deg;
		const si::Angle f_field = 94.0 * deg;
		const si::Duration video_length = 10.0 * s;
		const VideoRecorder::CameraPath with_rotor = rotor_camera(W);
		video.add("a", CameraView::at(camera_a, a_azimuth, a_elevation).field_of_view(a_field))
			.add("b", with_rotor)
			.add("c", CameraView::at(camera_c, c_azimuth, c_elevation).field_of_view(c_field))
			.add("d", CameraView::orbit(d_azimuth, d_elevation).field_of_view(d_field).view_height(side_height))
			.add("e", CameraView::orbit(e_azimuth, e_elevation).view_height(top_height))
			.add("f", CameraView::at(camera_f, f_azimuth, f_elevation).field_of_view(f_field))
			.set_length(video_length);
	}

private:
	/// @brief Makes camera b, which turns with the main rotor through its two revolutions, as the original.
	/// @param W the domain's width (X); the camera's position is a multiple of it from the domain's origin corner
	/// @return the camera's view at each video progress, from 0 (the start) to 1 (the end)
	static VideoRecorder::CameraPath rotor_camera(si::Length W) {
		using namespace si::symbols;
		const si::Position camera_b{ 0.5 * W, 0.462907 * W, 0.312979 * W };
		const VideoRecorder::CameraPath path = [camera_b](float progress) {
			const si::Angle azimuth = 90.0 * deg + static_cast<double>(progress) * (720.0 * deg);
			const si::Angle elevation = 36.0 * deg;
			const si::Angle field = 120.0 * deg;
			const CameraView view = CameraView::at(camera_b, azimuth, elevation).field_of_view(field);
			return view;
		};
		return path;
	}
};

} // namespace

/// @brief Sets up the helicopter (its fuselage 12.85 m long, 80 % of the domain's width) in an 8000 MiB grid, turns
/// the main rotor at 348 rpm and the tail rotor the other way, and records a 10 s video of two revolutions of the
/// main rotor.
///
/// The lattice Mach number gives the original's lattice tip speed 0.16. Ends the program with print_error() and the
/// splitting instructions when a part's STL is missing from resources/.
void main_setup() {
	using namespace si::symbols;
	const si::Length fuselage_length = 12.85 * m; // Bell 222B, as in the original
	const si::Length rotor_diameter = 12.12 * m;
	const si::Frequency rotor_speed = 348.0 / (60.0 * s); // 348 rpm
	const si::Speed tip_speed = rotor_speed * rotor_diameter * pi;
	const si::Duration simulation_time = 2.0 / rotor_speed; // 2 revolutions of the main rotor

	// the fuselage (Y) is 80 % of the domain width, as in the original
	const si::Length domain_width = fuselage_length / 0.8;
	Model body("Bell-222-body.stl");
	body.length(fuselage_length);
	body.needs({ "Bell-222-main.stl", "Bell-222-back.stl" });
	body.instructions({ "This example requires manually splitting BELL222__FIXED.stl into body and rotor components.",
	                    "Steps:",
	                    "  1. Download BELL222__FIXED.stl: cd resources && python download_all_thingiverse_stl.py",
	                    "  2. Open BELL222__FIXED.stl in Microsoft 3D Builder",
	                    "  3. Separate fuselage, main rotor, and tail rotor into 3 meshes",
	                    "  4. Save as Bell-222-body.stl, Bell-222-main.stl, and Bell-222-back.stl",
	                    "  5. Place all 3 files in resources/" });
	Domain domain = Domain::around(body);
	const si::Length domain_length = 1.2 * domain_width; // Y, as the original
	const si::Length domain_height = 0.3 * domain_width;
	const si::MemoryAmount vram = 8000.0 * MiB;
	domain.size(domain_width, domain_length, domain_height);
	domain.vram(vram);
	const si::LatticeMach mach(0.277); // the original's lattice tip speed 0.16
	Simulation sim(domain, Fluid::AIR, tip_speed, mach);

	// forward flight with a slight descent
	const si::Speed forward_velocity = 0.2 * tip_speed;
	const si::Speed descent_velocity = -0.1 * tip_speed;
	const si::Velocity flight{ 0.0 * m / s, forward_velocity, descent_velocity };
	sim.boundaries()
		.set_open_boundaries()
		.initialize_velocity(flight)
		.apply();

	// the main rotor turns about Z, the tail rotor about X the other way
	MovingPart main_rotor("Bell-222-main.stl");
	main_rotor.set_rotation_axis(Axis::Z).set_tip_speed(tip_speed);
	MovingPart tail_rotor("Bell-222-back.stl");
	tail_rotor.set_rotation_axis(Axis::X).set_tip_speed(tip_speed).reverse_direction();
	sim.parts()
		.add(main_rotor)
		.add(tail_rotor)
		.initialize();

	sim.graphics()
		.show_surface()
		.show_vortices()
		.apply();

	Bell222Cameras::add(sim.video(), domain_width);
	sim.run_for(simulation_time);
}
