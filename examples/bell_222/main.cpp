// Bell 222 helicopter
//
// Required extensions: FP16C, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SUBGRID
// STL from: https://www.thingiverse.com/thing:1625155/files
// Note: Requires manually splitting BELL222__FIXED.stl into body and rotor components.

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() {
	const float32_t rotor_diameter_m = 12.12f;
	const float32_t rotor_rpm = 348.0f;
	const float32_t tip_speed_mps = rotor_rpm / 60.0f * rotor_diameter_m * pif;
	const float32_t simulation_time_s = 0.34483f;  // 2 revolutions of main rotor
	const uint32_t update_interval = 4u;

	// Check for required STL files
	const string body_path = get_resource_path("Bell-222-body.stl");
	const string main_path = get_resource_path("Bell-222-main.stl");
	const string back_path = get_resource_path("Bell-222-back.stl");
	if(body_path.empty() || main_path.empty() || back_path.empty()) {
		print_info("This example requires manually splitting BELL222__FIXED.stl into body and rotor components.");
		print_info("Steps:");
		print_info("  1. Download BELL222__FIXED.stl: cd resources && python download_all_thingiverse_stl.py");
		print_info("  2. Open BELL222__FIXED.stl in Microsoft 3D Builder");
		print_info("  3. Separate fuselage, main rotor, and tail rotor into 3 meshes");
		print_info("  4. Save as Bell-222-body.stl, Bell-222-main.stl, and Bell-222-back.stl");
		print_info("  5. Place all 3 files in resources/");
		wait();
		return;
	}

	// Configure body using Y axis as reference (fuselage length)
	SimulationSetup sim(SimulationConfig("Bell-222-body.stl")
		.set_domain_aspect_ratio(1.0f, 1.2f, 0.3f)
		.set_vram_mb(8000u)
		.set_geometry_scale(0.8f)
		.set_reference_axis(SimulationConfig::ReferenceAxis::Y));

	sim.setup();
	sim.configure_units(tip_speed_mps, Fluid::AIR);
	sim.print_reynolds_number(Fluid::AIR);

	LBM lbm = sim.create_lbm(Fluid::AIR);

	// Voxelize body
	sim.voxelize(lbm);

	// Configure boundaries - forward flight with slight descent
	const float32_t forward_velocity_mps = 0.2f * tip_speed_mps;
	const float32_t descent_velocity_mps = -0.1f * tip_speed_mps;
	BoundaryBuilder(lbm)
		.set_open_boundaries()
		.initialize_velocity(0.0f, forward_velocity_mps, descent_velocity_mps)
		.apply();

	// Configure rotors
	MovingPartsManager parts(sim, lbm);

	// Main rotor - rotates around Z axis
	parts.add(MovingPart("Bell-222-main.stl")
		.set_rotation_axis(RotationAxis::Z)
		.set_tip_speed_mps(tip_speed_mps)
		.set_update_interval(update_interval));

	// Tail rotor - rotates around X axis (reversed direction)
	parts.add(MovingPart("Bell-222-back.stl")
		.set_rotation_axis(RotationAxis::X)
		.set_tip_speed_mps(tip_speed_mps)
		.reverse_direction()
		.set_update_interval(update_interval));

	parts.initialize();

	// Configure graphics
	GraphicsConfig(lbm)
		.show_surface()
		.show_vortices()
		.apply();

	// Run simulation
	const uint64_t lbm_T = sim.to_lbm_timesteps(simulation_time_s);
	print_info(to_string(simulation_time_s, 5u) + " seconds = " + to_string(lbm_T) + " time steps");

#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
	VideoRecorder()
		.add("a", CameraConfig()
			.set_free_position(0.528513f, 0.102095f, 1.302283f)
			.set_angles(16.0f, 47.0f)
			.set_fov(96.0f))
		.add("b", CameraConfig()
			.set_free_position(0.0f, -0.114244f, 0.543265f)
			.set_angles(90.0f, 36.0f)
			.set_fov(120.0f))
		.add("c", CameraConfig()
			.set_free_position(0.557719f, -0.503388f, -0.591976f)
			.set_angles(-43.0f, -21.0f)
			.set_fov(75.0f))
		.add("d", CameraConfig()
			.set_centered_position(58.0f, 9.0f)
			.set_fov(88.0f)
			.set_zoom(1.648722f))
		.add("e", CameraConfig()
			.set_centered_position(0.0f, 90.0f)
			.set_fov(100.0f)
			.set_zoom(1.100000f))
		.add("f", CameraConfig()
			.set_free_position(0.001612f, 0.523852f, 0.992613f)
			.set_angles(90.0f, 37.0f)
			.set_fov(94.0f))
		.set_fps(10.0f)
		.record(lbm, simulation_time_s, units, [&]() { parts.update(); }, update_interval);
#else
	parts.run(simulation_time_s, units);
#endif
}
