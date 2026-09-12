// Cessna 172 propeller aircraft
//
// Required extensions: FP16S, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SUBGRID
// STL from: https://www.thingiverse.com/thing:814319/files
// Note: Requires manually splitting Airplane.stl into body and rotor components.

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() {
	const float32_t wingspan_m = 11.0f;
	const float32_t flight_speed_mps = 226.0f / 3.6f;  // 226 km/h
	const float32_t simulation_time_s = 1.0f;
	const uint32_t update_interval = 4u;

	// Check for required STL files
	const string body_path = get_resource_path("Cessna-172-Skyhawk-body.stl");
	const string rotor_path = get_resource_path("Cessna-172-Skyhawk-rotor.stl");
	if(body_path.empty() || rotor_path.empty()) {
		print_info("This example requires manually splitting Airplane.stl into body and rotor components.");
		print_info("Steps:");
		print_info("  1. Download Airplane.stl: cd resources && python download_all_thingiverse_stl.py");
		print_info("  2. Open Airplane.stl in Microsoft 3D Builder");
		print_info("  3. Separate body and propeller into 2 meshes");
		print_info("  4. Save as Cessna-172-Skyhawk-body.stl and Cessna-172-Skyhawk-rotor.stl");
		print_info("  5. Place both files in resources/");
		wait();
		return;
	}

	// Configure static body
	SimulationSetup sim(SimulationConfig("Cessna-172-Skyhawk-body.stl")
		.set_domain_aspect_ratio(1.0f, 0.8f, 0.25f)
		.set_vram_mb(8000u)
		.set_geometry_scale(0.95f)
		.set_reference_axis(SimulationConfig::ReferenceAxis::X));

	sim.setup();
	sim.configure_units_with_length(wingspan_m, flight_speed_mps, Fluid::AIR);
	sim.print_reynolds_number(Fluid::AIR);

	LBM lbm = sim.create_lbm(Fluid::AIR);

	// Voxelize body
	sim.voxelize(lbm);

	// Configure boundaries
	BoundaryBuilder(lbm)
		.set_open_boundaries()
		.initialize_velocity_y(flight_speed_mps)
		.apply();

	// Configure propeller
	MovingPartsManager parts(sim, lbm);
	parts.add(MovingPart("Cessna-172-Skyhawk-rotor.stl")
		.set_rotation_axis(RotationAxis::Y)
		.set_tip_speed_mps(flight_speed_mps)
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
	print_info(to_string(simulation_time_s, 3u) + " seconds = " + to_string(lbm_T) + " time steps");

#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
	VideoRecorder()
		.add("front", CameraConfig()
			.set_free_position(0.192778f, -0.669183f, 0.657584f)
			.set_angles(-77.0f, 27.0f)
			.set_fov(100.0f))
		.add("bottom", CameraConfig()
			.set_free_position(0.224926f, -0.594332f, -0.277894f)
			.set_angles(-65.0f, -14.0f)
			.set_fov(100.0f))
		.add("back", CameraConfig()
			.set_free_position(0.0f, 0.650189f, 1.461048f)
			.set_angles(90.0f, 40.0f)
			.set_fov(100.0f))
		.set_video_length_s(5.0f)
		.record(lbm, simulation_time_s, units, [&]() { parts.update(); }, update_interval);
#else
	parts.run(simulation_time_s, units);
#endif
}
