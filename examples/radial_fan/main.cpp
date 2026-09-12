// Radial Fan
//
// Required extensions: FP16S, MOVING_BOUNDARIES, SUBGRID
// STL from: https://www.thingiverse.com/thing:6113/files
//
// A rotating fan in an open-top enclosure demonstrating moving boundary conditions.

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() {
	const float32_t fan_diameter_m = 0.3f;       // 30cm fan
	const float32_t tip_speed_mps = 30.0f;       // Blade tip speed
	const float32_t simulation_time_s = 1.0f;    // 1 second of rotation
	const uint32_t update_interval = 10u;

	// Configure domain - 3:3:1 aspect ratio with fan at 50% of domain width
	SimulationSetup sim(SimulationConfig("FAN_Solid_Bottom.stl")
		.set_domain_aspect_ratio(3.0f, 3.0f, 1.0f)
		.set_vram_mb(181u)
		.set_geometry_scale(0.5f)
		.set_center_offset_ratio(0.0f, 0.0f, -0.154f));  // Position fan near floor

	sim.setup();
	sim.configure_units_with_length(fan_diameter_m, tip_speed_mps, Fluid::AIR);
	sim.print_reynolds_number(Fluid::AIR);

	// Create LBM
	LBM lbm = sim.create_lbm(Fluid::AIR);

	// Configure boundaries - solid floor and walls, open top
	BoundaryBuilder(lbm)
		.set_solid_floor()
		.set_solid_walls()
		.apply();

	// Configure rotating fan
	MovingPartsManager parts(sim, lbm);
	parts.add(MovingPart("FAN_Solid_Bottom.stl")
		.set_rotation_axis(RotationAxis::Z)
		.set_tip_speed_mps(tip_speed_mps)
		.set_update_interval(update_interval));
	parts.initialize();

	// Configure graphics
	GraphicsConfig(lbm)
		.show_lattice()
		.show_surface()
		.show_vortices()
		.apply();

	// Run simulation
	const uint64_t total_timesteps = sim.to_lbm_timesteps(simulation_time_s);
	print_info(to_string(simulation_time_s, 1u) + " seconds = " + to_string(total_timesteps) + " time steps");

#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
	VideoRecorder()
		.add(CameraConfig()
			.set_free_position(0.353512f, -0.150326f, 1.643939f)
			.set_angles(-25.0f, 61.0f)
			.set_fov(100.0f))
		.set_video_length_s(30.0f)
		.record(lbm, simulation_time_s, units, [&]() { parts.update(); }, update_interval);
#else
	parts.run(simulation_time_s, units);
#endif
}
