// Star Wars TIE Fighter (tumbling)
//
// Required extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID
// STL from: https://www.thingiverse.com/thing:2919109/files
//
// Demonstrates tumbling simulation using MovingPartsManager with set_tumble().
// The mesh rotates around an arbitrary axis while flowing through the domain.

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() {
	const Length wingspan = 6.0_m;               // TIE Fighter wingspan
	const float32_t flow_velocity_mps = 50.0f;   // Flow velocity
	const float32_t simulation_time_s = 5.0f;    // 5 seconds of tumbling
	const uint32_t update_interval = 28u;

	// the wingspan (X) is 65 % of the domain width; the fighter's center is about 0.6 wingspans from the inlet
	const Length domain_width = wingspan / 0.65f;
	SimulationSetup sim(Domain::around(Model("DWG_Tie_Fighter_Assembled_02.stl").rotation(90_deg, 0_deg, 0_deg).length(wingspan, Axis::X))
		.size(domain_width, 2.0f * domain_width, domain_width)
		.model_offset(0_m, -0.94f * wingspan, 0_m)
		.vram(1760_mb));

	sim.setup();
	sim.configure_units(flow_velocity_mps, Fluid::AIR);
	sim.print_reynolds_number(Fluid::AIR);

	// Create LBM
	LBM lbm = sim.create_lbm(Fluid::AIR);

	// Configure tumbling using MovingPartsManager
	// Uses the same geometry as the main config, with arbitrary axis rotation
	MovingPartsManager parts(sim, lbm);
	parts.add(MovingPart(sim.get_geometry_filename())
		.set_tumble(float3(0.2f, 1.0f, 0.1f), radians(0.4032f))
		.set_update_interval(update_interval));
	parts.initialize();

	// Configure boundaries - open with inlet velocity
	BoundaryBuilder(lbm)
		.set_all_open()
		.initialize_velocity_y(flow_velocity_mps)
		.apply();

	// Configure graphics (includes lattice visualization for tumbling effect)
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
		.add("t", CameraConfig()
			.set_free_position(1.0f, -0.4f, 0.63f)
			.set_angles(-33.0f, 33.0f)
			.set_fov(80.0f))
		.add("b", CameraConfig()
			.set_free_position(0.3f, -1.5f, -0.45f)
			.set_angles(-83.0f, -10.0f)
			.set_fov(40.0f))
		.add("f", CameraConfig()
			.set_free_position(0.0f, 0.57f, 0.7f)
			.set_angles(90.0f, 29.5f)
			.set_fov(80.0f))
		.add("s", CameraConfig()
			.set_free_position(2.5f, 0.0f, 0.0f)
			.set_angles(0.0f, 0.0f)
			.set_fov(50.0f))
		.set_video_length_s(30.0f)
		.record(lbm, simulation_time_s, units, [&]() { parts.update(); }, update_interval);
#else
	parts.run(simulation_time_s, units);
#endif
}
