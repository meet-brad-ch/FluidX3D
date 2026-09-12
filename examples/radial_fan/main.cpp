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
	const Length fan_diameter = 0.3_m;           // 30 cm fan
	const Speed tip_speed = 30.0_mps;            // Blade tip speed
	const Duration simulation_time = 1.0_s;      // 1 second of rotation
	const uint32_t update_interval = 10u;

	// the fan fills half of the enclosure's width, near the floor
	SimulationSetup sim(Domain::around(Model("FAN_Solid_Bottom.stl").length(fan_diameter))
		.size(2.0f * fan_diameter, 2.0f * fan_diameter, 2.0f / 3.0f * fan_diameter)
		.model_offset(0_m, 0_m, -0.154f * fan_diameter)
		.vram(181_mb));

	sim.setup();
	sim.configure_units(tip_speed, Fluid::AIR);
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
		.set_tip_speed(tip_speed)
		.set_update_interval(update_interval));
	parts.initialize();

	// Configure graphics
	GraphicsConfig(lbm)
		.show_lattice()
		.show_surface()
		.show_vortices()
		.apply();

	// Run simulation
	const uint64_t total_timesteps = sim.to_lbm_timesteps(simulation_time);
	print_info(to_string(simulation_time.si(), 1u) + " seconds = " + to_string(total_timesteps) + " time steps");

#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
	VideoRecorder()
		.add(CameraConfig()
			.set_free_position(0.353512f, -0.150326f, 1.643939f)
			.set_angles(-25.0f, 61.0f)
			.set_fov(100.0f))
		.set_video_length(30.0_s)
		.record(lbm, simulation_time, [&]() { parts.update(); }, update_interval);
#else
	parts.run(simulation_time);
#endif
}
