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
	const Duration update_interval = 90_us;  // 10 time steps, as the original

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
#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
	const Length d = fan_diameter; // camera position from the domain's origin corner
	VideoRecorder()
		.add(CameraView::at({ 1.70702f * d, 0.699348f * d, 1.42929f * d }, -25_deg, 61_deg))
		.set_video_length(30.0_s)
		.record(lbm, simulation_time, parts);
#else
	parts.run(simulation_time);
#endif
}
