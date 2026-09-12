// Electric Ducted Fan (EDF)
//
// Required extensions: FP16S, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SUBGRID
// STL from: https://www.thingiverse.com/thing:3014759/files
//
// Note: This example uses MovingPartsManager for the rotor.

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() {
	const Length fan_diameter = 0.09_m;          // 90 mm EDF
	const Speed tip_speed = 100.0_mps;           // Blade tip speed
	const Speed inlet_velocity = 30.0_mps;       // 30% of tip speed
	const Duration simulation_time = 0.5_s;
	const Duration update_interval = 0.8_us; // 4 time steps, as the original

	// the stator's diameter (X and Z) is 98 % of the domain width; the domain is 1.5 widths long (Y)
	const Length domain_width = fan_diameter / 0.98f;
	SimulationSetup sim(Domain::around(Model("edf_v39.stl").rotation(0_deg, 0_deg, 180_deg).length(fan_diameter, Axis::X))
		.size(domain_width, 1.5f * domain_width, domain_width)
		.model_offset(0_m, -0.2f * fan_diameter, 0_m) // stator position
		.vram(8000_mb));

	sim.setup();
	sim.configure_units(tip_speed, Fluid::AIR);
	sim.print_reynolds_number(Fluid::AIR);

	// Create LBM
	LBM lbm = sim.create_lbm(Fluid::AIR);

	// Voxelize stator
	sim.voxelize(lbm);

	// Configure boundaries - open with inlet velocity
	BoundaryBuilder(lbm)
		.set_open_boundaries()
		.initialize_velocity_y(inlet_velocity)
		.apply();

	// the rotor's STL is not in the stator's coordinates (the duct's axis is off its origin): centered on the duct's
	// axis, 0.21 diameters behind the stator's center, as in the original
	MovingPartsManager parts(sim, lbm);
	parts.add(MovingPart("edf_v391.stl")
		.set_rotation_axis(RotationAxis::Y)
		.set_tip_speed(tip_speed)
		.centered_on_model(Position{ 0_m, -0.21f * fan_diameter, 0_m })
		.set_update_interval(update_interval));
	parts.initialize();

	// Configure graphics
	GraphicsConfig(lbm)
		.show_surface()
		.show_vortices()
		.apply();

	// Run simulation
#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
	VideoRecorder()
		.add([domain_width](float progress) { // pans around the fan during the video
			return CameraView::orbit(-70_deg + progress * 100_deg, 2_deg).field_of_view(60_deg).view_height(1.5f * domain_width / 1.284025f);
		})
		.set_video_length(30.0_s)
		.record(lbm, simulation_time, parts);
#else
	parts.run(simulation_time);
#endif
}
