// Bell 222 helicopter
//
// Required extensions: FP16C, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SUBGRID
// STL from: https://www.thingiverse.com/thing:1625155/files
// Note: Requires manually splitting BELL222__FIXED.stl into body and rotor components.

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() {
	const Length fuselage_length = 12.85_m; // Bell 222B, as in the original
	const Length rotor_diameter = 12.12_m;
	const Frequency rotor_speed = 348.0f / 60.0_s; // 348 rpm
	const Speed tip_speed = rotor_speed * rotor_diameter * pif;
	const Duration simulation_time = 2.0f / rotor_speed; // 2 revolutions of the main rotor

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

	// the fuselage (Y) is 80 % of the domain width, as in the original
	const Length domain_width = fuselage_length / 0.8f;
	SimulationSetup sim(Domain::around(Model("Bell-222-body.stl").length(fuselage_length))
		.size(domain_width, 1.2f * domain_width, 0.3f * domain_width)
		.vram(8000_mb));

	sim.setup();
	sim.configure_units(tip_speed, Fluid::AIR, LatticeMach(0.277f)); // the original's lattice tip speed 0.16
	sim.print_reynolds_number(Fluid::AIR);

	LBM lbm = sim.create_lbm(Fluid::AIR);

	// Voxelize body
	sim.voxelize(lbm);

	// Configure boundaries - forward flight with slight descent
	const Speed forward_velocity = 0.2f * tip_speed;
	const Speed descent_velocity = -0.1f * tip_speed;
	BoundaryBuilder(lbm)
		.set_open_boundaries()
		.initialize_velocity({ 0.0_mps, forward_velocity, descent_velocity })
		.apply();

	// Configure rotors
	MovingPartsManager parts(sim, lbm);

	// Main rotor - rotates around Z axis
	parts.add(MovingPart("Bell-222-main.stl")
		.set_rotation_axis(RotationAxis::Z)
		.set_tip_speed(tip_speed));

	// Tail rotor - rotates around X axis (reversed direction)
	parts.add(MovingPart("Bell-222-back.stl")
		.set_rotation_axis(RotationAxis::X)
		.set_tip_speed(tip_speed)
		.reverse_direction());

	parts.initialize();

	// Configure graphics
	GraphicsConfig(lbm)
		.show_surface()
		.show_vortices()
		.apply();

	// Run simulation
#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
	const Length W = domain_width; // camera positions from the domain's origin corner
	VideoRecorder()
		.add("a", CameraView::at({ 1.02851f * W, 0.722514f * W, 0.540685f * W }, 16_deg, 47_deg).field_of_view(96_deg))
		.add("b", [W](float progress) { // turns with the main rotor through its two revolutions, as the original
			return CameraView::at({ 0.5f * W, 0.462907f * W, 0.312979f * W }, 90_deg + progress * 720_deg, 36_deg).field_of_view(120_deg);
		})
		.add("c", CameraView::at({ 1.05772f * W, -0.0040656f * W, -0.0275928f * W }, -43_deg, -21_deg).field_of_view(75_deg))
		.add("d", CameraView::orbit(58_deg, 9_deg).field_of_view(88_deg).view_height(1.2f * W / 1.648722f))
		.add("e", CameraView::orbit(0_deg, 90_deg).view_height(1.2f * W / 1.1f))
		.add("f", CameraView::at({ 0.501612f * W, 1.22862f * W, 0.447784f * W }, 90_deg, 37_deg).field_of_view(94_deg))
		.set_video_length(10.0_s)
		.record(lbm, simulation_time, parts);
#else
	parts.run(simulation_time);
#endif
}
