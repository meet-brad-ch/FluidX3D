// Cessna 172 propeller aircraft
//
// Required extensions: FP16S, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SUBGRID
// STL from: https://www.thingiverse.com/thing:814319/files
// Note: Requires manually splitting Airplane.stl into body and rotor components.

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() {
	const Length wingspan = 11.0_m;
	const Speed flight_speed = 226.0_kmh;
	const Duration simulation_time = 1.0_s;

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

	// static body: the wingspan (X) is 95 % of the domain width
	const Length domain_width = wingspan / 0.95f;
	SimulationSetup sim(Domain::around(Model("Cessna-172-Skyhawk-body.stl").length(wingspan, Axis::X))
		.size(domain_width, 0.8f * domain_width, 0.25f * domain_width)
		.vram(8000_mb));

	sim.setup();
	sim.configure_units(flight_speed, Fluid::AIR);
	sim.print_reynolds_number(Fluid::AIR);

	LBM lbm = sim.create_lbm(Fluid::AIR);

	// Voxelize body
	sim.voxelize(lbm);

	// Configure boundaries
	BoundaryBuilder(lbm)
		.set_open_boundaries()
		.initialize_velocity_y(flight_speed)
		.apply();

	// Configure propeller
	MovingPartsManager parts(sim, lbm);
	parts.add(MovingPart("Cessna-172-Skyhawk-rotor.stl")
		.set_rotation_axis(RotationAxis::Y)
		.set_tip_speed(flight_speed)
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
		.add("front", CameraView::at({ 0.692778f * W, -0.135346f * W, 0.289396f * W }, -77_deg, 27_deg))
		.add("bottom", CameraView::at({ 0.724926f * W, -0.0754656f * W, 0.0555265f * W }, -65_deg, -14_deg))
		.add("back", CameraView::at({ 0.5f * W, 0.920151f * W, 0.490262f * W }, 90_deg, 40_deg))
		.set_video_length(5.0_s)
		.record(lbm, simulation_time, parts);
#else
	parts.run(simulation_time);
#endif
}
