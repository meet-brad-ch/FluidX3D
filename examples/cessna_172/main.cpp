// Cessna 172 propeller aircraft
//
// Extensions: FP16S, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SUBGRID
// STL from: https://www.thingiverse.com/thing:814319/files
// Note: Requires manually splitting Airplane.stl into body and rotor components.

#include "setup/setup.hpp"

void main_setup() {
	const Length wingspan = 11.0_m;
	const Speed flight_speed = 226.0_kmh;
	const Duration simulation_time = 1.0_s;

	// static body: the wingspan (X) is 95 % of the domain width
	const Length domain_width = wingspan / 0.95f;
	const Model body = Model("Cessna-172-Skyhawk-body.stl").length(wingspan, Axis::X)
		.needs({ "Cessna-172-Skyhawk-rotor.stl" })
		.instructions({ "This example requires manually splitting Airplane.stl into body and rotor components.",
		                "Steps:",
		                "  1. Download Airplane.stl: cd resources && python download_all_thingiverse_stl.py",
		                "  2. Open Airplane.stl in Microsoft 3D Builder",
		                "  3. Separate body and propeller into 2 meshes",
		                "  4. Save as Cessna-172-Skyhawk-body.stl and Cessna-172-Skyhawk-rotor.stl",
		                "  5. Place both files in resources/" });
	Simulation sim(Domain::around(body)
		.size(domain_width, 0.8f * domain_width, 0.25f * domain_width)
		.vram(8000_mb),
		Fluid::AIR, flight_speed);

	sim.boundaries()
		.set_open_boundaries()
		.initialize_velocity_y(flight_speed)
		.apply();

	// the propeller
	sim.parts()
		.add(MovingPart("Cessna-172-Skyhawk-rotor.stl")
			.set_rotation_axis(Axis::Y)
			.set_tip_speed(flight_speed)
			.reverse_direction())
		.initialize();

	sim.graphics()
		.show_surface()
		.show_vortices()
		.apply();

	const Length W = domain_width; // camera positions from the domain's origin corner
	sim.video()
		.add("front", CameraView::at({ 0.692778f * W, -0.135346f * W, 0.289396f * W }, -77_deg, 27_deg))
		.add("bottom", CameraView::at({ 0.724926f * W, -0.0754656f * W, 0.0555265f * W }, -65_deg, -14_deg))
		.add("back", CameraView::at({ 0.5f * W, 0.920151f * W, 0.490262f * W }, 90_deg, 40_deg))
		.set_length(5.0_s);
	sim.run_for(simulation_time);
}
