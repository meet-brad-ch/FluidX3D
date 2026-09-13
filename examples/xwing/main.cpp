// Star Wars X-wing aerodynamics
//
// Extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID
// STL from: https://www.thingiverse.com/thing:353276/files

#include "setup/setup.hpp"

void main_setup() {
	const Length fuselage_length = 13.4_m; // T-65 X-wing; the model's longest side
	const Speed flight_speed = 100.0_mps;

	// the fuselage (Y) is as long as the domain is wide, half its length
	Simulation sim(Domain::around(Model("X-Wing.stl").rotation(0_deg, 0_deg, 180_deg).length(fuselage_length))
		.size(fuselage_length, 2.0f * fuselage_length, 0.5f * fuselage_length)
		.model_offset(0_m, -0.45f * fuselage_length, 0_m)
		.vram(880_mb),
		Fluid::AIR, flight_speed, LatticeMach(0.13f)); // the original's lattice speed 0.075

	sim.boundaries()
		.set_open_boundaries()
		.initialize_velocity_y(flight_speed)
		.apply();

	sim.graphics()
		.show_surface()
		.show_vortices()
		.apply();

	const Length L = fuselage_length; // camera positions from the domain's origin corner
	sim.video()
		.add("t", CameraView::at({ 1.5f * L, 0.2f * L, 1.25f * L }, -33_deg, 42_deg).field_of_view(68_deg))
		.add("b", CameraView::at({ 1.0f * L, 0.3f * L, -0.1f * L }, -33_deg, -40_deg))
		.add("f", CameraView::at({ 0.5f * L, 2.02f * L, 0.625f * L }, 90_deg, 28_deg).field_of_view(80_deg))
		.add("s", CameraView::at({ 1.2f * L, 0.7f * L, 0.28f * L }))
		.set_length(30.0_s);
	sim.run_for(10.0_s);
}
