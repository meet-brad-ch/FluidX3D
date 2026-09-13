// SpaceX Starship aerodynamics
//
// Extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID
// STL from: https://www.thingiverse.com/thing:4912729/files

#include "setup/setup.hpp"

void main_setup() {
	const Length ship_length = 50.0_m; // Starship upper stage; the model's longest side
	const Speed reentry_speed = 100.0_mps;

	// the ship (Y) is 80 % of the domain length; the air flows up (+z) past it, as in a belly-first reentry
	const Length domain_length = ship_length / 0.8f;
	Simulation sim(Domain::around(Model("StarShipV2.stl").length(ship_length).repair_mesh())
		.size(0.5f * domain_length, domain_length, domain_length)
		.model_offset(0_m, 0.05f * ship_length, -0.445f * ship_length)
		.vram(1000_mb),
		Fluid::AIR, reentry_speed, LatticeMach(0.0866f)); // the original's lattice speed 0.05

	sim.boundaries()
		.set_open_boundaries()
		.initialize_velocity_z(reentry_speed)
		.apply();

	sim.graphics()
		.show_flags()
		.show_surface()
		.show_vortices()
		.apply();

	const Length D = domain_length; // camera positions from the domain's origin corner
	sim.video()
		.add("top", CameraView::at({ 1.30837f * D, -0.275261f * D, 1.52658f * D }, -38_deg, 37_deg).field_of_view(60_deg))
		.add("bottom", CameraView::at({ 0.609471f * D, 0.811263f * D, 0.001634f * D }, 32_deg, -40_deg).field_of_view(104_deg))
		.add("side", CameraView::at({ 1.12406f * D, 0.942782f * D, 0.587945f * D }, 24_deg, 2_deg).field_of_view(92_deg))
		.set_length(20.0_s);
	sim.run_for(10.0_s);
}
