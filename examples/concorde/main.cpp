// Concorde supersonic aircraft aerodynamics
//
// Extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID
// STL from: https://www.thingiverse.com/thing:1176931/files

#include "setup/setup.hpp"

void main_setup() {
	const Length fuselage_length = 62.0_m;
	const Speed cruise_speed = 300.0_kmh;

	// the fuselage (Y) is 56 % of the domain length
	const Length domain_length = fuselage_length / 0.56f;
	Simulation sim(Domain::around(Model("concord_cut_large.stl").rotation(90_deg, 0_deg, 90_deg).angle_of_attack(-10_deg).length(fuselage_length))
		.size(domain_length / 3.0f, domain_length, domain_length / 6.0f)
		.model_offset(0_m, -0.373f * fuselage_length, 0.03f * fuselage_length)
		.vram(2084_mb),
		Fluid::AIR, cruise_speed, LatticeMach(0.13f)); // the original's lattice speed 0.075

	sim.boundaries()
		.set_open_boundaries()
		.initialize_velocity_y(cruise_speed)
		.apply();

	sim.graphics()
		.show_surface()
		.show_vortices()
		.apply();

	const Length D = domain_length; // camera positions from the domain's origin corner
	sim.video()
		.add("front", CameraView::at({ 0.330448f * D, -0.382147f * D, 0.17739f * D }, -78_deg, 6_deg).field_of_view(22_deg))
		.add("back", CameraView::at({ 0.544454f * D, 1.90708f * D, 0.364068f * D }, 72_deg, 12_deg).field_of_view(20_deg))
		.add("side", CameraView::orbit(0_deg, 0_deg).field_of_view(25_deg).view_height(D / 1.648722f))
		.add("top", CameraView::orbit(0_deg, 90_deg).field_of_view(25_deg).view_height(D / 1.648722f))
		.add("wing", CameraView::at({ 0.256454f * D, 0.32028f * D, 0.134165f * D }, -56_deg, 31.6_deg))
		.add("follow", CameraView::at({ 0.2348f * D, 0.840055f * D, 0.353484f * D }, 80_deg, 35.6_deg).field_of_view(34_deg))
		.set_length(10.0_s);
	sim.run_for(10.0_s);
}
