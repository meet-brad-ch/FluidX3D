// Concorde supersonic aircraft aerodynamics
//
// Required extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID
// STL from: https://www.thingiverse.com/thing:1176931/files

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() {
	const Length fuselage_length = 62.0_m;
	const Speed cruise_speed = 300.0_kmh;

	// the fuselage (Y) is 56 % of the domain length
	const Length domain_length = fuselage_length / 0.56f;
	SimulationSetup sim(Domain::around(Model("concord_cut_large.stl").rotation(90_deg, 0_deg, 90_deg).angle_of_attack(-10_deg).length(fuselage_length))
		.size(domain_length / 3.0f, domain_length, domain_length / 6.0f)
		.model_offset(0_m, -0.373f * fuselage_length, 0.03f * fuselage_length)
		.vram(2084_mb));

	sim.setup();
	sim.configure_units(cruise_speed, Fluid::AIR, 0.075f);
	sim.print_reynolds_number(Fluid::AIR);

	LBM lbm = sim.create_lbm(Fluid::AIR);
	sim.voxelize(lbm);

	BoundaryBuilder(lbm)
		.set_open_boundaries()
		.initialize_velocity_y(cruise_speed)
		.apply();

	GraphicsConfig(lbm)
		.show_surface()
		.show_vortices()
		.apply();

#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
	const Length D = domain_length; // camera positions from the domain's origin corner
	VideoRecorder()
		.add("front", CameraView::at({ 0.330448f * D, -0.382147f * D, 0.17739f * D }, -78_deg, 6_deg).field_of_view(22_deg))
		.add("back", CameraView::at({ 0.544454f * D, 1.90708f * D, 0.364068f * D }, 72_deg, 12_deg).field_of_view(20_deg))
		.add("side", CameraView::orbit(0_deg, 0_deg).field_of_view(25_deg).view_height(D / 1.648722f))
		.add("top", CameraView::orbit(0_deg, 90_deg).field_of_view(25_deg).view_height(D / 1.648722f))
		.add("wing", CameraView::at({ 0.256454f * D, 0.32028f * D, 0.134165f * D }, -56_deg, 31.6_deg))
		.add("follow", CameraView::at({ 0.2348f * D, 0.840055f * D, 0.353484f * D }, 80_deg, 35.6_deg).field_of_view(34_deg))
		.set_video_length(10.0_s)
		.record(lbm, 10.0_s);
#else
	lbm.run();
#endif
}
