// Boeing 747 aerodynamics
//
// Required extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID
// STL from: https://www.thingiverse.com/thing:2772812/files

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() {
	const Length fuselage_length = 70.7_m;
	const Speed cruise_speed = 250.0_mps;

	// the domain is as wide as the fuselage (Y) is long; the wingspan (X) is 64 m
	SimulationSetup sim(Domain::around(Model("techtris_airplane.stl").angle_of_attack(-15_deg).length(fuselage_length).repair_mesh())
		.size(fuselage_length, 2.0f * fuselage_length, 0.5f * fuselage_length)
		.model_offset(0_m, -0.45f * fuselage_length, 0_m)
		.vram(880_mb));

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
	const Length L = fuselage_length; // camera position from the domain's origin corner
	VideoRecorder()
		.add(CameraView::at({ 1.5f * L, 0.2f * L, 1.25f * L }, -33_deg, 42_deg).field_of_view(68_deg))
		.set_video_length(10.0_s)
		.record(lbm, 10.0_s);
#else
	lbm.run();
#endif
}
