// Boeing 747 aerodynamics
//
// Extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID
// STL from: https://www.thingiverse.com/thing:2772812/files

#include "setup/setup.hpp"

void main_setup() {
	const Length fuselage_length = 70.7_m;
	const Speed cruise_speed = 250.0_mps;

	// the domain is as wide as the fuselage (Y) is long; the wingspan (X) is 64 m
	Simulation sim(Domain::around(Model("techtris_airplane.stl").angle_of_attack(-15_deg).length(fuselage_length).repair_mesh())
		.size(fuselage_length, 2.0f * fuselage_length, 0.5f * fuselage_length)
		.model_offset(0_m, -0.45f * fuselage_length, 0_m)
		.vram(880_mb),
		Fluid::AIR, cruise_speed, LatticeMach(0.13f)); // the original's lattice speed 0.075

	sim.boundaries()
		.set_open_boundaries()
		.initialize_velocity_y(cruise_speed)
		.apply();

	sim.graphics()
		.show_surface()
		.show_vortices()
		.apply();

	const Length L = fuselage_length; // camera position from the domain's origin corner
	sim.video()
		.add(CameraView::at({ 1.5f * L, 0.2f * L, 1.25f * L }, -33_deg, 42_deg).field_of_view(68_deg))
		.set_length(10.0_s);
	sim.run_for(10.0_s);
}
