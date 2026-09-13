// Space Shuttle aerodynamics on 8 GPUs
//
// Extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID
// STL from: https://www.thingiverse.com/thing:4975964/files

#include "setup/setup.hpp"

void main_setup() {
	const Length orbiter_length = 37.24_m; // NASA: 37.24 m long, 23.79 m wingspan
	const Speed flight_speed = 100.0_mps;

	// the orbiter (Y, before its angle of attack) is 30 % of the domain length; the grid is split among 2 x 4 x 1 GPUs
	const Length domain_length = orbiter_length / 0.3f;
	Simulation sim(Domain::around(Model("Full_Shuttle.stl").rotation(0_deg, 0_deg, 270_deg).angle_of_attack(-20_deg).length(orbiter_length))
		.size(0.25f * domain_length, domain_length, 0.2f * domain_length)
		.model_offset(0_m, -1.1f * orbiter_length, 0.05f * orbiter_length)
		.vram(1000_mb)
		.gpus(2u, 4u, 1u),
		Fluid::AIR, flight_speed, LatticeMach(0.13f)); // the original's lattice speed 0.075

	sim.boundaries()
		.set_open_boundaries()
		.initialize_velocity_y(flight_speed)
		.apply();

	sim.graphics()
		.show_flags()
		.show_surface()
		.show_vortices()
		.apply();

	const Length D = domain_length; // camera positions from the domain's origin corner
	sim.video()
		.add("top", CameraView::at({ -0.23399f * D, 0.864331f * D, 0.368885f * D }, -205_deg, 36_deg).field_of_view(74_deg))
		.add("bottom", CameraView::at({ -0.130302f * D, -0.018006f * D, 0.1f * D }, -137_deg, 0_deg).field_of_view(74_deg))
		.set_length(30.0_s);
	sim.run_for(10.0_s);
}
