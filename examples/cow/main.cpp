// Aerodynamics of a cow using Setup API

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() { // required extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS or GRAPHICS
	const Speed flow_velocity = 1.0_mps;
	const Length cow_length = 2.4_m;
	const Length domain_length = cow_length / 0.65f; // the cow is 65 % of the domain length

	SimulationSetup sim(Domain::around(Model("Cow_t.stl").rotation(180_deg, 0_deg, 180_deg).length(cow_length))
		.size(0.5f * domain_length, domain_length, 0.5f * domain_length)
		.gap_to_inlet(0.1f * cow_length) // the cow's nose
		.on_floor()
		.vram(1000_mb));

	sim.setup();
	sim.configure_units(flow_velocity, Fluid::AIR, LatticeMach(0.13f)); // the original's lattice speed 0.075
	sim.print_reynolds_number(Fluid::AIR);

	LBM lbm = sim.create_lbm(Fluid::AIR);
	sim.voxelize(lbm);

	BoundaryBuilder(lbm)
		.set_solid_floor()
		.set_open_boundaries()
		.initialize_velocity_y(flow_velocity)
		.apply();

	GraphicsConfig(lbm)
		.show_surface()
		.show_vortices()
		.apply();

#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
	VideoRecorder()
		.add(CameraView::orbit(-40_deg, 20_deg).field_of_view(78_deg).view_height(domain_length / 1.25f))
		.set_video_length(10.0_s)
		.record(lbm, 10.0_s);
#else
	lbm.run();
#endif
}
