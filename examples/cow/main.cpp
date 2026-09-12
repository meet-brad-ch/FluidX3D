// Aerodynamics of a cow using Setup API

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() { // required extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS or GRAPHICS
	const float32_t flow_velocity_mps = 1.0f;
	const Length cow_length = 2.4_m;
	const Length domain_length = cow_length / 0.65f; // the cow is 65 % of the domain length

	SimulationSetup sim(Domain::around(Model("Cow_t.stl").rotation(180_deg, 0_deg, 180_deg).length(cow_length))
		.size(0.5f * domain_length, domain_length, 0.5f * domain_length)
		.gap_to_inlet(0.1f * cow_length)   // the cow's nose
		.gap_to_floor(0.006f * cow_length) // about one cell
		.vram(1000_mb));

	sim.setup();
	sim.configure_units(flow_velocity_mps, Fluid::AIR, 0.075f);
	sim.print_reynolds_number(Fluid::AIR);

	LBM lbm = sim.create_lbm(Fluid::AIR);
	sim.voxelize(lbm);

	BoundaryBuilder(lbm)
		.set_solid_floor()
		.set_open_boundaries()
		.initialize_velocity_y(flow_velocity_mps)
		.apply();

	GraphicsConfig(lbm)
		.show_surface()
		.show_vortices()
		.apply();

#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
	VideoRecorder()
		.add(CameraConfig().set_angles(-40.0f, 20.0f).set_fov(78.0f).set_zoom(1.25f))
		.set_video_length_s(10.0f)
		.record(lbm, 10.0f, units);
#else
	lbm.run();
#endif
}
