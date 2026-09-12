// SpaceX Starship aerodynamics
//
// Required extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID
// STL from: https://www.thingiverse.com/thing:4912729/files

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() {
	const Length ship_length = 50.0_m; // Starship upper stage; the model's longest side
	const Speed reentry_speed = 100.0_mps;

	// the ship (Y) is 80 % of the domain length; the air flows up (+z) past it, as in a belly-first reentry
	const Length domain_length = ship_length / 0.8f;
	SimulationSetup sim(Domain::around(Model("StarShipV2.stl").length(ship_length).repair_mesh())
		.size(0.5f * domain_length, domain_length, domain_length)
		.model_offset(0_m, 0.05f * ship_length, -0.445f * ship_length)
		.vram(1000_mb));

	sim.setup();
	sim.configure_units(reentry_speed, Fluid::AIR, 0.05f); // the original's lattice speed
	sim.print_reynolds_number(Fluid::AIR);

	LBM lbm = sim.create_lbm(Fluid::AIR);
	sim.voxelize(lbm);

	BoundaryBuilder(lbm)
		.set_open_boundaries()
		.initialize_velocity_z(reentry_speed)
		.apply();

	GraphicsConfig(lbm)
		.show_lattice()
		.show_surface()
		.show_vortices()
		.apply();

#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
	VideoRecorder()
		.add("top", CameraConfig()
			.set_free_position(2.116744f, -0.775261f, 1.026577f)
			.set_angles(-38.0f, 37.0f)
			.set_fov(60.0f))
		.add("bottom", CameraConfig()
			.set_free_position(0.718942f, 0.311263f, -0.498366f)
			.set_angles(32.0f, -40.0f)
			.set_fov(104.0f))
		.add("side", CameraConfig()
			.set_free_position(1.748119f, 0.442782f, 0.087945f)
			.set_angles(24.0f, 2.0f)
			.set_fov(92.0f))
		.set_video_length(20.0_s)
		.record(lbm, 10.0_s);
#else
	lbm.run();
#endif
}
