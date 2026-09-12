// Star Wars X-wing aerodynamics
//
// Required extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID
// STL from: https://www.thingiverse.com/thing:353276/files

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() {
	const Length fuselage_length = 13.4_m; // T-65 X-wing; the model's longest side
	const Speed flight_speed = 100.0_mps;

	// the fuselage (Y) is as long as the domain is wide, half its length
	SimulationSetup sim(Domain::around(Model("X-Wing.stl").rotation(0_deg, 0_deg, 180_deg).length(fuselage_length))
		.size(fuselage_length, 2.0f * fuselage_length, 0.5f * fuselage_length)
		.model_offset(0_m, -0.45f * fuselage_length, 0_m)
		.vram(880_mb));

	sim.setup();
	sim.configure_units(flight_speed, Fluid::AIR, 0.075f); // the original's lattice speed
	sim.print_reynolds_number(Fluid::AIR);

	LBM lbm = sim.create_lbm(Fluid::AIR);
	sim.voxelize(lbm);

	BoundaryBuilder(lbm)
		.set_open_boundaries()
		.initialize_velocity_y(flight_speed)
		.apply();

	GraphicsConfig(lbm)
		.show_surface()
		.show_vortices()
		.apply();

#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
	VideoRecorder()
		.add("t", CameraConfig()
			.set_free_position(1.0f, -0.4f, 2.0f)
			.set_angles(-33.0f, 42.0f)
			.set_fov(68.0f))
		.add("b", CameraConfig()
			.set_free_position(0.5f, -0.35f, -0.7f)
			.set_angles(-33.0f, -40.0f)
			.set_fov(100.0f))
		.add("f", CameraConfig()
			.set_free_position(0.0f, 0.51f, 0.75f)
			.set_angles(90.0f, 28.0f)
			.set_fov(80.0f))
		.add("s", CameraConfig()
			.set_free_position(0.7f, -0.15f, 0.06f)
			.set_angles(0.0f, 0.0f)
			.set_fov(100.0f))
		.set_video_length(30.0_s)
		.record(lbm, 10.0_s);
#else
	lbm.run();
#endif
}
