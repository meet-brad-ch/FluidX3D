// SpaceX Starship aerodynamics
//
// Required extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID
// STL from: https://www.thingiverse.com/thing:4912729/files

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() {
	const Speed reentry_speed = 100.0_mps;

	SimulationSetup sim(SimulationConfig("StarShipV2.stl")
		.set_domain_aspect_ratio(1.0f, 2.0f, 2.0f)
		.set_vram_mb(1000u)
		.set_geometry_scale(1.6f)
		.set_center_offset_ratio(0.0f, 0.05f, -0.445f)
		.set_reference_axis(SimulationConfig::ReferenceAxis::X)
		.set_fix_mesh(true));

	sim.setup();
	sim.configure_units(reentry_speed, Fluid::AIR);
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
		.set_video_length_s(20.0f)
		.record(lbm, 10.0f, units);
#else
	lbm.run();
#endif
}
