// Concorde supersonic aircraft aerodynamics
//
// Required extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID
// STL from: https://www.thingiverse.com/thing:1176931/files

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() {
	const float32_t fuselage_length_m = 62.0f;
	const float32_t cruise_speed_mps = 300.0f / 3.6f;

	SimulationSetup sim(SimulationConfig("concord_cut_large.stl")
		.set_domain_aspect_ratio(1.0f, 3.0f, 0.5f)
		.set_vram_mb(6084u)
		.set_geometry_scale(0.56f)
		.set_rotation_deg(90.0f, 0.0f, 90.0f)
		.set_angle_of_attack_deg(-10.0f)
		.set_center_offset_ratio(0.0f, -0.373f, 0.03f)
		.set_reference_axis(SimulationConfig::ReferenceAxis::Y));

	sim.setup();
	sim.configure_units_with_length(fuselage_length_m, cruise_speed_mps, Fluid::AIR);
	sim.print_reynolds_number(Fluid::AIR);

	LBM lbm = sim.create_lbm(Fluid::AIR);
	sim.voxelize(lbm);

	BoundaryBuilder(lbm)
		.set_open_boundaries()
		.initialize_velocity_y(cruise_speed_mps)
		.apply();

	GraphicsConfig(lbm)
		.show_surface()
		.show_vortices()
		.apply();

#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
	VideoRecorder()
		.add("front", CameraConfig()
			.set_free_position(0.491343f, -0.882147f, 0.564339f)
			.set_angles(-78.0f, 6.0f)
			.set_fov(22.0f))
		.add("back", CameraConfig()
			.set_free_position(1.133361f, 1.407077f, 1.684411f)
			.set_angles(72.0f, 12.0f)
			.set_fov(20.0f))
		.add("side", CameraConfig()
			.set_angles(0.0f, 0.0f)
			.set_fov(25.0f)
			.set_zoom(1.648722f))
		.add("top", CameraConfig()
			.set_angles(0.0f, 90.0f)
			.set_fov(25.0f)
			.set_zoom(1.648722f))
		.add("wing", CameraConfig()
			.set_free_position(0.269361f, -0.179720f, 0.304988f)
			.set_angles(-56.0f, 31.6f)
			.set_fov(100.0f))
		.add("follow", CameraConfig()
			.set_free_position(0.204399f, 0.340055f, 1.620902f)
			.set_angles(80.0f, 35.6f)
			.set_fov(34.0f))
		.set_video_length_s(10.0f)
		.record(lbm, 10.0f, units);
#else
	lbm.run();
#endif
}
