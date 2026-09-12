// Boeing 747 aerodynamics
//
// Required extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID
// STL from: https://www.thingiverse.com/thing:2772812/files

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() {
	const float32_t fuselage_length_m = 70.7f;
	const float32_t cruise_speed_mps = 250.0f;

	SimulationSetup sim(SimulationConfig("techtris_airplane.stl")
		.set_domain_aspect_ratio(1.0f, 2.0f, 0.5f)
		.set_vram_mb(880u)
		.set_geometry_scale(1.0f)
		.set_angle_of_attack_deg(-15.0f)
		.set_center_offset_ratio(0.0f, -0.45f, 0.0f)
		.set_reference_axis(SimulationConfig::ReferenceAxis::X)
		.set_fix_mesh(true));

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
		.add(CameraConfig()
			.set_free_position(1.0f, -0.4f, 2.0f)
			.set_angles(-33.0f, 42.0f)
			.set_fov(68.0f))
		.set_video_length_s(10.0f)
		.record(lbm, 10.0f, units);
#else
	lbm.run();
#endif
}
