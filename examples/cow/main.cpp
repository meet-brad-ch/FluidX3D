// Aerodynamics of a cow using Setup API

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() { // required extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS or GRAPHICS
	const float32_t flow_velocity_mps = 1.0f;
	const float32_t cow_size_m = 2.4f;

	SimulationSetup sim(SimulationConfig("Cow_t.stl")
		.set_domain_aspect_ratio(1.0f, 2.0f, 1.0f)
		.set_vram_mb(1000u)
		.set_geometry_scale(0.65f)
		.set_rotation_deg(180.0f, 0.0f, 180.0f)
		.set_reference_axis(SimulationConfig::ReferenceAxis::Y)
		.set_pmin_offset_ratio(0.0f, 0.1f, 0.006f));

	sim.setup();
	sim.configure_units_with_length(cow_size_m, flow_velocity_mps, Fluid::AIR, 0.075f);
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
		.set_fps(10.0f)
		.record(lbm, 10.0f, units);
#else
	lbm.run();
#endif
}
