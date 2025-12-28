// Aerodynamics of a cow using Setup API
//
// Demonstrates SimulationSetup for automatic geometry handling with
// transparent SDF caching for improved voxelization quality.

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() { // required extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS or GRAPHICS
	const float32_t flow_velocity_mps = 1.0f;

	SimulationSetup sim(SimulationConfig("Cow_t.stl")
		.set_vram_mb(1000u)
		.set_rotation_deg(180.0f, 0.0f, 180.0f)
		.set_clearances_m(0.24f, 0.24f, 0.0f)
		.set_reference_axis(SimulationConfig::ReferenceAxis::MAX)
		.set_fix_mesh(true));

	sim.setup();
	sim.configure_units(flow_velocity_mps, Fluid::AIR);
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
