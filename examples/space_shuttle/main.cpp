// Space Shuttle aerodynamics
//
// Required extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID
// STL from: https://www.thingiverse.com/thing:4975964/files

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() {
	const float32_t flight_speed_mps = 100.0f;

	SimulationSetup sim(SimulationConfig("Full_Shuttle.stl")
		.set_domain_aspect_ratio(1.0f, 4.0f, 0.8f)
		.set_vram_mb(1000u)
		.set_geometry_scale(1.25f)
		.set_rotation_deg(0.0f, 0.0f, 270.0f)
		.set_angle_of_attack_deg(-20.0f)
		.set_center_offset_ratio(0.0f, 0.05f, 0.05f)
		.set_reference_axis(SimulationConfig::ReferenceAxis::X));

	sim.setup();
	sim.configure_units(flight_speed_mps, Fluid::AIR);
	sim.print_reynolds_number(Fluid::AIR);

	const auto& r = sim.get_results();
	// Multi-GPU: 2x4x1 = 8 GPUs
	LBM lbm(r.Nx, r.Ny, r.Nz, 2u, 4u, 1u, sim.to_lbm_viscosity(Fluid::AIR.kinematic_viscosity));
	sim.voxelize(lbm);

	BoundaryBuilder(lbm)
		.set_open_boundaries()
		.initialize_velocity_y(flight_speed_mps)
		.apply();

	GraphicsConfig(lbm)
		.show_lattice()
		.show_surface()
		.show_vortices()
		.apply();

#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
	VideoRecorder()
		.add("top", CameraConfig()
			.set_free_position(-1.435962f, 0.364331f, 1.344426f)
			.set_angles(-205.0f, 36.0f)
			.set_fov(74.0f))
		.add("bottom", CameraConfig()
			.set_free_position(-1.021207f, -0.518006f, 0.0f)
			.set_angles(-137.0f, 0.0f)
			.set_fov(74.0f))
		.set_fps(30.0f)
		.record(lbm, 10.0f, units);
#else
	lbm.run();
#endif
}
