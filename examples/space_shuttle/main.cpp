// Space Shuttle aerodynamics
//
// Required extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID
// STL from: https://www.thingiverse.com/thing:4975964/files

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() {
	const Length orbiter_length = 37.24_m; // NASA: 37.24 m long, 23.79 m wingspan
	const Speed flight_speed = 100.0_mps;

	// the orbiter (Y, before its angle of attack) is 30 % of the domain length
	const Length domain_length = orbiter_length / 0.3f;
	SimulationSetup sim(Domain::around(Model("Full_Shuttle.stl").rotation(0_deg, 0_deg, 270_deg).angle_of_attack(-20_deg).length(orbiter_length))
		.size(0.25f * domain_length, domain_length, 0.2f * domain_length)
		.model_offset(0_m, -1.1f * orbiter_length, 0.05f * orbiter_length)
		.vram(1000_mb));

	sim.setup();
	sim.configure_units(flight_speed, Fluid::AIR, 0.075f); // the original's lattice speed
	sim.print_reynolds_number(Fluid::AIR);

	const auto& r = sim.get_results();
	// Multi-GPU: 2x4x1 = 8 GPUs
	LBM lbm(r.Nx, r.Ny, r.Nz, 2u, 4u, 1u, sim.to_lbm_viscosity(Fluid::AIR.kinematic_viscosity));
	sim.voxelize(lbm);

	BoundaryBuilder(lbm)
		.set_open_boundaries()
		.initialize_velocity_y(flight_speed)
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
		.set_video_length(30.0_s)
		.record(lbm, 10.0_s);
#else
	lbm.run();
#endif
}
