// City Wind Simulation
//
// Required extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID
// STL from: resources/city.stl
//
// Urban wind flow simulation with atmospheric boundary layer profile.
// Demonstrates power-law wind profile for realistic urban aerodynamics.

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() {
	const Length city_size = 1000.0_m;           // city block size
	const float32_t wind_speed_mps = 10.0f;      // Wind speed at reference height
	const float32_t reference_height_m = 100.0f; // Reference height for wind profile
	const float32_t simulation_time_s = 60.0f;   // 1 minute of simulation

	// the city (its size along X) is 1.7 domain widths: the domain's side walls cut through it
	const Length domain_width = city_size / 1.7f;
	SimulationSetup sim(Domain::around(Model("city.stl").rotation(0_deg, 0_deg, 90_deg).length(city_size, Axis::X))
		.size(domain_width, 2.0f * domain_width, 0.5f * domain_width)
		.model_offset(0_m, -0.05f * city_size, -0.025f * city_size)
		.vram(2152_mb));

	sim.setup();
	sim.configure_units(wind_speed_mps, Fluid::AIR);
	sim.print_reynolds_number(Fluid::AIR);

	// Create LBM
	LBM lbm = sim.create_lbm(Fluid::AIR);

	// Voxelize city geometry
	sim.voxelize(lbm);

	// Configure boundaries with atmospheric boundary layer wind profile
	// Power-law profile: U(z) = U_ref * (z / z_ref)^alpha
	// alpha = 0.25 for urban/suburban terrain
	BoundaryBuilder(lbm)
		.set_solid_floor()
		.set_open_boundaries()
		.set_wind_profile_power_law(wind_speed_mps, reference_height_m, 0.25f)
		.set_wind_direction(Face::Y_MIN)
		.apply();

	// Configure graphics
	GraphicsConfig(lbm)
		.show_surface()
		.show_vortices()
		.apply();

	// Run simulation
	const uint64_t total_timesteps = sim.to_lbm_timesteps(simulation_time_s);
	print_info(to_string(simulation_time_s, 0u) + " seconds = " + to_string(total_timesteps) + " time steps");

#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
	VideoRecorder()
		.add("a", CameraConfig()
			.set_free_position(-1.088245f, -0.443919f, 1.717979f)
			.set_angles(215.0f, 39.0f)
			.set_fov(70.0f))
		.add("b", CameraConfig()
			.set_free_position(0.203233f, 0.036325f, 0.435000f)
			.set_angles(56.0f, 45.0f)
			.set_fov(105.0f))
		.add("c", CameraConfig()
			.set_free_position(-0.283501f, -0.099679f, 0.175468f)
			.set_angles(234.0f, 29.0f)
			.set_fov(117.0f))
		.set_video_length_s(30.0f)
		.record(lbm, simulation_time_s, units);
#else
	lbm.run();
#endif
}
