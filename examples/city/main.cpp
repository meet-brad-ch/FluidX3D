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
	const Speed wind_speed = 10.0_mps;           // Wind speed at reference height
	const Length reference_height = 100.0_m;     // Reference height for wind profile
	const Duration simulation_time = 1.0_min;

	// the city (its size along Y) is 85 % of the domain length
	const Length domain_width = city_size / 1.7f;
	SimulationSetup sim(Domain::around(Model("city.stl").rotation(0_deg, 0_deg, 90_deg).length(city_size))
		.size(domain_width, 2.0f * domain_width, 0.5f * domain_width)
		.model_offset(0_m, -0.05f * city_size, -0.025f * city_size)
		.cell_size(domain_width / 512.0f)); // 512 x 1024 x 256 cells, as the original

	sim.setup();
	sim.configure_units(wind_speed, Fluid::AIR, 0.07f);
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
		.set_wind_profile_power_law(wind_speed, reference_height, 0.25f)
		.set_wind_direction(Face::Y_MIN)
		.apply();

	// Configure graphics
	GraphicsConfig(lbm)
		.show_surface()
		.show_vortices()
		.apply();

	// Run simulation
	const uint64_t total_timesteps = sim.to_lbm_timesteps(simulation_time);
	print_info(to_string(simulation_time.si(), 0u) + " seconds = " + to_string(total_timesteps) + " time steps");

#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
	const Length W = domain_width; // camera positions from the domain's origin corner
	VideoRecorder()
		.add("a", CameraView::at({ -0.588245f * W, 0.112162f * W, 1.10899f * W }, 215_deg, 39_deg).field_of_view(70_deg))
		.add("b", CameraView::at({ 0.703233f * W, 1.07265f * W, 0.4675f * W }, 56_deg, 45_deg).field_of_view(105_deg))
		.add("c", CameraView::at({ 0.216499f * W, 0.800642f * W, 0.337734f * W }, 234_deg, 29_deg).field_of_view(117_deg))
		.set_video_length(30.0_s)
		.record(lbm, simulation_time);
#else
	lbm.run();
#endif
}
