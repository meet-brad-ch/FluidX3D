// Raindrop impact on sea water, using Setup API

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() { // raindrop impact; required extensions: FP16C, VOLUME_FORCE, EQUILIBRIUM_BOUNDARIES, SURFACE, INTERACTIVE_GRAPHICS or GRAPHICS
	// drop diameters and their impact (terminal) speeds; 13 is for validation
	const int select_drop_size = 12;
	//                              0      1      2      3      4      5      6      7      8      9     10     11     12     13
	const float diameters_mm[] = { 1.0f,  1.5f,  2.0f,  2.5f,  3.0f,  3.5f,  4.0f,  4.5f,  5.0f,  5.5f,  6.0f,  6.5f,  7.0f,  4.1f  };
	const float speeds_mps[] =   { 4.50f, 5.80f, 6.80f, 7.55f, 8.10f, 8.45f, 8.80f, 9.05f, 9.20f, 9.30f, 9.40f, 9.45f, 9.55f, 7.21f };
	const Length D = Length::from_si(1E-3f * diameters_mm[select_drop_size]);
	const Speed impact_speed = Speed::from_si(speeds_mps[select_drop_size]);
	const Angle inclination = 20_deg; // of the impact, 0 = vertical
	const Duration simulation_time = 0.003_s;
	const Acceleration gravity = 9.81_mps2;
	const FluidProperties sea_water = { 1024.8103_kgpm3, 1.0508E-6_m2ps, {}, {} }; // at 20 °C and 35 g/l salinity
	const SurfaceTension surface_tension = 73.81E-3_Npm;

	SimulationSetup sim(Domain::box(5.0f * D, 5.0f * D, 4.25f * D).vram(4000_mb)); // 419 x 419 x 356 cells, as the original
	sim.setup();
	sim.configure_units(impact_speed, sea_water, 0.05f);

	const float d = D.si(), u = impact_speed.si(), nu = sea_water.kinematic_viscosity.si(), rho = sea_water.density.si();
	const float sigma = surface_tension.si(), g = gravity.si();
	print_info("D = " + to_string(d, 6u));
	print_info("Re = " + to_string(units.si_Re(d, u, nu), 6u));
	print_info("We = " + to_string(units.si_We(d, u, rho, sigma), 6u));
	print_info("Fr = " + to_string(units.si_Fr(d, u, g), 6u));
	print_info("Ca = " + to_string(units.si_Ca(u, rho, nu, sigma), 6u));
	print_info("Bo = " + to_string(units.si_Bo(d, rho, g, sigma), 6u));
	print_info(to_string(to_uint(1000.0f * simulation_time.si())) + " ms = " + to_string(sim.to_lbm_timesteps(simulation_time)) + " LBM time steps");

	LBM lbm = sim.create_lbm_surface(sea_water.kinematic_viscosity, gravity, surface_tension);

	// a pool 2 diameters deep; the drop 3 cells above it, falling at the inclination toward the domain's center
	const Length depth = 2.0f * D, cell = sim.unit_scale().cell_size();
	const Length width = 5.0f * D, height = 4.25f * D;
	const float s = sinf(inclination.rad()), c = cosf(inclination.rad());
	const Position drop { 0.5f * width, 0.5f * width - D * (s / c), depth + 0.5f * D + 3.0f * cell };
	SurfaceBuilder(lbm)
		.set_water_level(depth)
		.add_water(Shape::sphere(drop, 0.5f * D), { Speed{}, s * impact_speed, -c * impact_speed })
		.set_solid_faces({ Face::Z_MIN })
		// the sides and the top, above the pool: splashes reaching them leave the domain
		.add_drain(!Shape::box({ cell, cell, 0_m }, { width - cell, width - cell, height - cell }) &
		           Shape::box({ 0_m, 0_m, depth + 0.25f * D }, { width, width, height }))
		.apply();

	GraphicsConfig(lbm)
		.show_free_surface()
		.apply();

#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
	VideoRecorder()
		.add("n", CameraView::orbit(-30_deg, 20_deg))
		.add("p", CameraView::orbit(10_deg, 40_deg))
		.add("o", CameraView::orbit(0_deg, 0_deg).field_of_view(45_deg))
		.add("t", CameraView::orbit(0_deg, 90_deg).field_of_view(45_deg))
		.set_video_length(20.0_s)
		.record(lbm, simulation_time);
#else
	lbm.run();
#endif
} /**/
