#include "defines.hpp"
#include "info.hpp"
#include "lbm.hpp"
#include "graphics.hpp"
#include "setup.hpp"
#include "shapes.hpp"

void main_setup() { // breaking waves on beach; required extensions: FP16S, VOLUME_FORCE, EQUILIBRIUM_BOUNDARIES, SURFACE, INTERACTIVE_GRAPHICS
	// physical parameters (SI units)
	const float domain_x_m = 1.0f;
	const float domain_y_m = 5.0f;
	const float domain_z_m = 0.75f;

	const float wave_amplitude_m = 0.05f;
	const float wave_frequency_hz = 0.5f;
	const float wave_velocity_mps = wave_amplitude_m * 2.0f * pif * wave_frequency_hz;

	const float beach_position_m = 1.0f;  // beach starts at 1m from inlet

	// simulation setup
	SimulationSetup sim(SimulationConfig()
		.set_domain_size_m(domain_x_m, domain_y_m, domain_z_m)
		.set_vram_mb(2000u));

	sim.setup();
	sim.configure_units(wave_velocity_mps, Fluid::WATER);

	// create LBM for free surface simulation
	LBM lbm = sim.create_lbm_surface(Fluid::WATER, 9.81f);

	// get domain dimensions for beach geometry
	const uint Nx = lbm.get_Nx();
	const float beach_y = sim.to_lbm_length(beach_position_m);

	// configure free surface with beach geometry
	SurfaceBuilder(lbm)
		// initial water level at 50% height
		.set_water_level(0.5f)
		.set_gravity_lbm(sim.to_lbm_force_per_volume(9.81f))
		.initialize_hydrostatic()
		// solid walls on all sides (wave inlet will override Y_MIN)
		.set_solid_walls()
		// sloped beach geometry (using fluid region predicate to exclude)
		.set_fluid_region([=](uint x, uint y, uint z) {
			// Exclude cells that are part of the sloped beach
			return !plane(x, y, z, float3((float)Nx / 2.0f, beach_y, 0.0f), float3(0.0f, -1.0f, 8.0f));
		})
		.apply();

	// add sloped beach as solid
	parallel_for(lbm.get_N(), [&](ulong n) {
		uint x = 0u, y = 0u, z = 0u;
		lbm.coordinates(n, x, y, z);
		if (plane(x, y, z, float3(lbm.center().x, beach_y, 0.0f), float3(0.0f, -1.0f, 8.0f))) {
			lbm.flags[n] = TYPE_S;
		}
	});

	// configure wave boundary at inlet (Y_MIN face)
	WaveBoundary wave(lbm);
	wave
		.set_wave_parameters_si(wave_amplitude_m, wave_frequency_hz)
		.set_inlet_face(Face::Y_MIN)
		.set_vertical_factor(0.5f)
		.initialize();

	// configure visualization
	GraphicsConfig(lbm)
		.inherit_modes()
		.show_flags()
		.show_free_surface()
		.apply();

	// wave generation loop (time-varying boundary condition)
	lbm.run(0u);
	while (true) {
		wave.update(lbm.get_t());
		lbm.run(100u);
	}
} /**/
