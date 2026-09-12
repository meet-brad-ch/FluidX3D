#include "defines.hpp"
#include "info.hpp"
#include "lbm.hpp"
#include "graphics.hpp"
#include "setup.hpp"
#include "shapes.hpp"

void main_setup() { // breaking waves on beach; required extensions: FP16S, VOLUME_FORCE, EQUILIBRIUM_BOUNDARIES, SURFACE, INTERACTIVE_GRAPHICS
	// physical parameters (SI units)
	const Length domain_x = 1.0_m;
	const Length domain_y = 5.0_m;
	const Length domain_z = 0.75_m;

	const float water_depth_m = 0.5f*domain_z.si(); // initial water level at 50% height
	const float shallow_wave_speed_mps = sqrt(9.81f*water_depth_m); // shallow-water wave speed sqrt(g*h), the fastest velocity in the flow

	// wave maker as in the original lattice setup (peak velocity 0.12, frequency 0.0007 per step, water depth 48 cells,
	// gravity 0.001), scaled to this water depth with the same Froude number
	const float wave_velocity_mps = 0.12f/sqrt(0.001f*48.0f)*shallow_wave_speed_mps; // about 1.05 m/s
	const float wave_frequency_hz = 0.0007f*sqrt(48.0f/0.001f)*sqrt(9.81f/water_depth_m); // about 0.78 Hz
	const float wave_amplitude_m = wave_velocity_mps/(2.0f*pif*wave_frequency_hz); // about 0.21 m

	const float beach_position_m = 1.0f;  // beach starts at 1m from inlet

	// simulation setup
	SimulationSetup sim(Domain::box(domain_x, domain_y, domain_z).vram(2000_mb));

	sim.setup();
	sim.configure_units(shallow_wave_speed_mps, Fluid::WATER, 0.22f); // wave speed in LBM units as in the original lattice setup: sqrt(0.001*48)

	// create LBM for free surface simulation
	// Reynolds number of the original lattice setup (wave speed, water depth 48 cells, viscosity 0.01);
	// water's own viscosity would need a much finer grid
	const float reynolds = sqrt(0.001f*48.0f)*48.0f/0.01f; // about 1050
	const float kinematic_viscosity_m2ps = shallow_wave_speed_mps*water_depth_m/reynolds;
	LBM lbm = sim.create_lbm_surface(kinematic_viscosity_m2ps, 9.81f);

	// get domain dimensions for beach geometry
	const uint Nx = lbm.get_Nx();
	const float beach_y = sim.to_lbm_length(beach_position_m);

	// configure free surface with beach geometry
	SurfaceBuilder(lbm)
		// initial water level at 50% height
		.set_water_level(0.5f)
		.set_gravity_lbm(sim.to_lbm_acceleration(9.81f))
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
