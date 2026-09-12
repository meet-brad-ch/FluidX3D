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

	const Length water_depth = 0.5f*domain_z; // initial water level at 50% height
	const Speed shallow_wave_speed = sqrt(9.81_mps2*water_depth); // shallow-water wave speed sqrt(g*h), the fastest velocity in the flow

	// wave maker as in the original lattice setup (peak velocity 0.12, frequency 0.0007 per step, water depth 48 cells,
	// gravity 0.001), scaled to this water depth with the same Froude number
	const Speed wave_velocity = 0.12f/sqrt(0.001f*48.0f)*shallow_wave_speed; // about 1.05 m/s
	const Frequency wave_frequency = 0.0007f*sqrt(48.0f/0.001f)*sqrt(9.81_mps2/water_depth); // about 0.78 Hz
	const Length wave_amplitude = wave_velocity/(2.0f*pif*wave_frequency); // about 0.21 m

	const Length beach_position = 1.0_m; // the beach starts 1 m from the inlet

	// simulation setup
	SimulationSetup sim(Domain::box(domain_x, domain_y, domain_z).cell_size(domain_x / 128.0f)); // 128 x 640 x 96 cells, as the original

	sim.setup();
	sim.configure_units(shallow_wave_speed, Fluid::WATER, sqrt(0.001f*48.0f)); // wave speed in LBM units as in the original lattice setup (0.22)

	// create LBM for free surface simulation
	// Reynolds number of the original lattice setup (wave speed, water depth 48 cells, viscosity 0.01);
	// water's own viscosity would need a much finer grid
	const float reynolds = sqrt(0.001f*48.0f)*48.0f/0.01f; // about 1050
	const KinematicViscosity kinematic_viscosity = shallow_wave_speed*water_depth/reynolds;
	LBM lbm = sim.create_lbm_surface(kinematic_viscosity, 9.81_mps2);

	// get domain dimensions for beach geometry
	const float beach_y = sim.to_lbm_length(beach_position);

	// configure free surface with beach geometry
	SurfaceBuilder(lbm)
		.set_water_level(water_depth)
		.initialize_hydrostatic()
		// solid walls on all sides (wave inlet will override Y_MIN)
		.set_solid_walls()
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
		.set_wave(wave_amplitude, wave_frequency)
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
