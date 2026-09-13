// Breaking waves on a beach

#include "setup/setup.hpp"

void main_setup() { // extensions: FP16S, VOLUME_FORCE, EQUILIBRIUM_BOUNDARIES, SURFACE, INTERACTIVE_GRAPHICS
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
	const Duration wave_period = 1.0f/wave_frequency;

	const Length beach_position = 1.0_m; // the beach starts 1 m from the inlet

	// Reynolds number of the original lattice setup (wave speed, water depth 48 cells, viscosity 0.01);
	// water's own viscosity would need a much finer grid
	const float reynolds = sqrt(0.001f*48.0f)*48.0f/0.01f; // about 1050
	const KinematicViscosity kinematic_viscosity = shallow_wave_speed*water_depth/reynolds;

	Simulation sim(Domain::box(domain_x, domain_y, domain_z).cell_size(domain_x / 128.0f), // 128 x 640 x 96 cells, as the original
	               Fluid::WATER.with_viscosity(kinematic_viscosity), shallow_wave_speed,
	               LatticeMach(sqrtf(3.0f*0.001f*48.0f))); // the wave speed as in the original lattice setup (0.22 cells per time step)
	sim.set_gravity(9.81_mps2);

	// the beach: a slope rising 1 in 8 from the floor at the beach position toward the far end; solid walls on all
	// sides (the wave inlet overrides Y_MIN)
	sim.surface()
		.set_water_level(water_depth)
		.initialize_hydrostatic()
		.set_solid_box()
		.add_solid(Shape::half_space({ 0.5f * domain_x, beach_position, 0.5f * sim.unit_scale().cell_size() }, float3(0.0f, -1.0f, 8.0f)))
		.apply();

	// the wave maker at the inlet (Y_MIN face), updated 14 times per wave period (every 100 time steps, as the original)
	sim.wave_maker()
		.set_wave(wave_amplitude, wave_frequency)
		.set_inlet_face(Face::Y_MIN)
		.set_vertical_factor(0.5f)
		.set_update_interval(wave_period/14.0f)
		.initialize();

	sim.graphics()
		.inherit_modes()
		.show_flags()
		.show_free_surface()
		.apply();

	sim.run();
}
