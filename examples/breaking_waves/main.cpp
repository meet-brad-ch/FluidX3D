/// @file main.cpp
/// @brief Breaking waves on a beach: a wave maker at the inlet sends waves up a 1-in-8 slope (free surface,
/// interactive graphics).

#include "setup/setup.hpp"

#include <cmath>

/// @brief Sets up a 1 x 5 x 0.75 m wave tank of 128 x 640 x 96 cells, half full, with the beach 1 m from the inlet,
/// and runs until the window is closed.
///
/// The wave maker, the Reynolds number and the lattice speed are the original example's lattice setup (peak velocity
/// 0.12, frequency 0.0007 per step, a water depth of 48 cells, gravity 0.001, a lattice viscosity of 0.01), scaled
/// to this water depth with the same Froude number; water's own viscosity would need a much finer grid.
void main_setup() { // extensions: FP16S, VOLUME_FORCE, EQUILIBRIUM_BOUNDARIES, SURFACE, INTERACTIVE_GRAPHICS
	using namespace si::symbols;
	const si::Length domain_x = 1.0 * m;
	const si::Length domain_y = 5.0 * m;
	const si::Length domain_z = 0.75 * m;
	const si::Acceleration gravity = 9.81 * m / (s * s);

	const si::Length water_depth = 0.5 * domain_z; // initial water level at 50% height
	const si::Speed shallow_wave_speed = au::sqrt(gravity * water_depth); // shallow-water wave speed sqrt(g*h), the fastest velocity in the flow

	// the wave maker of the original lattice setup, scaled with the same Froude number
	const si::Speed wave_velocity = 0.12 / std::sqrt(0.001 * 48.0) * shallow_wave_speed;              // about 1.05 m/s
	const si::Frequency wave_frequency = 0.0007 * std::sqrt(48.0 / 0.001) * au::sqrt(gravity / water_depth); // about 0.78 Hz
	const si::Length wave_amplitude = wave_velocity / (2.0 * pi * wave_frequency);                     // about 0.21 m
	const si::Duration wave_period = 1.0 / wave_frequency;

	const si::Length beach_position = 1.0 * m; // the beach starts 1 m from the inlet

	// Reynolds number of the original lattice setup (wave speed, water depth 48 cells, viscosity 0.01)
	const double reynolds = std::sqrt(0.001 * 48.0) * 48.0 / 0.01; // about 1050
	const si::KinematicViscosity kinematic_viscosity = shallow_wave_speed * water_depth / reynolds;

	const double cells_x = 128.0; // 128 x 640 x 96 cells, as the original
	const si::Length cell_size = domain_x / cells_x;
	Domain domain = Domain::box(domain_x, domain_y, domain_z);
	domain.cell_size(cell_size);
	const FluidProperties fluid = Fluid::WATER.with_viscosity(kinematic_viscosity);
	const si::LatticeMach mach(std::sqrt(3.0 * 0.001 * 48.0)); // the wave speed as in the original lattice setup (0.22 cells per time step)
	Simulation sim(domain, fluid, shallow_wave_speed, mach);
	sim.set_gravity(gravity);

	// the beach: a slope rising 1 in 8 from the floor at the beach position toward the far end; solid walls on all
	// sides (the wave inlet overrides Y_MIN)
	const si::Length cell = sim.converter().cell();
	const si::Position beach_start{ 0.5 * domain_x, beach_position, 0.5 * cell };
	const si::Direction beach_normal{ 0.0, -1.0, 8.0 };
	sim.surface()
		.set_water_level(water_depth)
		.initialize_hydrostatic()
		.set_solid_box()
		.add_solid(Shape::half_space(beach_start, beach_normal))
		.apply();

	// the wave maker at the inlet (Y_MIN face), updated 14 times per wave period (every 100 time steps, as the original)
	const si::Duration wave_update_interval = wave_period / 14.0;
	sim.wave_maker()
		.set_wave(wave_amplitude, wave_frequency)
		.set_inlet_face(Face::Y_MIN)
		.set_vertical_factor(0.5f)
		.set_update_interval(wave_update_interval)
		.initialize();

	sim.graphics()
		.inherit_modes()
		.show_flags()
		.show_free_surface()
		.apply();

	sim.run();
}
