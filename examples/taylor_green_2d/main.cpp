/// @file main.cpp
/// @brief 2D Taylor-Green vortices: 5 x 5 vortex pairs in a periodic square of water that decay by viscosity, shown
/// as the velocity field in a slice (D2Q9, interactive graphics).

#include "setup/setup.hpp"

#include <cmath>

/// @brief Sets up the vortices in a 1 m periodic square of 1024 x 1024 cells and runs until the window is closed.
///
/// The Reynolds number and the lattice speed are the original example's lattice setup (amplitude 0.2 cells per time
/// step, a wavelength of 204.8 cells, a lattice viscosity of 0.02).
void main_setup() { // extensions: D2Q9, INTERACTIVE_GRAPHICS
	using namespace si::symbols;
	const si::Length size = 1.0 * m;              // square, periodic
	const si::Length cell = size / 1024.0;        // 1024 x 1024 cells, as the original
	const si::Length wavelength = size / 5.0;     // 5 x 5 vortex pairs
	const double reynolds = 0.2 * 204.8 / 0.02;   // 2048: the original lattice setup (amplitude 0.2, wavelength 204.8 cells, viscosity 0.02)
	const si::Speed amplitude = reynolds * Fluid::WATER.kinematic_viscosity / wavelength; // 1 cm/s in water
	const si::Density density = Fluid::WATER.density;

	Domain domain = Domain::box(size, size, cell); // one cell high: 2D
	domain.cell_size(cell);
	const si::LatticeMach mach(0.346); // the original's lattice speed 0.2
	Simulation sim(domain, Fluid::WATER, amplitude, mach);

	// phases from the domain's center
	const auto phase = [=](si::Length position) {
		const si::Length from_center = position - 0.5 * size;
		const double wavelengths = au::as_raw_number(from_center / wavelength);
		const double angle = 2.0 * pi * wavelengths;
		return angle;
	};
	const auto velocity = [=](si::Position p) {
		const double x = phase(p.x);
		const double y = phase(p.y);
		const si::Speed u = amplitude * (std::cos(x) * std::sin(y));
		const si::Speed v = -amplitude * (std::sin(x) * std::cos(y));
		const si::Velocity flow{ u, v, si::Speed{} };
		return flow;
	};
	const auto pressure = [=](si::Position p) {
		const double x = phase(p.x);
		const double y = phase(p.y);
		const double shape = std::cos(2.0 * x) + std::cos(2.0 * y);
		const si::Pressure p_flow = -0.25 * density * amplitude * amplitude * shape;
		return p_flow;
	};
	sim.boundaries()
		.initialize_velocity(velocity)
		.initialize_pressure(pressure)
		.apply();

	sim.graphics()
		.show_velocity_field()
		.set_slice_mode(SliceMode::Z)
		.apply();

	sim.run();
}
