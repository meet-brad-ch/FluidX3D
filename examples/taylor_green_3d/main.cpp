/// @file main.cpp
/// @brief 3D Taylor-Green vortices: one vortex wavelength in a periodic cube of water that decays into turbulence,
/// shown as streamlines (interactive graphics).

#include "setup/setup.hpp"

#include <cmath>

/// @brief Sets up the classic Taylor-Green vortex in a 1 m periodic cube of 128³ cells and runs until the window is
/// closed.
///
/// The Reynolds number and the lattice speed are the original example's lattice setup (amplitude 0.25 cells per time
/// step, 128 cells, a lattice viscosity of 0.01).
void main_setup() { // extensions: INTERACTIVE_GRAPHICS
	using namespace si::symbols;
	const si::Length size = 1.0 * m;                // cube, periodic: one vortex wavelength
	const double reynolds = 0.25 * 128.0 / 0.01;    // 3200: the original lattice setup (amplitude 0.25, 128 cells, viscosity 0.01)
	const si::Speed amplitude = reynolds * Fluid::WATER.kinematic_viscosity / size; // 3.2 mm/s in water
	const si::Density density = Fluid::WATER.density;

	Domain domain = Domain::box(size, size, size);
	const double cells_per_side = 128.0; // 128³ cells, as the original
	const si::Length cell = size / cells_per_side;
	domain.cell_size(cell);
	const si::LatticeMach mach(0.433); // the original's lattice speed 0.25
	Simulation sim(domain, Fluid::WATER, amplitude, mach);

	// the classic Taylor-Green vortex (no flow along Z, its pressure), with phases from the domain's center; the
	// original's Z velocity made the start compressible and its pressure was that of the 2D vortices
	const auto phase = [=](si::Length position) {
		const si::Length from_center = position - 0.5 * size;
		const double wavelengths = au::as_raw_number(from_center / size);
		const double angle = 2.0 * pi * wavelengths;
		return angle;
	};
	const auto velocity = [=](si::Position p) {
		const double x = phase(p.x);
		const double y = phase(p.y);
		const double z = phase(p.z);
		const si::Speed u = amplitude * (std::cos(x) * std::sin(y) * std::sin(z));
		const si::Speed v = -amplitude * (std::sin(x) * std::cos(y) * std::sin(z));
		const si::Velocity flow{ u, v, si::Speed{} };
		return flow;
	};
	const auto pressure = [=](si::Position p) {
		const double x = phase(p.x);
		const double y = phase(p.y);
		const double z = phase(p.z);
		const double horizontal = std::cos(2.0 * x) + std::cos(2.0 * y);
		const double vertical = 2.0 - std::cos(2.0 * z);
		const si::Pressure p_flow = -density * amplitude * amplitude * (horizontal * vertical / 16.0);
		return p_flow;
	};
	sim.boundaries()
		.initialize_velocity(velocity)
		.initialize_pressure(pressure)
		.apply();

	sim.graphics()
		.show_streamlines()
		.apply();

	sim.run();
}
