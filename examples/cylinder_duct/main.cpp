/// @file main.cpp
/// @brief A cylinder in a rectangular duct of water, driven by a pressure gradient, at Reynolds number 25000
/// (interactive graphics).

#include "setup/setup.hpp"

#include <cmath>
#include <cstdint>

/// @brief Laminar flow through a rectangular duct: the body force per mass that drives a given speed on the duct's
/// axis. It is the series solution of the core's Units::f_from_u_rectangular_duct(), in si:: quantities and double:
/// f = pi³ nu u / (4 h²) / S, S = sum over odd n of (1 - 1 / cosh(n pi w / (2 h))) sin(n pi / 2) / n³, the terms added
/// from the smallest (n = 45) to the largest (n = 1), as the core adds them. The result equals the core's float result
/// within float rounding (a relative 1e-6 or less).
class RectangularDuct {
public:
	/// @brief Makes a duct of a cross-section.
	/// @param width the duct's width w, more than 0 m
	/// @param height the duct's height h, more than 0 m
	RectangularDuct(si::Length width, si::Length height) : width_(width), height_(height) {}

	/// @brief Gives the body force per mass (the pressure gradient per density) that drives laminar flow at a speed on
	/// the duct's axis.
	/// @param viscosity the fluid's kinematic viscosity
	/// @param center_speed the flow speed on the duct's axis
	/// @return the body force per mass, along the duct
	si::Acceleration drive(si::KinematicViscosity viscosity, si::Speed center_speed) const {
		const double half_pi = 0.5 * pi;
		const double aspect = au::as_raw_number(width_ / height_);
		const double half_pi_aspect = half_pi * aspect;
		double series = 0.0;
		for(std::uint32_t term = 0u; term < odd_terms; ++term) {
			const std::uint32_t terms_left = odd_terms - term;
			const double n = 2.0 * static_cast<double>(terms_left) - 1.0; // 45, 43, ..., 1: the smallest term first
			const double n_cubed = n * n * n;
			const double wall_part = 1.0 - 1.0 / std::cosh(n * half_pi_aspect);
			const double sign = std::sin(n * half_pi);
			series += wall_part / n_cubed * sign;
		}
		const double pi_cubed = pi * pi * pi;
		const si::Acceleration amplitude = pi_cubed * viscosity * center_speed / (height_factor * height_ * height_);
		const si::Acceleration force_per_mass = amplitude / series;
		return force_per_mass;
	}

private:
	static constexpr std::uint32_t odd_terms = 23u; ///< the series' terms (n = 1 to 45), as the core: more terms make its float cosh() overflow
	static constexpr double height_factor = 4.0;    ///< the 4 in the amplitude's divisor 4 h² (a ratio)

	si::Length width_{};  ///< the duct's width w
	si::Length height_{}; ///< the duct's height h
};

/// @brief Sets up a 1 cm cylinder across a 1 x 12 x 3 cm duct of 64 x 768 x 192 cells, periodic along the duct, and
/// runs until the window is closed.
///
/// The body force is the pressure gradient of laminar flow at the center speed through a square duct as wide as the
/// cylinder (RectangularDuct: the formula of the core's Units::f_from_u_rectangular_duct(), as the original). The lattice Mach number 1 puts the center
/// speed at the lattice speed of sound, as the original.
void main_setup() { // extensions: VOLUME_FORCE, INTERACTIVE_GRAPHICS
	using namespace si::symbols;
	const si::Length diameter = 1.0 * cm; // the cylinder's, also the duct's width
	const si::Length width = diameter;
	const si::Length length = 12.0 * diameter;
	const si::Length height = 3.0 * diameter;
	const double reynolds = 25000.0;
	const si::KinematicViscosity viscosity = Fluid::WATER.kinematic_viscosity;
	const si::Speed center_speed = reynolds * viscosity / diameter; // 2.5 m/s in water
	const double cells_per_diameter = 64.0; // 64 x 768 x 192 cells, as the original
	const double start_speed_ratio = 0.1; // the start velocity along the duct, a ratio of the center speed

	Domain domain = Domain::box(width, length, height); // 64 x 768 x 192 cells
	const si::Length cell = diameter / cells_per_diameter;
	domain.cell_size(cell);
	const si::LatticeMach mach(1.0); // the lattice speed of sound, as the original
	Simulation sim(domain, Fluid::WATER, center_speed, mach);

	// the pressure gradient (per density) of laminar flow at this center speed through a square duct as wide as the
	// cylinder, as the original
	const RectangularDuct duct(width, diameter);
	const si::Acceleration drive = duct.drive(viscosity, center_speed);
	const si::AccelerationVector body_force{ si::Acceleration{}, drive, si::Acceleration{} };
	sim.set_body_force(body_force);

	const si::Position cylinder_center{ 0.5 * width, 2.0 * diameter, 0.5 * height };
	const Shape cylinder = Shape::cylinder(cylinder_center, Axis::X, 0.5 * diameter, width);
	const si::Speed start_speed = start_speed_ratio * center_speed;
	sim.boundaries()
		.set_solid_faces({ Face::X_MIN, Face::X_MAX, Face::Z_MIN, Face::Z_MAX }) // periodic along Y
		.add_solid(cylinder)
		.initialize_velocity_y(start_speed)
		.apply();

	sim.graphics()
		.show_flags()
		.show_vortices()
		.apply();

	sim.run();
}
