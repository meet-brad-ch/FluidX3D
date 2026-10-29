// 2D Taylor-Green vortices in water: u = A*sin(kx)*cos(ky), v = -A*cos(kx)*sin(ky) is an exact solution of the
// Navier-Stokes equations whose kinetic energy decays as exp(-4*nu*k²*t).
// Passes when the viscosity measured from the decay is within 1 % of water's.

#include "setup/setup.hpp"
#include "physics_check.hpp"

#include <cmath>

namespace {

/// @brief The start field of the vortices: u = A sin(kx) cos(ky), v = -A cos(kx) sin(ky) and its pressure.
class VortexField {
public:
	/// @brief Makes the field of one wavelength over a periodic square.
	/// @param size the square's side, one wavelength
	/// @param amplitude the velocity amplitude A
	/// @param density the fluid's density, for the pressure
	VortexField(si::Length size, si::Speed amplitude, si::Density density) : size_(size), amplitude_(amplitude), density_(density) {}

	/// @brief Gives the start velocity at a point.
	/// @param p the point, from the domain's origin corner
	/// @return the velocity (no z component)
	si::Velocity velocity(si::Position p) const {
		const double x{ phase(p.x) };
		const double y{ phase(p.y) };
		const si::Speed u = amplitude_ * (std::sin(x) * std::cos(y));
		const si::Speed v = -amplitude_ * (std::cos(x) * std::sin(y));
		const si::Velocity flow{ u, v, si::Speed{} };
		return flow;
	}

	/// @brief Gives the start pressure at a point: rho A^2 / 4 (cos 2kx + cos 2ky).
	/// @param p the point, from the domain's origin corner
	/// @return the pressure
	si::Pressure pressure(si::Position p) const {
		const double double_x{ 2.0 * phase(p.x) };
		const double double_y{ 2.0 * phase(p.y) };
		const double shape{ std::cos(double_x) + std::cos(double_y) };
		const si::Pressure p_flow = quarter * density_ * amplitude_ * amplitude_ * shape;
		return p_flow;
	}

private:
	/// @brief Gives the phase of a position along one axis.
	/// @param position the position along the axis
	/// @return 2 pi times the position in wavelengths, in rad
	double phase(si::Length position) const {
		const double wavelengths{ au::as_raw_number(position / size_) };
		const double angle{ 2.0 * pi * wavelengths };
		return angle;
	}

	static constexpr double quarter = 0.25; ///< the factor 1/4 of the pressure

	si::Length size_{};     ///< the wavelength
	si::Speed amplitude_{}; ///< the velocity amplitude
	si::Density density_{}; ///< the fluid's density
};

/// @brief Measures the viscosity from the decay of the kinetic energy (exp(-4 nu k^2 t)) and reports it.
class DecayCheck {
public:
	/// @brief Makes the check.
	/// @param sim the simulation of the vortices (not owned, outlives the check)
	/// @param size the wavelength
	/// @param viscosity the fluid's kinematic viscosity, the expected value
	DecayCheck(Simulation& sim, si::Length size, si::KinematicViscosity viscosity) : sim_(sim), size_(size), viscosity_(viscosity) {}

	/// @brief At the start, keeps the energy; at the next call, reports the measured viscosity and stops the run.
	/// @param t the simulated time
	void at(si::Duration t) {
		if(t == si::Duration{}) {
			initial_energy_ = kinetic_energy();
			return;
		}
		const double energy_ratio{ kinetic_energy() / initial_energy_ };
		const auto k = 2.0 * pi / size_; // the wave number, 1/m
		const si::KinematicViscosity measured_viscosity = -std::log(energy_ratio) / (4.0 * k * k * t);
		const double viscosity_ratio{ au::as_raw_number(measured_viscosity / viscosity_) };
		const double error{ std::fabs(viscosity_ratio - 1.0) };
		PhysicsCheck::report("Taylor-Green vortices, viscosity from the decay", error, error_limit);
		sim_.stop();
	}

	static constexpr double error_limit = 0.01; ///< the largest relative error that passes (1 %)

private:
	/// @brief Gives the kinetic energy per density summed over the cells (only its decay is compared).
	/// @return the sum of the squared speeds in m^2/s^2
	double kinetic_energy() const {
		using namespace si::symbols;
		double energy{ 0.0 };
		const FieldReader fields = sim_.fields();
		const std::uint64_t count{ fields.cell_count() };
		for(std::uint64_t n{ 0u }; n < count; ++n) {
			const FieldReader::Cell c{ fields.cell(n) };
			const double speed{ c.velocity.magnitude().in(m / s) };
			energy += sq(speed);
		}
		return energy;
	}

	Simulation& sim_;                     ///< the simulation (not owned)
	si::Length size_{};                   ///< the wavelength
	si::KinematicViscosity viscosity_{};  ///< the expected viscosity
	double initial_energy_{ 0.0 };        ///< the energy at the start, in m^2/s^2
};

} // namespace

void main_setup() { // extensions: none (D3Q19, one cell high)
	using namespace si::symbols;
	const si::Length size = 1.0 * cm;         // a periodic square, one wavelength
	const si::Length cell = size / 128.0;     // 128 x 128 x 1 cells
	const si::KinematicViscosity viscosity = Fluid::WATER.kinematic_viscosity;
	const si::Density density = Fluid::WATER.density;
	const si::Speed amplitude = 100.0 * viscosity / size; // Reynolds number 100
	const auto k = 2.0 * pi / size;                      // wave number, 1/m
	const si::Duration decay_time = 1.0 / (2.0 * k * k * viscosity); // the velocity decays to 1/e

	Domain domain = Domain::box(size, size, cell);
	domain.cell_size(cell);
	Simulation sim(domain, Fluid::WATER, amplitude);

	const VortexField field(size, amplitude, density);
	sim.boundaries()
		.initialize_velocity([&field](si::Position p) { return field.velocity(p); })
		.initialize_pressure([&field](si::Position p) { return field.pressure(p); })
		.apply();

	DecayCheck check(sim, size, viscosity);
	sim.every(decay_time, [&check](si::Duration t) { check.at(t); });
	sim.run();
}
