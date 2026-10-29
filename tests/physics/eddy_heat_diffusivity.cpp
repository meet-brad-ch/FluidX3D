// Eddy heat diffusivity: a temperature wave T = 1 + A sin(2 pi y / Ny) decays by diffusion at exp(-D k^2 t). In a
// steady Couette flow u_x(z) between two equilibrium boundary layers, the flow carries no heat across y, and the
// Smagorinsky model of SUBGRID gives the uniform eddy viscosity nu_t = (C Delta)^2 S in the bulk, S = du_x/dz,
// (C Delta)^2 = 0.76421222/(18 sqrt 2) = 0.030019 (the constant in kernel.cpp). With LBM::set_inv_prandtl_t(1/Pr_t)
// the wave must decay with D_0 + nu_t/Pr_t, and without it with D_0 alone.
// D_0 is the diffusivity of the thermal relaxation rate FluidX3D sets for alpha, wT = 1/(2 alpha + 1/2): the D3Q7
// lattice diffuses with D = c^2 (tauT - 1/2), c^2 = 1/4, so D_0 = alpha/2.
// Measured in the middle half of the channel: the shear rate from the velocity field (the equilibrium layers make it a
// few % steeper than 2U/(Nz-1)), the diffusivity from the wave's amplitude at two times.
// Passes when the measured diffusivity is within 3 % of the expected one in both runs. With SCALAR_FV the finite
// volumes add diffusion across a flow at high cell Peclet numbers (here several hundred), a documented limit of the
// scheme: the total D is printed as information, and the gate is the eddy part, D with less D without, against
// nu_t/Pr_t within 3 %.

// On the core directly, not the atmosphere API: the test calibrates the core's eddy heat diffusivity in a Couette flow
// between equilibrium boundaries, in lattice units (no atmosphere: no floor heating, no buoyancy).

#include "setup/setup.hpp"
#include "physics_check.hpp"

#include <cmath>

namespace {

/// @brief What one run measured in the middle half of the channel, in lattice units.
struct Measured {
	double diffusivity{ 0.0 }; ///< the decay rate of the temperature wave over k^2
	double shear{ 0.0 };       ///< du_x/dz
};

/// @brief The sums of a least-squares line through (z, u_x).
struct SlopeSums {
	double sz{ 0.0 };    ///< the sum of z
	double su{ 0.0 };    ///< the sum of u_x
	double szz{ 0.0 };   ///< the sum of z^2
	double szu{ 0.0 };   ///< the sum of z u_x
	double cells{ 0.0 }; ///< the number of points
};

/// @brief A temperature wave across a Couette flow with walls at u_x = -U and +U: its decay and the shear rate.
class CouetteWave {
public:
	/// @brief Runs the wave and measures its decay and the shear rate in the middle half of the channel.
	/// @param inv_prandtl_t the eddy heat diffusivity's 1/Pr_t (0: off)
	/// @param alpha the core's alpha in lattice units
	/// @param U the walls' speed in lattice units
	/// @param Nz the channel's cells along z, the walls included
	/// @return the measurement
	static Measured measure(float inv_prandtl_t, float alpha, float U, uint Nz) {
		const double k{ 2.0 * pi / static_cast<double>(wave_cells) }; // pi: the macro of utilities.hpp
		LBM lbm(side_cells, wave_cells, Nz, viscosity, 0.0f, 0.0f, 0.0f, 0.0f, alpha, 0.0f);
		start(lbm, k, U, Nz);
		lbm.set_inv_prandtl_t(inv_prandtl_t);
		lbm.run(first_step);
		const double A1{ amplitude(lbm, k, Nz) };
		lbm.run(last_step - first_step);
		const double A2{ amplitude(lbm, k, Nz) };
		Measured m{};
		const double decay_steps{ static_cast<double>(last_step - first_step) };
		m.diffusivity = std::log(A1 / A2) / (k * k * decay_steps);
		m.shear = shear_of(lbm, Nz);
		return m;
	}

private:
	/// @brief Tells whether a level is in the middle half of the channel.
	/// @param z the level
	/// @param Nz the channel's cells along z
	/// @return true for Nz/4 <= z <= 3 Nz/4
	static bool middle(uint z, uint Nz) {
		const bool above_quarter{ 4u * z >= Nz };
		const bool below_three_quarters{ 4u * z <= 3u * Nz };
		return above_quarter && below_three_quarters;
	}

	/// @brief Writes the Couette profile, the wave and the equilibrium walls.
	/// @param lbm the lattice
	/// @param k the wave number in 1/cells
	/// @param U the walls' speed in lattice units
	/// @param Nz the channel's cells along z
	static void start(LBM& lbm, double k, float U, uint Nz) {
		const float zc{ 0.5f * static_cast<float>(Nz - 1u) };
		const float S{ 2.0f * U / static_cast<float>(Nz - 1u) }; // the start profile's shear rate between the layers z = 0 and z = Nz-1
		const ulong cells = lbm.get_N();
		for(ulong n{ 0ull }; n < cells; n++) {
			uint x{ 0u };
			uint y{ 0u };
			uint z{ 0u };
			lbm.coordinates(n, x, y, z);
			lbm.u.x[n] = S * (static_cast<float>(z) - zc); // the Couette profile from the start
			const double phase{ k * static_cast<double>(y) };
			const float wave{ static_cast<float>(std::sin(phase)) };
			lbm.T[n] = 1.0f + start_amplitude * wave;
			if(z == 0u || z == Nz - 1u) {
				lbm.flags[n] = TYPE_E;
			}
		}
	}

	/// @brief Gives the sine mode's amplitude over the middle half of the channel.
	/// @param lbm the lattice
	/// @param k the wave number in 1/cells
	/// @param Nz the channel's cells along z
	/// @return the amplitude in lattice temperature
	static double amplitude(LBM& lbm, double k, uint Nz) {
		lbm.update_fields();
		lbm.T.read_from_device();
		double s{ 0.0 };
		ulong counted{ 0ull };
		const ulong cells = lbm.get_N();
		for(ulong n{ 0ull }; n < cells; n++) {
			uint x{ 0u };
			uint y{ 0u };
			uint z{ 0u };
			lbm.coordinates(n, x, y, z);
			if(!middle(z, Nz)) {
				continue;
			}
			const double phase{ k * static_cast<double>(y) };
			s += (static_cast<double>(lbm.T[n]) - 1.0) * std::sin(phase);
			counted++;
		}
		const double a{ 2.0 * s / static_cast<double>(counted) };
		return a;
	}

	/// @brief Gives the shear rate: the least-squares slope of u_x over z in the middle half.
	/// @param lbm the lattice after its run
	/// @param Nz the channel's cells along z
	/// @return du_x/dz in lattice units
	static double shear_of(LBM& lbm, uint Nz) {
		lbm.u.read_from_device();
		SlopeSums sums{};
		const ulong cells = lbm.get_N();
		for(ulong n{ 0ull }; n < cells; n++) {
			uint x{ 0u };
			uint y{ 0u };
			uint z{ 0u };
			lbm.coordinates(n, x, y, z);
			if(!middle(z, Nz)) {
				continue;
			}
			const double zd{ static_cast<double>(z) };
			const double ud{ static_cast<double>(lbm.u.x[n]) };
			sums.sz += zd;
			sums.su += ud;
			sums.szz += zd * zd;
			sums.szu += zd * ud;
			sums.cells += 1.0;
		}
		const double slope{ (sums.cells * sums.szu - sums.sz * sums.su) / (sums.cells * sums.szz - sums.sz * sums.sz) };
		return slope;
	}

	static constexpr uint side_cells = 4u;          ///< the channel's cells along x
	static constexpr uint wave_cells = 64u;         ///< the channel's cells along y, one wavelength
	static constexpr float viscosity = 1.0E-4f;     ///< the kinematic viscosity in lattice units
	static constexpr float start_amplitude = 0.01f; ///< the wave's start amplitude A in lattice temperature
	static constexpr ulong first_step = 2000ull;    ///< the step of the first amplitude
	static constexpr ulong last_step = 30000ull;    ///< the step of the second amplitude
};

} // namespace

void main_setup() { // extensions: EQUILIBRIUM_BOUNDARIES TEMPERATURE SUBGRID
	const float alpha{ 1.0E-4f };
	const float U{ 0.15f };
	const float inv_prandtl_t{ 3.0f }; // Pr_t = 1/3
	const uint Nz{ 98u }; // tall: the faster decay next to the equilibrium layers (steeper shear there) spreads ~5 cells by t2, far from the middle half
#ifdef THERMAL_DIFFUSIVITY_FIX
	const double D0{ static_cast<double>(alpha) }; // wT = 1/(4 alpha + 1/2): alpha is the diffusivity (also SCALAR_FV, which needs the fix)
#else // THERMAL_DIFFUSIVITY_FIX
	const double D0{ 0.5 * static_cast<double>(alpha) }; // wT = 1/(2 alpha + 1/2): the D3Q7 lattice diffuses with alpha/2
#endif // THERMAL_DIFFUSIVITY_FIX
	const double smagorinsky_constant{ 0.76421222 }; // the constant in kernel.cpp
	const double smagorinsky_divisor{ 18.0 * std::sqrt(2.0) };
	const double smagorinsky{ smagorinsky_constant / smagorinsky_divisor }; // (C Delta)^2
	const double error_limit{ 0.03 }; // the largest relative error that passes (3 %)

	const Measured plain = CouetteWave::measure(0.0f, alpha, U, Nz);
	std::printf("\nwithout eddy heat diffusivity: D %.4e (expected D_0 %.4e)\n", plain.diffusivity, D0);
#ifndef SCALAR_FV
	const double plain_error{ std::fabs(plain.diffusivity / D0 - 1.0) };
	PhysicsCheck::report("eddy heat diffusivity: off, D_0 only", plain_error, error_limit);
#endif // SCALAR_FV

	const Measured eddy = CouetteWave::measure(inv_prandtl_t, alpha, U, Nz);
	const double nu_t{ smagorinsky * eddy.shear };
	const double eddy_expected{ nu_t * static_cast<double>(inv_prandtl_t) };
	const double expected{ D0 + eddy_expected };
	std::printf("\nwith eddy heat diffusivity: D %.4e (expected %.4e; shear %.4e, nu_t %.4e)\n", eddy.diffusivity, expected, eddy.shear, nu_t);
#ifdef SCALAR_FV
	const double eddy_part{ eddy.diffusivity - plain.diffusivity };
	const double plain_ratio{ plain.diffusivity / D0 };
	const double eddy_ratio{ eddy.diffusivity / expected };
	std::printf("information: D/expected without the eddy part %.3f, with it %.3f (the extra diffusion across the flow)\n", plain_ratio, eddy_ratio);
	std::printf("the eddy part: D with less D without %.4e (expected nu_t/Pr_t %.4e)\n", eddy_part, eddy_expected);
	const double part_error{ std::fabs(eddy_part / eddy_expected - 1.0) };
	PhysicsCheck::report("eddy heat diffusivity: the eddy part nu_t/Pr_t", part_error, error_limit);
#else // SCALAR_FV
	const double eddy_error{ std::fabs(eddy.diffusivity / expected - 1.0) };
	PhysicsCheck::report("eddy heat diffusivity: D_0 + nu_t/Pr_t", eddy_error, error_limit);
#endif // SCALAR_FV
}
