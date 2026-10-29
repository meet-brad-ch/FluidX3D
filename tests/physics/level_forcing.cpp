// LEVEL_FORCING: the per-level terms of a convective boundary layer:
// per horizontal level z a force (fx, fy: the wind nudging), a damping rate r of the vertical velocity (the sponge under
// the lid) and a relaxation of the temperature toward a reference T_ref at the same rate (the sponge's temperature). The
// host sets them with LBM::set_level_forcing(fx, fy, rate, T_ref) at any time (one write of 4 Nz floats per domain).
// Three checks in a periodic box of air at rest (VOLUME_FORCE TEMPERATURE LEVEL_FORCING, nu 0.01, alpha 1e-3):
// 1. The nudging: every 20 steps the host sets fx(z) = (u_t(z) - <ux>(z)) / tau with the target u_t(z) = 0.01 sin(2 pi z / Nz)
//    and tau = 2000 steps, as a CBL's wind nudging (MicroHH's --nudge); the sine profile also diffuses at nu k^2
//    (9.6e-5 per step against 1/tau = 5e-4), so the layer means follow u_t / (1 + nu k^2 tau) (1 - e^(-(1 + nu k^2 tau)))
//    = 0.585 u_t after tau steps (0.632 without the diffusion; the first run measured 0.585); within 1 % of 0.01 at every
//    level, and the same run split into two domains along x must agree with the one-domain run to 1e-5 at every level.
// 2. The w damping: a uniform w = 0.01 everywhere (divergence-free) and r = 1/500 at every level; after 1500 steps
//    w = 0.01 e^-3 within 2 % (the damping is applied as a force -r rho w per step, Guo forcing: its own discretisation
//    error against the exponential is part of what is measured).
// 3. The temperature, in a second box at rest: T = 1.1 in the upper half, T_ref = 1 there at r = 1/500 (0 below);
//    after 1500 steps the levels in the middle of the upper half hold 1 + 0.1 e^-3 within 2 % (diffusion with alpha 1e-3
//    reaches 1.7 cells from the step in that time; the middle is far from it), the lower half stays at 1 within 1e-3.
//    At rest, so that no flow carries the step toward the checked levels.

// On the core directly, not the atmosphere API: the test checks the core's per-level terms with arrays set by hand in a
// periodic box without walls (a sine target, uniform damping); the API's LevelForcing writes these arrays and is checked
// in atmosphere_column and atmosphere_profiles.

#include "setup/setup.hpp"
#include "physics_check.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

namespace {

/// @brief The layer means of ux, w and T of one LBM over its cells.
struct ColumnMeans {
	std::vector<double> ux{}; ///< the mean ux of each level in lattice units
	std::vector<double> uz{}; ///< the mean w of each level in lattice units
	std::vector<double> T{};  ///< the mean T of each level in lattice temperature
};

/// @brief The worst errors of the sponge runs.
struct SpongeErrors {
	double worst_w{ 0.0 };  ///< the largest relative error of the damped w
	double worst_T{ 0.0 };  ///< the largest relative error of the relaxed T in the middle of the upper half
	uint worst_T_z{ 0u };   ///< the level of worst_T
	double lower_T{ 0.0 };  ///< the largest |T - 1| in the middle of the lower half
};

/// @brief The core's per-level terms in a periodic box: the nudging, the w damping and the T relaxation.
class LevelForcingChecks {
public:
	/// @brief The nudging in one domain and in two: checks the reached fraction of the target and the split.
	static void nudging() {
		const std::vector<double> one = nudged(1u);
		const std::vector<double> two = nudged(2u);
		double worst{ 0.0 };
		double split{ 0.0 };
		const double levels{ static_cast<double>(one.size()) };
		const double k{ 2.0 * pi_value / levels };
		const double decay{ viscosity_value * k * k * static_cast<double>(tau) }; // nu k^2 tau: the sine's viscous decay over tau
		const double reached{ (1.0 - std::exp(-(1.0 + decay))) / (1.0 + decay) }; // the fraction of u_t after tau steps
		const double peak{ target_amplitude * reached };
		std::printf("\nnudging after tau = 2000 steps (target 0.01 sin; with the viscous decay nu k^2 tau = %.3f the expected fraction is %.4f, %.5f at the peak):\n", decay, reached, peak);
		const uint level_count{ static_cast<uint>(one.size()) };
		for(uint z{ 0u }; z < level_count; z++) {
			const double phase{ 2.0 * pi_value * static_cast<double>(z) / levels };
			const double target{ target_amplitude * std::sin(phase) };
			const double expected{ reached * target };
			worst = std::fmax(worst, std::fabs(one[z] - expected) / target_amplitude);
			split = std::fmax(split, std::fabs(one[z] - two[z]));
			if(z % printed_every == 0u) {
				std::printf("  z %2u: <ux> %.6f (expected %.6f), two domains %.6f\n", z, one[z], expected, two[z]);
			}
		}
		PhysicsCheck::report("level nudging reaches the nudged-and-diffused fraction of the target at t = tau (of the amplitude)", worst, nudging_limit);
		PhysicsCheck::report("level nudging: two domains against one (absolute, limit 1e-5)", split, split_limit);
	}

	/// @brief The w damping and the T relaxation, each in its own box: checks e^-3 after 3/r steps.
	static void sponge() {
		const ColumnMeans damped = sponge_run(start_w, false);
		const ColumnMeans m = sponge_run(0.0f, true);
		const double e_minus_3{ static_cast<double>(static_cast<float>(std::exp(-3.0))) };
		const SpongeErrors errors = sponge_errors(damped, m, e_minus_3);
		std::printf("the worst checked level of the T relaxation: z %u, T %.6f; T at z %u..%u:", errors.worst_T_z, m.T[errors.worst_T_z], box_levels / 2u + shown_margin,
		            box_levels - shown_margin);
		for(uint z{ box_levels / 2u + shown_margin }; z <= box_levels - shown_margin; z++) {
			std::printf(" %.5f", m.T[z]);
		}
		std::printf("\n");
		const double expected_w{ start_w_value * e_minus_3 };
		const double expected_T{ 1.0 + temperature_step * e_minus_3 };
		const uint top{ box_levels - 1u };
		const uint middle{ box_levels / 2u };
		const uint upper{ box_levels * 3u / 4u };
		const uint lower{ box_levels / 4u };
		std::printf("\nafter 1500 steps at r = 1/500: <w> at z 0 / 32 / 63: %.6f / %.6f / %.6f (expected %.6f); T at z 48: %.6f (expected %.6f), at z 16: %.6f\n",
		            damped.uz[0], damped.uz[middle], damped.uz[top], expected_w, m.T[upper], expected_T, m.T[lower]);
		PhysicsCheck::report("level damping of w: e^-3 after 3/r steps", errors.worst_w, sponge_limit);
		PhysicsCheck::report("level relaxation of T toward T_ref: e^-3 after 3/r steps", errors.worst_T, sponge_limit);
		PhysicsCheck::report("level relaxation of T leaves the unsponged levels (absolute)", errors.lower_T, unsponged_limit);
	}

private:
	/// @brief Gives the layer means of ux, w and T.
	/// @param lbm the lattice
	/// @return the means of each level
	static ColumnMeans layer_means(LBM& lbm) {
		lbm.update_fields();
		lbm.u.read_from_device();
		lbm.T.read_from_device();
		const uint Nz = lbm.get_Nz();
		ColumnMeans m{};
		m.ux.assign(Nz, 0.0);
		m.uz.assign(Nz, 0.0);
		m.T.assign(Nz, 0.0);
		std::vector<double> count(Nz, 0.0);
		const ulong cells = lbm.get_N();
		for(ulong n{ 0ull }; n < cells; n++) {
			uint x{ 0u };
			uint y{ 0u };
			uint z{ 0u };
			lbm.coordinates(n, x, y, z);
			m.ux[z] += static_cast<double>(lbm.u.x[n]);
			m.uz[z] += static_cast<double>(lbm.u.z[n]);
			m.T[z] += static_cast<double>(lbm.T[n]);
			count[z] += 1.0;
		}
		for(uint z{ 0u }; z < Nz; z++) {
			m.ux[z] /= count[z];
			m.uz[z] /= count[z];
			m.T[z] /= count[z];
		}
		return m;
	}

	/// @brief Runs the nudging: every interval the host sets fx(z) = (u_t(z) - <ux>(z)) / tau.
	/// @param Dx the domains along x
	/// @return the layer means of ux after tau steps
	static std::vector<double> nudged(uint Dx) {
		LBM lbm(box_cells, box_cells, box_levels, Dx, 1u, 1u, viscosity, 0.0f, 0.0f, 0.0f, 0.0f, alpha, 0.0f);
		const ulong cells = lbm.get_N();
		for(ulong n{ 0ull }; n < cells; n++) {
			lbm.T[n] = 1.0f;
		}
		std::vector<float> fx(box_levels, 0.0f);
		const std::vector<float> fy(box_levels, 0.0f);
		const std::vector<float> rate(box_levels, 0.0f);
		const std::vector<float> T_ref(box_levels, 1.0f);
		std::vector<float> target(box_levels, 0.0f);
		for(uint z{ 0u }; z < box_levels; z++) {
			const float phase{ 2.0f * pi_float * static_cast<float>(z) / static_cast<float>(box_levels) };
			target[z] = target_amplitude_float * std::sin(phase);
		}
		lbm.set_level_forcing(fx, fy, rate, T_ref);
		lbm.run(0ull);
		const float tau_float{ static_cast<float>(tau) };
		for(ulong t{ 0ull }; t < tau; t += nudge_interval) {
			const ColumnMeans m = layer_means(lbm);
			for(uint z{ 0u }; z < box_levels; z++) {
				fx[z] = (target[z] - static_cast<float>(m.ux[z])) / tau_float; // the nudging force per step
			}
			lbm.set_level_forcing(fx, fy, rate, T_ref);
			lbm.run(nudge_interval);
		}
		const std::vector<double> ux = layer_means(lbm).ux;
		return ux;
	}

	/// @brief Runs a box with the sponge rate r at every level for 3/r steps.
	/// @param w the uniform start w
	/// @param T_step whether T starts at 1.1 in the upper half (else 1 everywhere)
	/// @return the layer means after the run
	static ColumnMeans sponge_run(float w, bool T_step) {
		LBM lbm(box_cells, box_cells, box_levels, viscosity, 0.0f, 0.0f, 0.0f, 0.0f, alpha, 0.0f);
		const ulong cells = lbm.get_N();
		for(ulong n{ 0ull }; n < cells; n++) {
			uint x{ 0u };
			uint y{ 0u };
			uint z{ 0u };
			lbm.coordinates(n, x, y, z);
			lbm.u.z[n] = w;
			const bool upper{ T_step && z >= box_levels / 2u };
			lbm.T[n] = upper ? start_warm : 1.0f;
		}
		const std::vector<float> fx(box_levels, 0.0f);
		const std::vector<float> fy(box_levels, 0.0f);
		const float r{ 1.0f / sponge_time };
		const std::vector<float> rate(box_levels, r);
		const std::vector<float> T_ref(box_levels, 1.0f);
		lbm.set_level_forcing(fx, fy, rate, T_ref);
		lbm.run(sponge_steps);
		const ColumnMeans m = layer_means(lbm);
		return m;
	}

	/// @brief Gives the worst errors of the two sponge runs.
	/// @param damped the w damping run
	/// @param m the T relaxation run
	/// @param e_minus_3 e^-3 (rounded to float, as the expected values)
	/// @return the errors
	static SpongeErrors sponge_errors(const ColumnMeans& damped, const ColumnMeans& m, double e_minus_3) {
		SpongeErrors errors{};
		const double expected_w{ start_w_value * e_minus_3 };
		const double expected_excess{ temperature_step * e_minus_3 };
		for(uint z{ 0u }; z < box_levels; z++) {
			errors.worst_w = std::fmax(errors.worst_w, std::fabs(damped.uz[z] / expected_w - 1.0));
			const bool upper_middle{ z >= box_levels / 2u + checked_margin && z < box_levels - checked_margin };
			if(upper_middle) {
				keep_worst_T(errors, m.T[z], expected_excess, z);
			}
			const bool lower_middle{ z >= checked_margin && z < box_levels / 2u - checked_margin };
			if(lower_middle) {
				errors.lower_T = std::fmax(errors.lower_T, std::fabs(m.T[z] - 1.0));
			}
		}
		return errors;
	}

	/// @brief Keeps the relaxed T's error at a level when it is the worst so far.
	/// @param errors the errors so far
	/// @param T the level's mean T
	/// @param expected_excess the expected T - 1 after the run
	/// @param z the level
	static void keep_worst_T(SpongeErrors& errors, double T, double expected_excess, uint z) {
		const double e{ std::fabs((T - 1.0) / expected_excess - 1.0) };
		if(e > errors.worst_T) {
			errors.worst_T = e;
			errors.worst_T_z = z;
		}
	}

	static constexpr uint box_cells = 32u;                ///< the box's cells along x and y
	static constexpr uint box_levels = 64u;               ///< the box's levels
	static constexpr float viscosity = 0.01f;             ///< the kinematic viscosity in lattice units
	static constexpr float alpha = 1.0E-3f;               ///< the thermal diffusion coefficient in lattice units
	static constexpr ulong tau = 2000ull;                 ///< the nudging time scale in steps
	static constexpr ulong nudge_interval = 20ull;        ///< the steps between two nudging updates
	static constexpr double pi_value = 3.14159265;        ///< pi as the test wrote it (the expected values)
	static constexpr float pi_float = 3.14159265f;        ///< pi as the test wrote it (the target profile)
	static constexpr double target_amplitude = 0.01;      ///< the target's amplitude in lattice units
	static constexpr float target_amplitude_float = 0.01f; ///< the target's amplitude as the core's float
	static constexpr uint printed_every = 8u;             ///< the levels between two printed levels
	static constexpr double viscosity_value = 0.01;       ///< nu as the expected values write it (the double 0.01)
	static constexpr float start_w = 0.01f;               ///< the damping run's uniform start w
	static constexpr double start_w_value = 0.01;         ///< the start w as the expected values write it
	static constexpr float start_warm = 1.1f;             ///< the relaxation run's start T in the upper half
	static constexpr double temperature_step = 0.1;       ///< the T step in the upper half, as the expected values write it
	static constexpr float sponge_time = 500.0f;          ///< 1/r in steps
	static constexpr ulong sponge_steps = 1500ull;        ///< 3/r steps
	static constexpr uint checked_margin = 8u;            ///< the levels next to the step and the box's ends that are not checked
	static constexpr uint shown_margin = 6u;              ///< the margin of the printed T levels
	static constexpr double nudging_limit = 0.01;         ///< the largest nudging error that passes (of the amplitude)
	static constexpr double split_limit = 1.0E-5;         ///< the largest difference between one domain and two
	static constexpr double sponge_limit = 0.02;          ///< the largest relative damping and relaxation error (2 %)
	static constexpr double unsponged_limit = 1.0E-3;     ///< the largest |T - 1| of the unsponged levels
};

} // namespace

void main_setup() { // extensions: VOLUME_FORCE TEMPERATURE LEVEL_FORCING
	LevelForcingChecks::nudging();
	LevelForcingChecks::sponge();
}
