// The thermal diffusivity: a temperature wave T = 1 + A sin(k y) in a periodic box decays by diffusion as
// exp(-D k^2 t). The D3Q7 lattice diffuses with D = c^2 (tauT - 1/2), c^2 = 1/4, so FluidX3D's relaxation rate
// wT = 1/(2 alpha + 1/2) gives D = alpha/2. With THERMAL_DIFFUSIVITY_FIX the rate is 1/(4 alpha + 1/2) and D = alpha.
// SCALAR_FV builds (they need the fix) carry T by finite volumes with D = alpha. Each case is measured against the
// diffusivity of the build (alpha/2 or alpha):
// 1. at rest, alpha = 2e-3 and 0.125: within 1 %;
// 2. in a uniform flow u = 0.1 along x, the wave along y (the flow carries no heat across the wave): within 1 %;
// 3. in the same flow, the wave along x (carried with the flow): within 5 %. The D3Q7 equilibrium is linear in u, so
//    its diffusion along the flow is D - u^2 (tauT - 1/2) (printed).
// With SCALAR_FV, the flow cases at alpha 2e-3 (cell Peclet number 50) are printed as information only: there the
// limiter and the diagonal links add diffusion (D/alpha about 1.07 across and 1.3 along the flow), a documented limit
// of the scheme (research/convection/README.md). FP16S thermal DDFs do not diffuse a wave in a flow at all: TEMPERATURE
// needs FP32 storage.

#include "setup/setup.hpp"
#include "physics_check.hpp"

#include <cmath>

namespace {

/// @brief One measurement of the wave's decay, in lattice units.
struct WaveDecay {
	double diffusivity{ 0.0 }; ///< log(A1/A2) / (k^2 (t2 - t1))
	double amplitude1{ 0.0 };  ///< the wave's amplitude at t1
	double amplitude2{ 0.0 };  ///< the wave's amplitude at t2
};

/// @brief A wave of the build's diffusivity, at rest and in a uniform flow.
class ThermalWave {
public:
	/// @brief Gives the diffusivity that the build's relaxation rate implies for alpha.
	/// @param alpha the core's alpha in lattice units
	/// @return alpha with THERMAL_DIFFUSIVITY_FIX, alpha/2 without it
	static double expected_diffusivity(double alpha) {
#ifdef THERMAL_DIFFUSIVITY_FIX
		return alpha;
#else // THERMAL_DIFFUSIVITY_FIX
		return 0.5 * alpha;
#endif // THERMAL_DIFFUSIVITY_FIX
	}

	/// @brief Gives the lattice relaxation time the build sets for alpha.
	/// @param alpha the core's alpha in lattice units
	/// @return tauT = 4 D + 1/2
	static double relaxation_time(double alpha) {
		const double tau{ 4.0 * expected_diffusivity(alpha) + 0.5 };
		return tau;
	}

	/// @brief Gives the steps so that the wave decays by about a factor e between t1 and t2 (D k^2 (t2 - t1) = 1),
	/// at least shortest_run.
	/// @param alpha the core's alpha in lattice units
	/// @return t2
	static ulong steps_for(double alpha) {
		const double k{ 2.0 * pi / static_cast<double>(wave_cells) };
		const double D{ expected_diffusivity(alpha) };
		const double steps{ 1.0 / (D * k * k) };
		ulong t2{ shortest_run };
		if(steps >= static_cast<double>(shortest_run)) {
			t2 = static_cast<ulong>(steps);
		}
		return t2;
	}

	/// @brief Runs a wave of amplitude A along x or y in a periodic box in uniform motion u (along x) from t1 to t2.
	/// @param alpha the core's alpha in lattice units
	/// @param u the flow along x in lattice units
	/// @param along_x whether the wave runs along x (with the flow) or along y (across it)
	/// @param t1 the step of the first amplitude
	/// @param t2 the step of the second amplitude
	/// @return the measured decay
	static WaveDecay measure(float alpha, float u, bool along_x, ulong t1, ulong t2) {
		const double k{ 2.0 * pi / static_cast<double>(wave_cells) }; // pi: the macro of utilities.hpp
		const uint cells_x{ along_x ? wave_cells : side_cells };
		const uint cells_y{ along_x ? side_cells : wave_cells };
		LBM lbm(cells_x, cells_y, box_levels, viscosity, 0.0f, 0.0f, 0.0f, 0.0f, alpha, 0.0f);
		const ulong cells = lbm.get_N();
		for(ulong n{ 0ull }; n < cells; n++) {
			uint x{ 0u };
			uint y{ 0u };
			uint z{ 0u };
			lbm.coordinates(n, x, y, z);
			lbm.u.x[n] = u; // the uniform flow (periodic box: it stays)
			const uint along{ along_x ? x : y };
			const double phase{ k * static_cast<double>(along) };
			const float wave{ static_cast<float>(std::sin(phase)) };
			lbm.T[n] = 1.0f + start_amplitude * wave;
		}
		lbm.run(t1);
		WaveDecay d{};
		d.amplitude1 = amplitude(lbm, k, along_x);
		lbm.run(t2 - t1);
		d.amplitude2 = amplitude(lbm, k, along_x);
		const double decay_steps{ static_cast<double>(t2 - t1) };
		d.diffusivity = std::log(d.amplitude1 / d.amplitude2) / (k * k * decay_steps);
		return d;
	}

	/// @brief Prints and reports one case: the measured diffusivity against the expected one.
	/// @param name the check's name
	/// @param d the measured decay
	/// @param expected the expected diffusivity in lattice units
	/// @param limit the largest relative error that passes
	/// @return true when the check passed
	static bool check(const char* name, const WaveDecay& d, double expected, double limit) {
		const double ratio{ d.diffusivity / expected };
		std::printf("\n%s: D %.5e, expected %.5e, ratio %.4f (A %.3e -> %.3e)\n", name, d.diffusivity, expected, ratio, d.amplitude1, d.amplitude2);
		const double error{ std::fabs(ratio - 1.0) };
		const bool passed{ PhysicsCheck::report(name, error, limit) };
		return passed;
	}

	static constexpr ulong first_part = 10ull;        ///< t1 = t2 / first_part
	static constexpr double rest_limit = 0.01;        ///< the largest relative error at rest and across a flow (1 %)
	static constexpr double along_limit = 0.05;       ///< the largest relative error along a flow (5 %)
	static constexpr std::size_t name_capacity = 128u; ///< the length of a check's name

private:
	/// @brief Gives the wave's amplitude: the magnitude of its Fourier component over all cells (sine and cosine parts,
	/// so a wave carried with the flow is measured wherever it stands).
	/// @param lbm the lattice
	/// @param k the wave number in 1/cells
	/// @param along_x whether the wave runs along x
	/// @return the amplitude in lattice temperature
	static double amplitude(LBM& lbm, double k, bool along_x) {
		lbm.update_fields();
		lbm.T.read_from_device();
		double s{ 0.0 };
		double c{ 0.0 };
		ulong counted{ 0ull };
		const ulong cells = lbm.get_N();
		for(ulong n{ 0ull }; n < cells; n++) {
			uint x{ 0u };
			uint y{ 0u };
			uint z{ 0u };
			lbm.coordinates(n, x, y, z);
			const uint along{ along_x ? x : y };
			const double phase{ k * static_cast<double>(along) };
			const double dT{ static_cast<double>(lbm.T[n]) - 1.0 };
			s += dT * std::sin(phase);
			c += dT * std::cos(phase);
			counted++;
		}
		const double magnitude{ std::sqrt(s * s + c * c) };
		const double a{ 2.0 * magnitude / static_cast<double>(counted) };
		return a;
	}

	static constexpr uint wave_cells = 64u;         ///< the box's cells along the wave, one wavelength
	static constexpr uint side_cells = 4u;          ///< the box's cells across the wave
	static constexpr uint box_levels = 8u;          ///< the box's cells along z
	static constexpr float viscosity = 1.0E-4f;     ///< the kinematic viscosity in lattice units
	static constexpr float start_amplitude = 0.01f; ///< the wave's start amplitude A in lattice temperature
	static constexpr ulong shortest_run = 1000ull;  ///< the fewest steps of a run
};

/// @brief The test's cases: the build line and the three cases at each alpha.
class ThermalDiffusivityCases {
public:
	/// @brief Prints the build's thermal rate and DDF precision.
	static void print_build() {
	#ifdef THERMAL_DIFFUSIVITY_FIX
		std::printf("\nbuild: THERMAL_DIFFUSIVITY_FIX (wT = 1/(4 alpha + 1/2), expected D = alpha)\n");
	#else // THERMAL_DIFFUSIVITY_FIX
		std::printf("\nbuild: upstream rate (wT = 1/(2 alpha + 1/2), expected D = alpha/2)\n");
	#endif // THERMAL_DIFFUSIVITY_FIX
	#if defined(FP16S)
		std::printf("precision: FP16S thermal DDFs\n");
	#elif defined(FP16C)
		std::printf("precision: FP16C thermal DDFs\n");
	#else
		std::printf("precision: FP32 thermal DDFs\n");
	#endif
	}

	/// @brief Measures the three cases at one alpha.
	/// @param alpha the core's alpha in lattice units
	/// @param small whether this is the small alpha (cell Peclet number 50 in the flow)
	/// @param u the flow along x in lattice units
	static void check_alpha(float alpha, bool small, float u) {
		const double alpha_d{ static_cast<double>(alpha) };
		const double expected{ ThermalWave::expected_diffusivity(alpha_d) };
		const ulong t2{ ThermalWave::steps_for(alpha_d) };
		const ulong t1{ t2 / ThermalWave::first_part };
		const double tau{ ThermalWave::relaxation_time(alpha_d) };
		const unsigned long long printed_t1{ static_cast<unsigned long long>(t1) };
		const unsigned long long printed_t2{ static_cast<unsigned long long>(t2) };
		std::printf("\nalpha %.4e: tauT %.5f, steps %llu to %llu\n", alpha, tau, printed_t1, printed_t2);
		char name[ThermalWave::name_capacity]{};
		std::snprintf(name, sizeof(name), "thermal diffusivity at rest, alpha %.0e", alpha);
		ThermalWave::check(name, ThermalWave::measure(alpha, 0.0f, false, t1, t2), expected, ThermalWave::rest_limit);
	#ifdef SCALAR_FV
		if(small) { // cell Peclet number 50: the limiter's and the diagonal links' extra diffusion, a documented limit
			const double across{ ThermalWave::measure(alpha, u, false, t1, t2).diffusivity / expected };
			const double along{ ThermalWave::measure(alpha, u, true, t1, t2).diffusivity / expected };
			const double peclet{ static_cast<double>(u) / alpha_d };
			std::printf("information: D/alpha across a flow u %.2f %.3f, along it %.3f (alpha %.0e, cell Peclet number %.0f)\n", u, across, along, alpha, peclet);
			return;
		}
	#else // SCALAR_FV
		static_cast<void>(small); // only SCALAR_FV treats the small alpha apart
	#endif // SCALAR_FV
		std::snprintf(name, sizeof(name), "thermal diffusivity across a flow u %.2f, alpha %.0e", u, alpha);
		ThermalWave::check(name, ThermalWave::measure(alpha, u, false, t1, t2), expected, ThermalWave::rest_limit);
		std::snprintf(name, sizeof(name), "thermal diffusivity along a flow u %.2f, alpha %.0e", u, alpha);
		const double u_d{ static_cast<double>(u) };
		const double galilean{ expected - u_d * u_d * (tau - 0.5) }; // the linear equilibrium's error along the flow
		std::printf("(along the flow the lattice's diffusion is D - u^2 (tauT - 1/2) = %.5e)\n", galilean);
		ThermalWave::check(name, ThermalWave::measure(alpha, u, true, t1, t2), expected, ThermalWave::along_limit);
	}
};

} // namespace

void main_setup() { // extensions: TEMPERATURE
	const float alpha_small{ 2.0E-3f };
	const float alpha_large{ 0.125f };
	const float u{ 0.1f };
	ThermalDiffusivityCases::print_build();
	ThermalDiffusivityCases::check_alpha(alpha_small, true, u);
	ThermalDiffusivityCases::check_alpha(alpha_large, false, u);
}
