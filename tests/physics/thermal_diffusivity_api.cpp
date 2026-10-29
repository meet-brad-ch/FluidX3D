// The thermal diffusivity through the Setup API: a Simulation made with a fluid's thermal diffusivity must diffuse heat
// with that diffusivity in every build. The core's D3Q7 rate gives D = alpha/2 without THERMAL_DIFFUSIVITY_FIX and
// D = alpha with it (thermal_diffusivity.cpp measures the core); Simulation gives the core the alpha of
// ScalarTransportRules::core_alpha(), so the measured D is the fluid's in all three builds (upstream rate, the fix,
// SCALAR_FV). A temperature wave T = 1 + A sin(k y) in a periodic box at rest decays as exp(-D k^2 t).
// Passes when the measured diffusivity is within 1 % of the fluid's, at the lattice diffusivities 2e-3 and 0.0625.

#include "setup/setup.hpp"
#include "physics_check.hpp"

#include <cmath>

namespace {

/// @brief Measures the decay of a temperature wave in a Simulation of air whose thermal diffusivity is a lattice value
/// at the test's scale, against that value.
class WaveDecay {
public:
	/// @brief Runs the wave and reports the measured diffusivity against the wanted one.
	/// @param lattice_diffusivity the wanted diffusivity in lattice units
	/// @return true when the check passed
	static bool check(float lattice_diffusivity) {
		using namespace si::symbols;
		// the scale does not depend on the fluid's diffusivity: a first Simulation gives it (no LBM is created)
		const si::Speed reference = reference_speed();
		const si::LatticeConverter scale = Simulation(wave_box(), Fluid::AIR, reference).converter();
		FluidProperties fluid = Fluid::AIR;
		const Diffusivity wanted = Diffusivity::lattice(si::LatticeDiffusivity(lattice_diffusivity));
		fluid.thermal_diffusivity = wanted.physical_value(scale);
		Simulation sim(wave_box(), fluid, reference);
		const si::Temperature cold = si::kelvin_point(299.5);
		const si::Temperature hot = si::kelvin_point(300.5);
		sim.set_temperatures(cold, hot); // the lattice temperature 1 at 300 K, no gravity: no buoyancy
		const double expected{ static_cast<double>(sim.converter().viscosity(fluid.thermal_diffusivity)) }; // the fluid's, in lattice units

		LBM& lbm = sim.lbm(); // expert use: the wave in lattice temperature
		const double k{ 2.0 * pi / static_cast<double>(wave_cells) };
		start_wave(lbm, k);
		const double steps{ 1.0 / (expected * k * k) }; // a decay by about a factor e
		ulong t2{ shortest_run };
		if(steps >= static_cast<double>(shortest_run)) {
			t2 = static_cast<ulong>(steps);
		}
		const ulong t1{ t2 / first_part };
		lbm.run(t1);
		const double a1{ amplitude(lbm, k) };
		lbm.run(t2 - t1);
		const double a2{ amplitude(lbm, k) };
		const double decay_steps{ static_cast<double>(t2 - t1) };
		const double measured{ std::log(a1 / a2) / (k * k * decay_steps) };
		const double fluid_m2s{ fluid.thermal_diffusivity.in(m * m / s) };
		const double ratio{ measured / expected };
		const unsigned long long printed_t1{ static_cast<unsigned long long>(t1) };
		const unsigned long long printed_t2{ static_cast<unsigned long long>(t2) };
		std::printf("\nfluid diffusivity %.5e m2/s = %.5e in lattice units: measured %.5e, ratio %.4f (A %.3e -> %.3e, steps %llu to %llu)\n",
		            fluid_m2s, expected, measured, ratio, a1, a2, printed_t1, printed_t2);
		char name[name_capacity]{};
		const double printed_diffusivity{ static_cast<double>(lattice_diffusivity) };
		std::snprintf(name, sizeof(name), "thermal diffusivity through the Setup API, lattice value %.0e", printed_diffusivity);
		const double error{ std::fabs(ratio - 1.0) };
		const bool passed{ PhysicsCheck::report(name, error, error_limit) };
		return passed;
	}

private:
	/// @brief Gives the box of the test: 4 x 64 x 8 cells of 1 cm, the wave along y.
	/// @return the box
	static Domain wave_box() {
		using namespace si::symbols;
		const si::Length cell = 1.0 * cm;
		const double width_cells{ 4.0 };
		const double height_cells{ 8.0 };
		const si::Length width = width_cells * cell;
		const si::Length length = static_cast<double>(wave_cells) * cell;
		const si::Length height = height_cells * cell;
		Domain box = Domain::box(width, length, height);
		box.cell_size(cell);
		return box;
	}

	/// @brief Gives the reference speed of the test: it sets the time step, nothing moves.
	/// @return 1 m/s
	static si::Speed reference_speed() {
		using namespace si::symbols;
		const si::Speed speed = 1.0 * m / s;
		return speed;
	}

	/// @brief Writes the start wave T = 1 + A sin(k y) into every cell.
	/// @param lbm the lattice
	/// @param k the wave number in 1/cells
	static void start_wave(LBM& lbm, double k) {
		const ulong cells = lbm.get_N();
		for(ulong n{ 0ull }; n < cells; n++) {
			uint x{ 0u };
			uint y{ 0u };
			uint z{ 0u };
			lbm.coordinates(n, x, y, z);
			const double phase{ k * static_cast<double>(y) };
			const float wave{ static_cast<float>(std::sin(phase)) };
			lbm.T[n] = 1.0f + start_amplitude * wave;
		}
	}

	/// @brief Gives the wave's amplitude: the magnitude of its Fourier component along y over all cells.
	/// @param lbm the lattice
	/// @param k the wave number in 1/cells
	/// @return the amplitude in lattice temperature
	static double amplitude(LBM& lbm, double k) {
		lbm.update_fields();
		lbm.T.read_from_device();
		double sine_sum{ 0.0 };
		double cosine_sum{ 0.0 };
		const ulong cells = lbm.get_N();
		for(ulong n{ 0ull }; n < cells; n++) {
			uint x{ 0u };
			uint y{ 0u };
			uint z{ 0u };
			lbm.coordinates(n, x, y, z);
			const double dT{ static_cast<double>(lbm.T[n]) - 1.0 };
			const double phase{ k * static_cast<double>(y) };
			sine_sum += dT * std::sin(phase);
			cosine_sum += dT * std::cos(phase);
		}
		const double magnitude{ std::sqrt(sine_sum * sine_sum + cosine_sum * cosine_sum) };
		const double a{ 2.0 * magnitude / static_cast<double>(cells) };
		return a;
	}

	static constexpr uint32_t wave_cells = 64u;       ///< the box's cells along y, one wavelength
	static constexpr float start_amplitude = 0.01f;   ///< the wave's start amplitude A in lattice temperature
	static constexpr ulong shortest_run = 1000ull;    ///< the fewest steps of a run
	static constexpr ulong first_part = 10ull;        ///< the run's first part, 1/10, before the first amplitude
	static constexpr std::size_t name_capacity = 128u; ///< the length of the check's name
	static constexpr double error_limit = 0.01;       ///< the largest relative error that passes (1 %)
};

} // namespace

void main_setup() { // extensions: TEMPERATURE (+ THERMAL_DIFFUSIVITY_FIX, + SCALAR_FV)
#if defined(SCALAR_FV)
	std::printf("\nbuild: SCALAR_FV (finite volumes, D = alpha)\n");
#elif defined(THERMAL_DIFFUSIVITY_FIX)
	std::printf("\nbuild: THERMAL_DIFFUSIVITY_FIX (the core diffuses with alpha)\n");
#else
	std::printf("\nbuild: upstream rate (the core diffuses with alpha/2, Simulation gives it twice the diffusivity)\n");
#endif
	for(const float d : { 2.0E-3f, 0.0625f }) {
		WaveDecay::check(d);
	}
}
