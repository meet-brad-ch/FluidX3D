// Thermal diffusivity of the D3Q7 temperature lattice: a temperature wave T = 1 + A sin(k y) in a periodic box decays
// by diffusion as exp(-D k^2 t). The lattice's diffusivity is D = c^2 (tauT - 1/2) with c^2 = 1/4, so the relaxation
// rate FluidX3D sets, wT = 1/(2 alpha + 1/2), gives D = alpha/2 and not alpha (the defect the cbl-heat branch found); with
// THERMAL_DIFFUSIVITY_FIX the rate is 1/(4 alpha + 1/2) and D = alpha. Measured, each against the diffusivity the build's
// rate implies (alpha/2 or alpha):
// 1. at rest, alpha = 2e-3 (tauT 0.504 or 0.508: the convective boundary layer's effective diffusivity, the eddy part
//    of the subgrid model; at the molecular value 1e-4, tauT 0.5002, the diffusion of a wave per step is below the
//    float32 rounding of the populations once the air moves, measured 0.08 to 0.55 of the expected value, 2026-10-25);
// 2. at rest, alpha = 0.125 (tauT well away from 1/2: 0.75 without the fix, 1.0 with it);
// 3. in a uniform flow u = 0.1 along x, the wave along y (the flow carries no heat across the wave): the same D;
// 4. in the same flow, the wave along x (carried with the flow): the D3Q7 equilibrium is linear in u, so its diffusion
//    along the flow is D - u^2 (tauT - 1/2) (the Galilean error of the lattice), reported, limit 5 %.
// The same source builds in FP16S (thermal_diffusivity_fp16s.cpp) and with the fix (thermal_diffusivity_fixed.cpp,
// thermal_diffusivity_fixed_fp16s.cpp); the test reads the build's macros for the expected values.
// Passes when 1-3 are within 1 % and 4 within 5 %. The FP16S builds are registered as expected to fail: with FP16S
// thermal DDFs a wave across or along a flow of u = 0.1 does not decay at all (measured 2026-10-25: the diffusive change
// per step is below the half-float resolution of the populations that carry the u term), and at rest the decay is 1.4
// to 7.4 % off; the temperature lattice needs FP32 storage.

#include "setup/setup.hpp"
#include "physics_check.hpp"

#include <cmath>

namespace {

/// One measurement of the wave's decay, in lattice units.
struct WaveDecay {
	double diffusivity = 0.0; ///< log(A1/A2) / (k^2 (t2 - t1))
	double amplitude1 = 0.0;  ///< the wave's amplitude at t1
	double amplitude2 = 0.0;  ///< the wave's amplitude at t2
};

/// The diffusivity the build's relaxation rate implies for alpha.
double expected_diffusivity(const double alpha) {
#ifdef THERMAL_DIFFUSIVITY_FIX
	return alpha;
#else // THERMAL_DIFFUSIVITY_FIX
	return 0.5*alpha;
#endif // THERMAL_DIFFUSIVITY_FIX
}

/// The lattice relaxation time the build sets for alpha.
double relaxation_time(const double alpha) {
	return 4.0*expected_diffusivity(alpha)+0.5;
}

/// The wave's amplitude: the magnitude of its Fourier component over all cells (sine and cosine parts, so a wave
/// carried with the flow is measured wherever it stands).
double amplitude(LBM& lbm, const double k, const bool along_x) {
	lbm.update_fields();
	lbm.T.read_from_device();
	double s = 0.0, c = 0.0;
	ulong cells = 0ull;
	for(ulong n=0ull; n<lbm.get_N(); n++) {
		uint x=0u, y=0u, z=0u;
		lbm.coordinates(n, x, y, z);
		const double phase = k*(double)(along_x ? x : y);
		const double dT = (double)lbm.T[n]-1.0;
		s += dT*std::sin(phase);
		c += dT*std::cos(phase);
		cells++;
	}
	return 2.0*std::sqrt(s*s+c*c)/(double)cells;
}

/// A wave of amplitude A along x or y in a periodic box in uniform motion u (along x), run from t1 to t2.
WaveDecay measure(const float alpha, const float u, const bool along_x, const ulong t1, const ulong t2) {
	const uint N = 64u, Nz = 8u;
	const double k = 2.0*pi/(double)N; // pi: the macro of utilities.hpp
	const float A = 0.01f;
	LBM lbm(along_x ? N : 4u, along_x ? 4u : N, Nz, 1.0E-4f, 0.0f, 0.0f, 0.0f, 0.0f, alpha, 0.0f);
	for(ulong n=0ull; n<lbm.get_N(); n++) {
		uint x=0u, y=0u, z=0u;
		lbm.coordinates(n, x, y, z);
		lbm.u.x[n] = u; // the uniform flow (periodic box: it stays)
		lbm.T[n] = 1.0f+A*(float)std::sin(k*(double)(along_x ? x : y));
	}
	lbm.run(t1);
	WaveDecay d;
	d.amplitude1 = amplitude(lbm, k, along_x);
	lbm.run(t2-t1);
	d.amplitude2 = amplitude(lbm, k, along_x);
	d.diffusivity = std::log(d.amplitude1/d.amplitude2)/(k*k*(double)(t2-t1));
	return d;
}

/// Steps so that the wave decays by about a factor e between t1 and t2 (D k^2 (t2 - t1) = 1), at least 1000.
ulong steps_for(const double alpha) {
	const double k = 2.0*pi/64.0;
	const double D = expected_diffusivity(alpha);
	const double steps = 1.0/(D*k*k);
	return steps < 1000.0 ? 1000ull : (ulong)steps;
}

/// Reports one case: the measured diffusivity against the expected one.
bool check(const char* name, const WaveDecay& d, const double expected, const double limit) {
	std::printf("\n%s: D %.5e, expected %.5e, ratio %.4f (A %.3e -> %.3e)\n", name, d.diffusivity, expected, d.diffusivity/expected, d.amplitude1, d.amplitude2);
	return PhysicsCheck::report(name, std::fabs(d.diffusivity/expected-1.0), limit);
}

} // namespace

void main_setup() { // extensions: TEMPERATURE
	const float alpha_small = 2.0E-3f, alpha_large = 0.125f, u = 0.1f;
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
	for(const float alpha : { alpha_small, alpha_large }) {
		const double expected = expected_diffusivity((double)alpha);
		const ulong t2 = steps_for((double)alpha), t1 = t2/10ull;
		std::printf("\nalpha %.4e: tauT %.5f, steps %llu to %llu\n", alpha, relaxation_time((double)alpha), (unsigned long long)t1, (unsigned long long)t2);
		char name[128];
		std::snprintf(name, sizeof(name), "thermal diffusivity at rest, alpha %.0e", alpha);
		check(name, measure(alpha, 0.0f, false, t1, t2), expected, 0.01);
		std::snprintf(name, sizeof(name), "thermal diffusivity across a flow u %.2f, alpha %.0e", u, alpha);
		check(name, measure(alpha, u, false, t1, t2), expected, 0.01);
		std::snprintf(name, sizeof(name), "thermal diffusivity along a flow u %.2f, alpha %.0e", u, alpha);
		const double galilean = expected-(double)u*(double)u*(relaxation_time((double)alpha)-0.5); // the linear equilibrium's error along the flow
		std::printf("(along the flow the lattice's diffusion is D - u^2 (tauT - 1/2) = %.5e)\n", galilean);
		check(name, measure(alpha, u, true, t1, t2), expected, 0.05);
	}
}
