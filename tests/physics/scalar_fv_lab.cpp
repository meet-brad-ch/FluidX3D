// The finite-volume scalar scheme (scalar_fv_reference.hpp, ProLB's) on the host, in a decaying temperature wave:
// T = 1 + A sin(k x) in a periodic 64 x 4 x 4 box, carried along
// x by a uniform u, decays as exp(-D k^2 t). The SCALAR_FV build of thermal_diffusivity measured along u = 0.1 a
// diffusivity short by about u^2 dt/2 = 5e-3 (9.2e-5 instead of 2e-3; 0.120 instead of 0.125): the explicit Euler step
// adds -u^2 dt/2 to an advected field (hypothesis from the measured shortfall). Feng et al. 2021 (anelastic LB, eq 36)
// take a second-order midpoint step. This lab measures D along the flow for both time steps at alpha 2e-3, 0.014 (the CBL
// example's) and 0.125, and at rest.
// Passes when the midpoint step gives D within 5 % of alpha along the flow for alpha >= 0.014 (the
// tauT floor 0.556 of the thermal tests). Measured first (2026-10-26): at alpha 2e-3 (cell Peclet 50) the limiter adds about 6e-4
// (D/alpha 1.30), the known diffusion of a limited scheme at the wave's extrema; that row is printed as the limit.

#include "setup/setup.hpp"
#include "physics_check.hpp"
#include "scalar_fv_reference.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

namespace {

/// The measured diffusivity of one case.
struct WaveResult {
	double D{};      ///< the measured diffusivity (lattice units)
	double ratio{};  ///< D / alpha
};

/// @brief Runs the wave and measures D from the amplitude of mode k at two times.
WaveResult run_wave(const double alpha, const float u, const bool rk2) {
	const uint L{ 64u }, W{ 4u };
	const ulong N{ (ulong)L*W*W };
	const double k{ 2.0*3.14159265358979/(double)L }, A{ 0.01 };
	std::vector<float> start(N, 0.0f);
	for(ulong n{}; n<N; n++) start[n] = (float)(1.0+A*std::sin(k*(double)(n%L)));
	FiniteVolumeScalar field(L, W, W, std::vector<bool>(N, false), start);
	field.use_rk2(rk2);
	std::vector<float> velocity(3ull*N, 0.0f), D(N, (float)alpha);
	for(ulong n{}; n<N; n++) velocity[n] = u;
	const auto amplitude = [&]() {
		double a{}, b{};
		for(ulong n{}; n<N; n++) { const double x{ (double)(n%L) }; a += (field.at(n)-1.0)*std::sin(k*x); b += (field.at(n)-1.0)*std::cos(k*x); }
		return 2.0/(double)N*std::sqrt(a*a+b*b);
	};
	const ulong t1{ (ulong)(0.1/(alpha*k*k)) }, t2{ (ulong)(1.0/(alpha*k*k)) }; // decay from about e^-0.1 to e^-1
	double A1{};
	for(ulong t{ 1ull }; t<=t2; t++) {
		field.step(velocity.data(), D.data());
		if(t==t1) A1 = amplitude();
	}
	const double A2{ amplitude() };
	WaveResult r{};
	r.D = std::log(A1/A2)/(k*k*(double)(t2-t1));
	r.ratio = r.D/alpha;
	return r;
}

} // namespace

void main_setup() { // extensions: TEMPERATURE (the host scheme only; no GPU lattice is built)
	const double alphas[] = { 2.0E-3, 0.014, 0.125 };
	double worst_rk2{};
	std::printf("\nfinite-volume lab: a wave T = 1 + 0.01 sin(2 pi x / 64), D measured from its decay (D / alpha: 1 is right)\n");
	for(const double alpha : alphas) {
		const WaveResult rest_euler{ run_wave(alpha, 0.0f, false) }, along_euler{ run_wave(alpha, 0.1f, false) }, along_rk2{ run_wave(alpha, 0.1f, true) };
		std::printf("  alpha %.3e: at rest D/alpha %.4f; along u 0.1: Euler %.4f (D %.3e, short %.2e), midpoint %.4f (D %.3e)\n",
		            alpha, rest_euler.ratio, along_euler.ratio, along_euler.D, alpha-along_euler.D, along_rk2.ratio, along_rk2.D);
		if(alpha>=0.014) worst_rk2 = std::fmax(worst_rk2, std::fabs(along_rk2.ratio-1.0)); // the CBL range
	}
	PhysicsCheck::report("finite-volume lab: the midpoint step diffuses with alpha along the flow, alpha >= 0.014 (relative)", worst_rk2, 0.05);
}
