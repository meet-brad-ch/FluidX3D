// The D3Q7 scalar scheme in prescribed flows, on the host only: which part of
// a flow makes a uniform scalar non-uniform? The GPU's lattice equals the host reference bit for bit
// (scalar_reference.cpp), so the host reference (scalar_reference_lattice.hpp) is the scheme under study.
//
// The flows: 2-D Taylor-Green cells, u = U a(t) sin(k x) cos(k y), v = -U a(t) cos(k x) sin(k y), w = 0, periodic,
// no walls. Their central-difference divergence on the lattice is exactly 0 (the D3Q7 stencil sees no compression),
// so a drift here comes from the scheme's other terms. The cases vary one parameter each against a base case
// (U 0.03, one cell pair across 64 cells, tauT 0.556 as in the CBL example): the speed, the wavenumber, tauT, and the time
// dependence a(t) (steady, ramped, oscillating). A uniform translation is the control (exactly no drift).
//
// Reported per case: the largest |dQ/Q| after the steps, the same at half the steps, and the correlation of the
// final dQ/Q with the field d_i d_j (u_i u_j) (central differences), the second-order term a linear equilibrium
// leaves in the recovered equation (hypothesis to test, not a cited result). Passes when the control has no drift
// (the lab works); the scaling numbers are the result.

#include "setup/setup.hpp"
#include "physics_check.hpp"
#include "scalar_reference_lattice.hpp"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace {

/// One prescribed flow.
struct LabCase {
	std::string name;
	float U = 0.03f;            ///< lattice speed amplitude
	uint pairs = 1u;            ///< Taylor-Green cell pairs across the box (k = 2 pi pairs / L)
	float alpha = 0.014f;       ///< lattice diffusivity, wT = 1 / (4 alpha + 1/2) (THERMAL_DIFFUSIVITY_FIX)
	uint time_mode = 0u;        ///< 0 steady, 1 ramp a = t / steps, 2 oscillating a = sin(2 pi t / period), 3 uniform translation (control)
	float period = 400.0f;      ///< steps, for the oscillating case
};

/// What one case did.
struct LabResult {
	double drift_half = 0.0; ///< the largest |dQ/Q| at half the steps
	double drift_end = 0.0;  ///< the same at the end
	double corr_uu = 0.0;    ///< the correlation of the final dQ/Q with d_i d_j (u_i u_j)
	double rms_uu = 0.0;     ///< the rms of d_i d_j (u_i u_j) at the end (for the scaling)
};

/// @brief Runs one case on the host reference and measures the drift of a uniform Q.
LabResult run_case(const LabCase& c, const uint L, const ulong steps) {
	const uint Nz = 4u;
	const ulong N = (ulong)L*L*Nz;
	const float Q0 = 3.6f; // as 26 g/kg with def_q_scale 0.01
	ScalarReference lattice(L, L, Nz, std::vector<bool>(N, false), std::vector<float>(N, Q0), 1u);
	std::vector<float> u(3ull*N, 0.0f), w(N, 1.0f/(4.0f*c.alpha+0.5f));
	const double k = 2.0*3.14159265358979*(double)c.pairs/(double)L;
	const auto index = [&](const uint x, const uint y, const uint z) { return (ulong)x+((ulong)y+(ulong)z*L)*L; };
	const auto amplitude = [&](const ulong t) {
		if(c.time_mode==1u) return (double)t/(double)steps;
		if(c.time_mode==2u) return std::sin(2.0*3.14159265358979*(double)t/(double)c.period);
		return 1.0;
	};
	const auto set_flow = [&](const ulong t) {
		const double a = amplitude(t);
		for(uint z=0u; z<Nz; z++) for(uint y=0u; y<L; y++) for(uint x=0u; x<L; x++) {
			const ulong n = index(x, y, z);
			if(c.time_mode==3u) { u[n] = c.U; u[N+n] = 0.5f*c.U; u[2ull*N+n] = 0.0f; continue; }
			u[n] = (float)(c.U*a*std::sin(k*(double)x)*std::cos(k*(double)y));
			u[N+n] = (float)(-c.U*a*std::cos(k*(double)x)*std::sin(k*(double)y));
			u[2ull*N+n] = 0.0f;
		}
	};
	const auto largest_drift = [&]() {
		double d = 0.0;
		for(ulong n=0ull; n<N; n++) d = std::fmax(d, std::fabs(((double)lattice.at(n)-(double)Q0)/(double)Q0));
		return d;
	};
	LabResult r;
	for(ulong t=1ull; t<=steps; t++) {
		lattice.stream();
		if(t==steps/2ull) r.drift_half = largest_drift();
		set_flow(t);
		lattice.collide(u.data(), w.data());
	}
	r.drift_end = largest_drift();
	// d_i d_j (u_i u_j) by central differences (2-D: i, j in x, y), against the final dQ/Q
	const auto uu = [&](const uint x, const uint y, const uint i, const uint j) { const ulong n = index(x%L, y%L, 0u); return (double)u[(ulong)i*N+n]*(double)u[(ulong)j*N+n]; };
	double sxy = 0.0, sxx = 0.0, syy = 0.0;
	for(uint y=0u; y<L; y++) for(uint x=0u; x<L; x++) {
		const uint xp = x+1u, xm = x+L-1u, yp = y+1u, ym = y+L-1u;
		const double dxx = uu(xp, y, 0u, 0u)-2.0*uu(x, y, 0u, 0u)+uu(xm, y, 0u, 0u);
		const double dyy = uu(x, yp, 1u, 1u)-2.0*uu(x, y, 1u, 1u)+uu(x, ym, 1u, 1u);
		const double dxy = 0.25*(uu(xp, yp, 0u, 1u)-uu(xp, ym, 0u, 1u)-uu(xm, yp, 0u, 1u)+uu(xm, ym, 0u, 1u));
		const double f = dxx+dyy+2.0*dxy;
		const double q = ((double)lattice.at(index(x, y, 0u))-(double)Q0)/(double)Q0;
		sxy += f*q; sxx += f*f; syy += q*q;
	}
	r.corr_uu = (sxx>0.0&&syy>0.0) ? sxy/std::sqrt(sxx*syy) : 0.0;
	r.rms_uu = std::sqrt(sxx/((double)L*L));
	return r;
}

} // namespace

void main_setup() { // extensions: TEMPERATURE (the host reference only; no GPU lattice is built)
	const uint L = 64u;
	const ulong steps = 2000ull;
	const LabCase cases[] = {
		{ "control: uniform translation (0.03, 0.015)", 0.03f, 1u, 0.014f, 3u },
		{ "base: steady, U 0.03, 1 pair, tauT 0.556", 0.03f, 1u, 0.014f, 0u },
		{ "U halved (0.015)", 0.015f, 1u, 0.014f, 0u },
		{ "2 pairs (k doubled)", 0.03f, 2u, 0.014f, 0u },
		{ "tauT 0.612 (tauT - 1/2 doubled)", 0.03f, 1u, 0.028f, 0u },
		{ "ramp a = t / steps", 0.03f, 1u, 0.014f, 1u },
		{ "oscillating a = sin(2 pi t / 400)", 0.03f, 1u, 0.014f, 2u, 400.0f },
		{ "oscillating, tauT 0.612", 0.03f, 1u, 0.028f, 2u, 400.0f },
	};
	std::printf("\nscalar scheme lab: Taylor-Green cells in a %u x %u x 4 periodic box, %llu steps, uniform Q 3.6\n", L, L, (unsigned long long)steps);
	double control = 0.0;
	for(const LabCase& c : cases) {
		const LabResult r = run_case(c, L, steps);
		std::printf("  %-44s max |dQ/Q| %.3e at half, %.3e at the end; correlation with d_i d_j (u_i u_j) %+.3f (its rms %.2e)\n",
		            c.name.c_str(), r.drift_half, r.drift_end, r.corr_uu, r.rms_uu);
		if(c.time_mode==3u) control = r.drift_end;
	}
	PhysicsCheck::report("scalar scheme lab: the control (uniform translation) keeps Q uniform (|dQ/Q| over 1e-6)", control/1.0E-6, 1.0);
}
