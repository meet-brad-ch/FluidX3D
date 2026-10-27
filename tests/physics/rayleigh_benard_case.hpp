// One Rayleigh-Benard case of air (Pr = 0.71) between a hot floor and a cold ceiling, periodic sideways, shared by the
// physics tests rayleigh_benard (onset and Nu(Ra)) and rayleigh_benard_grid (Nu against the cell count).
// Lattice: THERMAL_DIFFUSIVITY_FIX (alpha is the diffusivity); the floor and ceiling TYPE_S (no slip, the wall half a
// cell outside them), the temperature fixed one layer in (TYPE_T, 1 + dT/2 and 1 - dT/2, dT = 1; a TYPE_T cell is a
// moving fluid cell whose T is reset every step), H = the distance between the fixed-temperature planes (Nz - 3); the
// buoyancy by a tiny constant force fz and a large beta (f = fz (1 - beta (T - 1)), so the hydrostatic ramp is
// negligible): g beta dT = Ra nu alpha / H^3; nu fixed at 0.005 (tau 0.515), so the free-fall speed falls with H.
// Writes bin/<prefix>_nu.csv (step, Nu) and bin/<prefix>_section.csv (x, z, T, w at y = Ny/2) for charts.
#pragma once

#include "setup/setup.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

namespace rayleigh_benard {

/// The lattice setup of one Rayleigh number.
struct RayleighCase {
	double Ra = 0.0;           ///< the Rayleigh number g beta dT H^3 / (nu alpha)
	uint H = 32u;              ///< cells between the fixed-temperature planes
	double aspect = 2.016;     ///< box width over H (2.016: the critical wavelength)
	ulong steps = 0ull;        ///< steps in all; Nu averaged over the second half
	double perturbation = 0.0; ///< the start's temperature perturbation amplitude (0: a random one of 1e-3)
};

/// What one case measured.
struct RayleighResult {
	double nusselt = 0.0;  ///< the mean over the second half
	double spread = 0.0;   ///< the standard deviation of the samples in that half
	double max_speed = 0.0;///< the largest |u| seen in the last sample
};

/// Nu = 1 + <w (T - <T>)> H / (D dT) over the fluid cells (not the fixed-temperature or solid ones).
inline double nusselt_of(LBM& lbm, const double D, const double dT, const uint H) {
	lbm.update_fields();
	lbm.T.read_from_device();
	lbm.u.read_from_device();
	double sumT = 0.0; ulong fluid = 0ull;
	for(ulong n=0ull; n<lbm.get_N(); n++) { if(lbm.flags[n]==0u) { sumT += (double)lbm.T[n]; fluid++; } }
	const double Tmean = sumT/(double)fluid;
	double flux = 0.0;
	for(ulong n=0ull; n<lbm.get_N(); n++) { if(lbm.flags[n]==0u) flux += (double)lbm.u.z[n]*((double)lbm.T[n]-Tmean); }
	flux /= (double)fluid;
	return 1.0+flux*(double)H/(D*dT);
}

/// The vertical section at y = Ny/2 (x, z, T, w) for the chart.
inline void write_section(LBM& lbm, const std::string& path) {
	std::ofstream f(path);
	f << "x,z,T,w\n";
	const uint y = lbm.get_Ny()/2u;
	for(uint z=0u; z<lbm.get_Nz(); z++) for(uint x=0u; x<lbm.get_Nx(); x++) {
		const ulong n = lbm.index(x, y, z);
		f << x << "," << z << "," << lbm.T[n] << "," << lbm.u.z[n] << "\n";
	}
}

/// Runs one case and writes <prefix>_nu.csv and <prefix>_section.csv.
inline RayleighResult run_case(const RayleighCase& c, const std::string& prefix) {
	const double Pr = 0.71;
	const float nu = 0.005f; // tau 0.515
	const double alpha = (double)nu/Pr; // the diffusivity (THERMAL_DIFFUSIVITY_FIX)
	const double dT = 1.0;
	const double H3 = (double)c.H*(double)c.H*(double)c.H;
	const double g_beta_dT = c.Ra*(double)nu*alpha/H3; // the buoyancy acceleration scale per unit dT
	const float fz = -1.0E-6f; // the constant part (hydrostatic): negligible ramp
	const float beta = (float)(g_beta_dT/(-(double)fz*dT)); // f = fz (1 - beta (T - 1)): buoyancy -fz beta (T - 1) up for warm cells
	const uint Nz = c.H+3u, L = (uint)std::lround(c.aspect*(double)c.H);
	std::printf("\nRa %.3g: H %u, box %u x %u x %u, nu %.4f, alpha %.5f, g beta dT %.3e, free-fall speed %.4f, beta %.1f, steps %llu\n",
	            c.Ra, c.H, L, L, Nz, nu, alpha, g_beta_dT, std::sqrt(g_beta_dT*(double)c.H), beta, (unsigned long long)c.steps);
	LBM lbm(L, L, Nz, nu, 0.0f, 0.0f, fz, 0.0f, (float)alpha, beta);
	uint seed = 12345u;
	for(ulong n=0ull; n<lbm.get_N(); n++) {
		uint x=0u, y=0u, z=0u;
		lbm.coordinates(n, x, y, z);
		seed = 1664525u*seed+1013904223u;
		const float noise = ((float)(seed>>8)/16777216.0f-0.5f)*2.0f; // -1..1
		const float amplitude = c.perturbation>0.0 ? (float)c.perturbation : 1.0E-3f;
		const float conduction = 1.0f+(float)dT*(0.5f-((float)z-1.0f)/(float)(c.H)); // linear between the planes z = 1 and z = Nz-2
		if(z==0u||z==Nz-1u) { lbm.flags[n] = TYPE_S; lbm.T[n] = conduction; }
		else if(z==1u) { lbm.flags[n] = TYPE_T; lbm.T[n] = 1.0f+0.5f*(float)dT; }
		else if(z==Nz-2u) { lbm.flags[n] = TYPE_T; lbm.T[n] = 1.0f-0.5f*(float)dT; }
		else lbm.T[n] = conduction+amplitude*noise;
	}
	const ulong sample = 1000ull;
	std::ofstream csv(prefix+"_nu.csv");
	csv << "step,nusselt\n";
	std::vector<double> second_half;
	RayleighResult r;
	for(ulong t=0ull; t<c.steps; t+=sample) {
		lbm.run(sample);
		const double Nu = nusselt_of(lbm, alpha, dT, c.H);
		csv << (t+sample) << "," << Nu << "\n";
		if(t+sample>c.steps/2ull) second_half.push_back(Nu);
		if((t+sample)%(c.steps/10ull)==0ull) std::printf("  step %llu: Nu %.4f\n", (unsigned long long)(t+sample), Nu);
	}
	csv.close();
	double s = 0.0, s2 = 0.0;
	for(const double Nu : second_half) { s += Nu; s2 += Nu*Nu; }
	r.nusselt = s/(double)second_half.size();
	r.spread = std::sqrt(std::fmax(0.0, s2/(double)second_half.size()-r.nusselt*r.nusselt));
	for(ulong n=0ull; n<lbm.get_N(); n++) r.max_speed = std::fmax(r.max_speed, (double)std::sqrt(lbm.u.x[n]*lbm.u.x[n]+lbm.u.y[n]*lbm.u.y[n]+lbm.u.z[n]*lbm.u.z[n]));
	write_section(lbm, prefix+"_section.csv");
	std::printf("  Nu %.4f +- %.4f over the second half, max |u| %.4f\n", r.nusselt, r.spread, r.max_speed);
	return r;
}

/// Kerr 1996 (J. Fluid Mech. 310): Nu = 0.186 Ra^0.276 (Pr 0.7, 5e4 <= Ra <= 2e7).
inline double kerr_1996(const double Ra) { return 0.186*std::pow(Ra, 0.276); }

} // namespace rayleigh_benard
