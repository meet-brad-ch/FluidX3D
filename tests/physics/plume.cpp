// A turbulent plume from a heated patch on the floor of a still, neutral box: the
// centreline rise and the width against the plume laws of Morton, Taylor and Turner 1956 (the entrainment hypothesis):
// for a source of buoyancy flux B the time-mean centreline velocity falls as w_c = C_w (B / z)^(1/3) and the width grows
// as b = C_b z above the virtual origin; Gaussian profiles give C_w about 4.7 (Chen and Rodi 1980: 4.7, Rouse et al.
// 1952: 4.7) and C_b about 0.10 to 0.12 (the e-folding radius; the entrainment coefficient alpha 0.08 to 0.13).
// Setup: the floor TYPE_S, its first fluid layer heated inside a disc of radius r0 (TYPE_Y, heat_rate per step), the
// lateral walls periodic and far, the ceiling TYPE_S (adiabatic); the buoyancy by a tiny fz and a large beta
// (f = fz (1 - beta (T - 1))), so B = |fz| beta heat_rate N_patch (lattice units); SUBGRID keeps the plume stable at its
// Reynolds number (w_c b / nu about 3e3); THERMAL_DIFFUSIVITY_FIX. The molecular diffusivity is 0.014 (tauT 0.556),
// not air's nu / 0.71: at tauT 0.503 the thermal lattice creates heat and negative temperatures in a buoyant plume
// (tests/physics/heat_conservation.cpp), and at 0.528 it still undershoots 12 % at the sharp source, the cold fluid sinks and
// stirs the whole box; at 0.556 the undershoot is 2.5 % and the plume carries the source's heat flux. A turbulent
// plume's laws do not depend on the molecular Prandtl number. The source disc has r0 = 12 (B four times the first
// runs', the plume Reynolds number about 3000: at r0 = 6 the plume stayed laminar, w_c constant with height), and the
// start carries random velocity noise of 0.01 to trigger the transition.
// The box has no heat sink, so it is a filling box (Baines and Turner 1969): the plume's outflow spreads under the
// ceiling and its front descends at Q(z) / A, Q the plume's volume flux; the plume laws hold only below the front. With
// A = 256 x 256 and B as below the front passes z = 200 after about 33 000 steps, so the fields are time-averaged from
// step 12 000 (the plume established over the whole height) to 32 000, and the fit runs from 2.5 r0 to z = 180.
// Per level of the time-mean w: w_c = its maximum within N/4 of the patch axis, b = the mean e-folding radius of the
// mean w around that maximum over the 4 axis directions. Fits by least squares in log-log over the range: the exponent
// of w_c in z (expected -1/3), of b in z (expected 1), with the virtual origin from the width's line. Passes when the w
// exponent is within 0.05 of -1/3, the width exponent within 0.1 of 1, and C_w and C_b within the published ranges
// (C_w 3.9 to 5.5, C_b 0.08 to 0.14). Writes bin/plume_profile.csv (z, w_c, b) for a chart, bin/plume_section.csv (x, z, mean w, mean T at y = N/2) and bin/plume_plane.csv (x, y, mean w at
// z = 100) to see the plume itself.

#include "setup/setup.hpp"
#include "physics_check.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

namespace {

/// The plume at one height, from the time-mean fields.
struct PlumeLevel {
	double w_centre = 0.0; ///< the maximum of the mean w near the axis
	double width = 0.0;    ///< the e-folding radius of the mean w around that maximum (0 when not found in all 4 directions)
};

/// A least-squares line log(y) = a + p log(x) over the points.
struct PowerFit {
	double exponent = 0.0;
	double prefactor = 0.0;
};

PowerFit fit_power(const std::vector<double>& x, const std::vector<double>& y) {
	double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
	const double n = (double)x.size();
	for(size_t i=0; i<x.size(); i++) {
		const double lx = std::log(x[i]), ly = std::log(y[i]);
		sx += lx; sy += ly; sxx += lx*lx; sxy += lx*ly;
	}
	PowerFit f;
	f.exponent = (n*sxy-sx*sy)/(n*sxx-sx*sx);
	f.prefactor = std::exp((sy-f.exponent*sx)/n);
	return f;
}

/// The plume at one level of the time-mean w field (N x N plane at z, row-major y then x).
PlumeLevel level_of(const std::vector<double>& mean_w, const uint N, const uint z, const uint cx, const uint cy) {
	const auto at = [&](const uint x, const uint y) { return mean_w[((size_t)z*N+y)*N+x]; };
	PlumeLevel l;
	uint xm = cx, ym = cy;
	for(uint y=cy-N/4u; y<=cy+N/4u; y++) for(uint x=cx-N/4u; x<=cx+N/4u; x++) {
		if(at(x, y)>l.w_centre) { l.w_centre = at(x, y); xm = x; ym = y; }
	}
	double radii = 0.0; int found = 0;
	const int dirs[4][2] = { { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 } };
	for(const auto& d : dirs) {
		for(int r=1; r<(int)N/2; r++) {
			const int x = ((int)xm+d[0]*r+(int)N)%(int)N, y = ((int)ym+d[1]*r+(int)N)%(int)N;
			const double w = at((uint)x, (uint)y), w_prev = at((uint)(((int)xm+d[0]*(r-1)+(int)N)%(int)N), (uint)(((int)ym+d[1]*(r-1)+(int)N)%(int)N));
			const double target = l.w_centre/2.718281828;
			if(w<target) { radii += (double)(r-1)+(w_prev-target)/(w_prev-w); found++; break; } // linear crossing
		}
	}
	if(found==4) l.width = radii/4.0;
	return l;
}

} // namespace

void main_setup() { // extensions: VOLUME_FORCE TEMPERATURE THERMAL_DIFFUSIVITY_FIX SUBGRID
	const uint N = 256u, Nz = 256u, r0 = 12u;
	const float nu = 5.0E-4f, fz = -1.0E-6f, beta = 2000.0f; // |fz| beta = 2e-3 per unit T
	const float heat_rate = 1.0E-3f; // per step per patch cell
	const ulong steps = 32000ull, sample = 250ull, average_from = 12000ull;
	const uint z_fit_top = 180u; // below the filling-box front at the end of the window
	// the thermal settings: Pr_t = 1/3 as the CBL, alpha 0.014 (tauT 0.556, see the file's comment: heat_conservation.cpp shows
	// heat creation and negative T at tauT 0.503, and the plume a 12 % undershoot at 0.528). A diagnostic run writes "<1/Pr_t> <alpha>" into plume_settings.txt in
	// its working folder (the command line is FluidX3D's device list, so it cannot carry them); the outputs get a tag.
	float inv_prandtl_t = 3.0f, alpha = 0.014f; // tauT 0.556
	std::string tag;
	{
		std::ifstream setting("plume_settings.txt");
		float value_prt = 0.0f, value_alpha = 0.0f;
		if(setting>>value_prt>>value_alpha) {
			inv_prandtl_t = value_prt; alpha = value_alpha;
			tag = "_iprt"+std::to_string((int)std::lround(10.0f*value_prt))+"_alpha"+std::to_string((int)std::lround(1.0E4f*value_alpha));
		}
	}
	LBM lbm(N, N, Nz, nu, 0.0f, 0.0f, fz, 0.0f, alpha, beta);
	const uint cx = N/2u, cy = N/2u;
	ulong patch = 0ull;
	uint seed = 12345u;
	for(ulong n=0ull; n<lbm.get_N(); n++) {
		uint x=0u, y=0u, z=0u;
		lbm.coordinates(n, x, y, z);
		lbm.T[n] = 1.0f;
		if(z>0u&&z<Nz-1u) { // the start noise (+-0.005) in the fluid only: a TYPE_S cell with a velocity would be a moving wall
			seed = 1664525u*seed+1013904223u; lbm.u.x[n] = 0.01f*((float)(seed>>8)/16777216.0f-0.5f);
			seed = 1664525u*seed+1013904223u; lbm.u.y[n] = 0.01f*((float)(seed>>8)/16777216.0f-0.5f);
			seed = 1664525u*seed+1013904223u; lbm.u.z[n] = 0.01f*((float)(seed>>8)/16777216.0f-0.5f);
		}
		if(z==0u||z==Nz-1u) lbm.flags[n] = TYPE_S;
		else if(z==1u&&sq((float)x-(float)cx)+sq((float)y-(float)cy)<=sq((float)r0)) { lbm.flags[n] = TYPE_Y; patch++; }
	}
	lbm.set_heat_rate(heat_rate);
	lbm.set_inv_prandtl_t(inv_prandtl_t);
	const double B = (double)(-fz)*(double)beta*(double)heat_rate*(double)patch; // the buoyancy flux
	std::printf("\nplume: box %u x %u x %u, patch %llu cells (r0 %u), B %.4e, nu %.1e, alpha %.2e, 1/Pr_t %.1f, %llu steps, mean over %llu..%llu every %llu\n",
	            N, N, Nz, (unsigned long long)patch, r0, B, nu, alpha, inv_prandtl_t, (unsigned long long)steps, (unsigned long long)average_from, (unsigned long long)steps, (unsigned long long)sample);

	std::vector<double> mean_w(lbm.get_N(), 0.0), mean_T(lbm.get_N(), 0.0);
	std::vector<double> flux_total(Nz, 0.0); // per level: the sample mean of sum_xy w (T - T_plane), the plume's heat flux with its turbulent part
	int samples = 0;
	for(ulong t=0ull; t<steps; t+=sample) {
		lbm.run(sample);
		if(t+sample<average_from) continue;
		lbm.update_fields();
		lbm.u.read_from_device();
		lbm.T.read_from_device();
		for(ulong n=0ull; n<lbm.get_N(); n++) { mean_w[n] += (double)lbm.u.z[n]; mean_T[n] += (double)lbm.T[n]; }
		for(uint z=0u; z<Nz; z++) {
			double sw = 0.0, sT = 0.0, swT = 0.0;
			for(uint y=0u; y<N; y++) for(uint xp=0u; xp<N; xp++) { const ulong n = lbm.index(xp, y, z); const double w = (double)lbm.u.z[n], T = (double)lbm.T[n]; sw += w; sT += T; swT += w*T; }
			flux_total[z] += swT-sw*sT/(double)((ulong)N*N);
		}
		samples++;
		if((t+sample)%(steps/10ull)==0ull) std::printf("  step %llu (%d samples)\n", (unsigned long long)(t+sample), samples);
	}
	for(double& w : mean_w) w /= (double)samples;
	for(double& T : mean_T) T /= (double)samples;
	{ // the undershoot: the lowest instantaneous T of a fluid cell at the last sample (the lattice's dispersion at the sharp source)
		float lowest = 1.0E9f, highest = -1.0E9f;
		for(ulong n=0ull; n<lbm.get_N(); n++) { if(!(lbm.flags[n]&TYPE_S)) { lowest = std::fmin(lowest, lbm.T[n]); highest = std::fmax(highest, lbm.T[n]); } }
		std::printf("  last sample: lowest T %.4f, highest T %.4f (start 1)\n", lowest, highest);
	}
	{ // the mean fields to look at: the vertical section through the axis and the plane at z = 100
		std::ofstream section("plume_section"+tag+".csv");
		section << "x,z,w,T" << std::endl;
		for(uint z=0u; z<Nz; z++) for(uint x=0u; x<N; x++) { const ulong n = lbm.index(x, cy, z); section << x << "," << z << "," << mean_w[n] << "," << mean_T[n] << std::endl; }
		std::ofstream plane("plume_plane"+tag+".csv");
		plane << "x,y,w" << std::endl;
		for(uint y=0u; y<N; y++) for(uint xp=0u; xp<N; xp++) plane << xp << "," << y << "," << mean_w[lbm.index(xp, y, 100u)] << std::endl;
	}
	std::ofstream csv("plume_profile"+tag+".csv");
	csv << "z,w_centre,width,flux_total,flux_mean,T_ambient" << std::endl; // flux: sum_xy w (T - T_plane) per step, against B_source = heat_rate x patch
	std::vector<double> zs, ws, bs;
	const double z_lo = 2.5*(double)r0, z_hi = (double)z_fit_top;
	for(uint z=2u; z<Nz-1u; z++) {
		const PlumeLevel l = level_of(mean_w, N, z, cx, cy);
		double sw = 0.0, sT = 0.0, swT = 0.0;
		for(uint y=0u; y<N; y++) for(uint xp=0u; xp<N; xp++) { const ulong n = lbm.index(xp, y, z); sw += mean_w[n]; sT += mean_T[n]; swT += mean_w[n]*mean_T[n]; }
		const double T_plane = sT/(double)((ulong)N*N), flux_mean = swT-sw*T_plane;
		csv << z << "," << l.w_centre << "," << l.width << "," << flux_total[z]/(double)samples << "," << flux_mean << "," << T_plane << std::endl;
		if((double)z>=z_lo&&(double)z<=z_hi&&l.width>0.0) { zs.push_back((double)z); ws.push_back(l.w_centre); bs.push_back(l.width); }
	}
	csv.close();
	// the virtual origin from the width (b = C_b (z - z0)): a line fit b over z
	double sz = 0.0, sb = 0.0, szz = 0.0, szb = 0.0;
	for(size_t i=0; i<zs.size(); i++) { sz += zs[i]; sb += bs[i]; szz += zs[i]*zs[i]; szb += zs[i]*bs[i]; }
	const double n = (double)zs.size();
	const double C_b = (n*szb-sz*sb)/(n*szz-sz*sz), z0 = -((sb-C_b*sz)/n)/C_b;
	std::vector<double> zv;
	for(const double z : zs) zv.push_back(z-z0);
	const PowerFit fw = fit_power(zv, ws), fb = fit_power(zv, bs);
	const double C_w = fw.prefactor/std::cbrt(B); // w_c = C_w B^(1/3) (z - z0)^(-1/3)
	std::printf("\nplume fits over z %.0f..%.0f (%zu levels), virtual origin z0 %.1f: w_c exponent %.3f (expected -0.333), C_w %.2f (B^(1/3) %.4f; expected 3.9-5.5); width exponent %.3f (expected 1), C_b %.3f (expected 0.08-0.14)\n",
	            z_lo, z_hi, zs.size(), z0, fw.exponent, C_w, std::cbrt(B), fb.exponent, C_b);
	PhysicsCheck::report("plume: centreline w exponent -1/3 (+-0.05)", std::fabs(fw.exponent+1.0/3.0)/(1.0/3.0), 0.15);
	PhysicsCheck::report("plume: width exponent 1 (+-0.1)", std::fabs(fb.exponent-1.0), 0.1);
	PhysicsCheck::report("plume: C_w in 3.9-5.5", C_w<3.9||C_w>5.5 ? 1.0 : 0.0, 0.5);
	PhysicsCheck::report("plume: C_b in 0.08-0.14", C_b<0.08||C_b>0.14 ? 1.0 : 0.0, 0.5);
}
