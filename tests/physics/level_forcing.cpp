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
// 3. The temperature: T = 1.1 in the upper half, T_ref = 1 there at r = 1/500 (0 below); after 1500 steps the levels in
//    the middle of the upper half hold 1 + 0.1 e^-3 within 2 % (diffusion with alpha 1e-3 reaches 1.7 cells from the
//    step in that time; the middle is far from it), the lower half stays at 1 within 1e-3.

#include "setup/setup.hpp"
#include "physics_check.hpp"

#include <cmath>
#include <cstdio>
#include <vector>

namespace {

/// The layer means of ux and w and T of one LBM over its fluid cells.
struct LayerMeans {
	std::vector<double> ux, uz, T;
};

LayerMeans layer_means(LBM& lbm) {
	lbm.update_fields();
	lbm.u.read_from_device();
	lbm.T.read_from_device();
	const uint Nz = lbm.get_Nz();
	LayerMeans m;
	m.ux.assign(Nz, 0.0); m.uz.assign(Nz, 0.0); m.T.assign(Nz, 0.0);
	std::vector<double> count(Nz, 0.0);
	for(ulong n=0ull; n<lbm.get_N(); n++) {
		uint x=0u, y=0u, z=0u;
		lbm.coordinates(n, x, y, z);
		m.ux[z] += (double)lbm.u.x[n]; m.uz[z] += (double)lbm.u.z[n]; m.T[z] += (double)lbm.T[n]; count[z] += 1.0;
	}
	for(uint z=0u; z<Nz; z++) { m.ux[z] /= count[z]; m.uz[z] /= count[z]; m.T[z] /= count[z]; }
	return m;
}

/// The nudging run: returns the layer means of ux after tau steps.
std::vector<double> nudged(const uint Dx) {
	const uint N = 32u, Nz = 64u;
	const float nu = 0.01f, alpha = 1.0E-3f;
	const ulong tau = 2000ull, interval = 20ull;
	LBM lbm(N, N, Nz, Dx, 1u, 1u, nu, 0.0f, 0.0f, 0.0f, 0.0f, alpha, 0.0f);
	for(ulong n=0ull; n<lbm.get_N(); n++) lbm.T[n] = 1.0f;
	std::vector<float> fx(Nz, 0.0f), fy(Nz, 0.0f), rate(Nz, 0.0f), T_ref(Nz, 1.0f), target(Nz, 0.0f);
	for(uint z=0u; z<Nz; z++) target[z] = 0.01f*std::sin(2.0f*3.14159265f*(float)z/(float)Nz);
	lbm.set_level_forcing(fx, fy, rate, T_ref);
	lbm.run(0ull);
	for(ulong t=0ull; t<tau; t+=interval) {
		const LayerMeans m = layer_means(lbm);
		for(uint z=0u; z<Nz; z++) fx[z] = (target[z]-(float)m.ux[z])/(float)tau; // the nudging force per step
		lbm.set_level_forcing(fx, fy, rate, T_ref);
		lbm.run(interval);
	}
	return layer_means(lbm).ux;
}

} // namespace

void main_setup() { // extensions: VOLUME_FORCE TEMPERATURE LEVEL_FORCING
	const float e_minus_3 = (float)std::exp(-3.0);
	{ // 1. the nudging, one domain and two
		const std::vector<double> one = nudged(1u), two = nudged(2u);
		double worst = 0.0, split = 0.0;
		const double k = 2.0*3.14159265/(double)one.size(), decay = 0.01*k*k*2000.0; // nu k^2 tau: the sine's viscous decay over tau
		const double reached = (1.0-std::exp(-(1.0+decay)))/(1.0+decay); // the fraction of u_t after tau steps
		std::printf("\nnudging after tau = 2000 steps (target 0.01 sin; with the viscous decay nu k^2 tau = %.3f the expected fraction is %.4f, %.5f at the peak):\n", decay, reached, 0.01*reached);
		for(uint z=0u; z<one.size(); z++) {
			const double target = 0.01*std::sin(2.0*3.14159265*(double)z/(double)one.size());
			worst = std::fmax(worst, std::fabs(one[z]-reached*target)/0.01);
			split = std::fmax(split, std::fabs(one[z]-two[z]));
			if(z%8u==0u) std::printf("  z %2u: <ux> %.6f (expected %.6f), two domains %.6f\n", z, one[z], reached*target, two[z]);
		}
		PhysicsCheck::report("level nudging reaches the nudged-and-diffused fraction of the target at t = tau (of the amplitude)", worst, 0.01);
		PhysicsCheck::report("level nudging: two domains against one (absolute, limit 1e-5)", split, 1.0E-5);
	}
	{ // 2. the w damping and 3. the temperature relaxation in one box
		const uint N = 32u, Nz = 64u;
		const float nu = 0.01f, alpha = 1.0E-3f, r = 1.0f/500.0f;
		LBM lbm(N, N, Nz, nu, 0.0f, 0.0f, 0.0f, 0.0f, alpha, 0.0f);
		for(ulong n=0ull; n<lbm.get_N(); n++) {
			uint x=0u, y=0u, z=0u;
			lbm.coordinates(n, x, y, z);
			lbm.u.z[n] = 0.01f;
			lbm.T[n] = z>=Nz/2u ? 1.1f : 1.0f;
		}
		std::vector<float> fx(Nz, 0.0f), fy(Nz, 0.0f), rate(Nz, r), T_ref(Nz, 1.0f);
		lbm.set_level_forcing(fx, fy, rate, T_ref);
		lbm.run(1500ull);
		const LayerMeans m = layer_means(lbm);
		double worst_w = 0.0, worst_T = 0.0, lower_T = 0.0;
		uint worst_T_z{};
		for(uint z=0u; z<Nz; z++) {
			worst_w = std::fmax(worst_w, std::fabs(m.uz[z]/(0.01*(double)e_minus_3)-1.0));
			if(z>=Nz/2u+8u&&z<Nz-8u) {
				const double e{ std::fabs((m.T[z]-1.0)/(0.1*(double)e_minus_3)-1.0) };
				if(e>worst_T) { worst_T = e; worst_T_z = z; }
			}
			if(z>=8u&&z<Nz/2u-8u) lower_T = std::fmax(lower_T, std::fabs(m.T[z]-1.0));
		}
		std::printf("the worst checked level of the T relaxation: z %u, T %.6f; T at z %u..%u:", worst_T_z, m.T[worst_T_z], Nz/2u+6u, Nz-6u);
		for(uint z=Nz/2u+6u; z<=Nz-6u; z++) std::printf(" %.5f", m.T[z]);
		std::printf("\n");
		std::printf("\nafter 1500 steps at r = 1/500: <w> at z 0 / 32 / 63: %.6f / %.6f / %.6f (expected %.6f); T at z 48: %.6f (expected %.6f), at z 16: %.6f\n",
		            m.uz[0], m.uz[32], m.uz[63], 0.01*(double)e_minus_3, m.T[48], 1.0+0.1*(double)e_minus_3, m.T[16]);
		PhysicsCheck::report("level damping of w: e^-3 after 3/r steps", worst_w, 0.02);
		PhysicsCheck::report("level relaxation of T toward T_ref: e^-3 after 3/r steps", worst_T, 0.02);
		PhysicsCheck::report("level relaxation of T leaves the unsponged levels (absolute)", lower_T, 1.0E-3);
	}
}
