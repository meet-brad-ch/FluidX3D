// A plain D3Q7 BGK scalar lattice on the host, written from the textbook equations the kernel claims to solve
// (calculate_g_eq() of kernel.cpp, operation by operation): push streaming, periodic, bounce-back at solid cells with
// a delay of 1 or 2 steps. Used by scalar_reference.cpp (fed with the GPU's flow, DEBUG_PROBE) and scalar_scheme_lab.cpp
// (fed with prescribed flows).
#pragma once

#include "setup/setup.hpp"

#include <array>
#include <cmath>
#include <vector>
/// @brief A plain D3Q7 BGK scalar lattice on the host, fed with an equilibrium velocity and a relaxation rate per cell and step.
class ScalarReference {
public:
	/// @brief Starts every fluid cell at the equilibrium at rest of its value, as the kernel's initialize does.
	/// @param nx, ny, nz the box (periodic in every direction where no solid cell is)
	/// @param solid_cells true for the TYPE_S cells
	/// @param start the start value of every cell (T or Q, lattice units)
	/// @param bounce_back_delay 1: the population sent toward a wall returns on the next step (standard halfway
	///        bounce-back); 2: it returns one step later (what the Esoteric-Pull kernel does)
	ScalarReference(const uint nx, const uint ny, const uint nz, const std::vector<bool>& solid_cells, const std::vector<float>& start, const uint bounce_back_delay)
		: Nx(nx), Ny(ny), Nz(nz), N((ulong)nx*(ulong)ny*(ulong)nz), solid(solid_cells), delay(bounce_back_delay) {
		if(delay!=1u&&delay!=2u) print_error("ScalarReference: the bounce-back delay is 1 or 2 steps.");
		if(solid.size()!=N||start.size()!=N) print_error("ScalarReference: the solid and start vectors need Nx Ny Nz values.");
		g.assign(7ull*N, 0.0f);
		g_old.assign(7ull*N, 0.0f);
		incoming.assign(7ull*N, 0.0f);
		value.assign(N, 0.0f);
		for(ulong n=0ull; n<N; n++) {
			std::array<float, 7> geq{};
			equilibrium(start[n], 0.0f, 0.0f, 0.0f, geq);
			for(uint i=0u; i<7u; i++) { g[i*N+n] = geq[i]; g_old[i*N+n] = geq[i]; }
		}
	}
	/// @brief The solid cells of a GPU lattice (TYPE_S, not TYPE_E: TYPE_BO of the kernel), for the constructor.
	static std::vector<bool> solid_of(LBM& lbm) {
		std::vector<bool> s(lbm.get_N(), false);
		for(ulong n=0ull; n<lbm.get_N(); n++) s[n] = (lbm.flags[n]&(TYPE_S|TYPE_E))==TYPE_S;
		return s;
	}
	/// @brief Streams the post-collision populations (push, periodic, bounce-back at solid cells) and sums the value.
	void stream() {
		for(uint z=0u; z<Nz; z++) for(uint y=0u; y<Ny; y++) for(uint x=0u; x<Nx; x++) {
			const ulong n = index(x, y, z);
			if(solid[n]) continue;
			float sum = 0.0f;
			for(uint i=0u; i<7u; i++) {
				const ulong s = index((x+Nx-(uint)(int)cx[i])%Nx, (y+Ny-(uint)(int)cy[i])%Ny, (z+Nz-(uint)(int)cz[i])%Nz); // the cell the population comes from, n - c_i
				const float gi = solid[s] ? (delay==1u ? g : g_old)[opposite[i]*N+n] : g[i*N+s]; // bounce-back: the cell's own population sent toward the wall returns reversed, after the delay
				incoming[i*N+n] = gi;
				sum += gi; // the kernel's order: g_0 .. g_6, then + 1
			}
			value[n] = sum+1.0f;
		}
	}
	/// @brief Collides with this step's equilibrium velocity and relaxation rate.
	/// @param u_eq ux, uy, uz of every cell (3 N floats, structure of arrays)
	/// @param w_T the relaxation rate of every cell (N floats)
	void collide(const float* u_eq, const float* w_T) {
		g_old.swap(g); // the previous post-collision populations, for the delayed bounce-back
		const float* u = u_eq;
		if(average) { // the mean of this step's and the last step's velocity: the step-to-step (acoustic) part removed
			if(u_last.empty()) u_last.assign(u_eq, u_eq+3ull*N);
			u_mean.resize(3ull*N);
			for(ulong m=0ull; m<3ull*N; m++) { u_mean[m] = 0.5f*(u_eq[m]+u_last[m]); u_last[m] = u_eq[m]; }
			u = u_mean.data();
		}
		if(smooth) { // the D3Q19-consistent velocity: each component averaged over the 4 neighbours across it, weights 1/3 and 1/6
			smoothed.assign(3ull*N, 0.0f);
			for(uint z=0u; z<Nz; z++) for(uint y=0u; y<Ny; y++) for(uint x=0u; x<Nx; x++) {
				const ulong n = index(x, y, z);
				const ulong xp = index((x+1u)%Nx, y, z), xm = index((x+Nx-1u)%Nx, y, z), yp = index(x, (y+1u)%Ny, z), ym = index(x, (y+Ny-1u)%Ny, z), zp = index(x, y, (z+1u)%Nz), zm = index(x, y, (z+Nz-1u)%Nz);
				smoothed[n]          = u_eq[n]/3.0f+(u_eq[yp]+u_eq[ym]+u_eq[zp]+u_eq[zm])/6.0f;
				smoothed[N+n]        = u_eq[N+n]/3.0f+(u_eq[N+xp]+u_eq[N+xm]+u_eq[N+zp]+u_eq[N+zm])/6.0f;
				smoothed[2ull*N+n]   = u_eq[2ull*N+n]/3.0f+(u_eq[2ull*N+xp]+u_eq[2ull*N+xm]+u_eq[2ull*N+yp]+u_eq[2ull*N+ym])/6.0f;
			}
			u = smoothed.data();
		}
		for(ulong n=0ull; n<N; n++) {
			if(solid[n]) continue;
			std::array<float, 7> geq{};
			equilibrium(value[n], u[n], u[N+n], u[2ull*N+n], geq);
			const float w = rate>0.0f ? rate : w_T[n];
			for(uint i=0u; i<7u; i++) g[i*N+n] = std::fma(1.0f-w, incoming[i*N+n], w*geq[i]);
		}
		if(chopard) { // Chopard, Falcone, Latt 2009 eq. 14: F_i = (c_i/c_s^2) t_i (1 - 1/(2 tau)) d_t(C u), D3Q7: t_i 1/8, c_s^2 1/4
			if(flux_last.empty()) { flux_last.assign(3ull*N, 0.0f); for(ulong n=0ull; n<N; n++) for(uint d=0u; d<3u; d++) flux_last[(ulong)d*N+n] = value[n]*u[(ulong)d*N+n]; }
			for(ulong n=0ull; n<N; n++) {
				if(solid[n]) continue;
				const float w = rate>0.0f ? rate : w_T[n];
				const float k = 0.5f*(1.0f-0.5f*w);
				for(uint d=0u; d<3u; d++) {
					const float f = value[n]*u[(ulong)d*N+n];
					const float dt = f-flux_last[(ulong)d*N+n];
					flux_last[(ulong)d*N+n] = f;
					g[(2u*d+1u)*N+n] = std::fma(k, dt, g[(2u*d+1u)*N+n]);
					g[(2u*d+2u)*N+n] = std::fma(-k, dt, g[(2u*d+2u)*N+n]);
				}
			}
		}
		if(advective) { // the advective-form source +C div_7(u): the conservative scheme's -C div u is cancelled, a uniform C stays uniform
			for(uint z=0u; z<Nz; z++) for(uint y=0u; y<Ny; y++) for(uint x=0u; x<Nx; x++) {
				const ulong n = index(x, y, z);
				if(solid[n]) continue;
				const ulong xp = index((x+1u)%Nx, y, z), xm = index((x+Nx-1u)%Nx, y, z), yp = index(x, (y+1u)%Ny, z), ym = index(x, (y+Ny-1u)%Ny, z), zp = index(x, y, (z+1u)%Nz), zm = index(x, y, (z+Nz-1u)%Nz);
				const float div = 0.5f*((u[xp]-u[xm])+(u[N+yp]-u[N+ym])+(u[2ull*N+zp]-u[2ull*N+zm])); // solid neighbours carry u = 0
				const float q = value[n]*div;
				g[n] = std::fma(0.25f, q, g[n]); // spread like the kernel's heat source: weights 1/4 and 1/8
				for(uint i=1u; i<7u; i++) g[i*N+n] = std::fma(0.125f, q, g[i*N+n]);
			}
		}
	}
	/// @brief The value of a cell after the last stream (Sum g + 1).
	float at(const ulong n) const { return value[n]; }
	/// @brief The flux Sum c g of a cell after the last stream, component d (0 x, 1 y, 2 z).
	float flux(const ulong n, const uint d) const { return incoming[(2u*d+1u)*N+n]-incoming[(2u*d+2u)*N+n]; }
	/// @brief Population i of a cell after the last stream (before the collision).
	float population(const ulong n, const uint i) const { return incoming[i*N+n]; }
	/// @brief Whether a cell is solid.
	bool is_solid(const ulong n) const { return solid[n]; }
	/// @brief A scheme variant: the equilibrium takes the velocity averaged across each component
	/// with the D3Q19 weights (1/3 itself, 1/6 each of the 4 neighbours across), so that the D3Q7 central-difference
	/// divergence of the scalar's velocity equals the D3Q19 divergence of the flow's (the mass continuity).
	void use_d3q19_consistent_velocity(const bool on) { smooth = on; }
	/// @brief A scheme variant: a source +C div_7(u) per step turns the conservative form
	/// d_t C + div(C u) into the advective form d_t C + u . grad C, which keeps a uniform C uniform whatever div u is
	/// (the scalar treatment of Feng et al. 2021, the anelastic LB paper, done there by finite volumes).
	void use_advective_form(const bool on) { advective = on; }
	/// @brief A diagnostic: the relaxation rate forced to a value for every cell (0: the given wT).
	void override_rate(const float w) { rate = w; }
	/// @brief A diagnostic: the equilibrium takes the mean of this and the last step's velocity.
	void average_velocity_over_two_steps(const bool on) { average = on; }
	/// @brief A scheme variant: the Chopard, Falcone, Latt 2009 correction force (eq. 14), which
	/// cancels the linear equilibrium's (D/c_s^2) d_t div(C u) error term.
	void use_chopard_correction(const bool on) { chopard = on; }
private:
	bool chopard = false;            ///< the Chopard correction variant
	std::vector<float> flux_last;    ///< C u of the last step, for d_t(C u)
	float rate = 0.0f;               ///< the forced relaxation rate (0: off)
	bool average = false;            ///< the two-step velocity mean
	std::vector<float> u_last, u_mean;
	bool advective = false;          ///< the advective-form source variant
	bool smooth = false;             ///< the D3Q19-consistent velocity variant
	std::vector<float> smoothed;     ///< its velocity field, 3 N
	uint Nx = 0u, Ny = 0u, Nz = 0u;
	ulong N = 0ull;
	std::vector<bool> solid;
	uint delay = 1u;             ///< the bounce-back delay in steps
	std::vector<float> g;        ///< post-collision populations, population i of cell n at i*N+n
	std::vector<float> g_old;    ///< the post-collision populations of the step before
	std::vector<float> incoming; ///< after streaming
	std::vector<float> value;    ///< Sum g + 1 after streaming
	static constexpr std::array<int, 7> cx = { 0, 1, -1, 0, 0, 0, 0 }; ///< D3Q7 in the kernel's order: 0, +x, -x, +y, -y, +z, -z
	static constexpr std::array<int, 7> cy = { 0, 0, 0, 1, -1, 0, 0 };
	static constexpr std::array<int, 7> cz = { 0, 0, 0, 0, 0, 1, -1 };
	static constexpr std::array<uint, 7> opposite = { 0u, 2u, 1u, 4u, 3u, 6u, 5u };
	ulong index(const uint x, const uint y, const uint z) const { return (ulong)x+((ulong)y+(ulong)z*(ulong)Ny)*(ulong)Nx; }
	/// @brief calculate_g_eq() of the kernel, operation by operation.
	static void equilibrium(const float T, const float ux, const float uy, const float uz, std::array<float, 7>& geq) {
		const float wsT4 = 0.5f*T, wsTm1 = 0.125f*(T-1.0f);
		geq[0] = std::fma(0.25f, T, -0.25f);
		geq[1] = std::fma(wsT4, ux, wsTm1); geq[2] = std::fma(wsT4, -ux, wsTm1);
		geq[3] = std::fma(wsT4, uy, wsTm1); geq[4] = std::fma(wsT4, -uy, wsTm1);
		geq[5] = std::fma(wsT4, uz, wsTm1); geq[6] = std::fma(wsT4, -uz, wsTm1);
	}
};
