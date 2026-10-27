// The moist thermal in saturated, neutral air: the box, its physical scales
// and its set-up, used by moist_bubble.cpp (the moist thermal against its dry twin).
#pragma once

#include "setup/setup.hpp"
#include "moist_air.hpp"

#include <cmath>
#include <vector>

/// @brief The box and its physical scales: cells of 10 m, dt 0.16 s, the CBL example's lattice parameters.
struct ThermalBox {
	uint N = 64u;                ///< cells wide (periodic in x and y)
	uint Nz = 128u;              ///< cells high, the bottom and top layers solid
	double dx = 10.0;            ///< m
	double dt = 0.16;            ///< s
	double env_thl = 300.0;      ///< K
	double moist_qt = 0.026;     ///< kg/kg, the moist case's total water everywhere
	double excess = 2.0;         ///< K, the moist thermal's peak thl excess
	float centre_z = 30.0f;      ///< cells
	float radius = 16.0f;        ///< cells
	float nu = 1.0E-5f;          ///< lattice viscosity, as in the CBL example
	float alpha = 0.014f;        ///< lattice diffusivity (tauT 0.556 with THERMAL_DIFFUSIVITY_FIX), as in the CBL example
	float fz = -1.0E-6f;         ///< the tiny volume force of the Boussinesq convention
	float inv_prandtl_t = 3.0f;  ///< Pr_t 1/3, as in the CBL example
	ulong sample = 50ull;        ///< steps between read-outs
	ulong steps = 1500ull;       ///< steps at most
	/// @brief The height of a level's cell centre above the floor in m.
	double height(const uint z) const { return ((double)z-0.5)*dx; }
	/// @brief The level's pressure in Pa, hydrostatic for the environment's thl from 101 325 Pa.
	double pressure(const uint z) const { return moist_air::hydrostatic_pressure(std::fmax(height(z), 0.0), env_thl); }
	/// @brief beta of the example's convention: |fz| beta (theta - theta_ref) is g dt^2/dx (theta - theta_ref)/theta_0.
	float beta() const { return (float)(9.81*dt*dt/dx/(env_thl*1.0E-6)); }
	/// @brief The cos^2 shape of the thermal at a cell, 1 at the centre, 0 from one radius out.
	double shape(const uint x, const uint y, const uint z) const {
		const double cx = 0.5*(double)N;
		const double L = std::sqrt(sq((double)x+0.5-cx)+sq((double)y+0.5-cx)+sq((double)z+0.5-(double)centre_z))/(double)radius;
		return L<1.0 ? sq(std::cos(0.5*3.14159265358979*L)) : 0.0;
	}
	/// @brief Sets up a constructed LBM (N x N x Nz, nu, fz, alpha, beta() of this box): the solid floor and lid with the
	/// air's values (the first Esoteric-Pull step streams the wall cells' initial populations into the fluid), the thl
	/// excess, the uniform qt, theta_0, Pr_t and the level pressure and reference theta_v. The density starts in
	/// hydrostatic balance with the volume force fz: p = rho / 3 and dp/dz = fz give rho = 1 + 3 fz (z - the middle
	/// level), mean 1. A uniform start left the box out of balance, and its lowest vertical sound mode rang at
	/// fz times a quarter period (1.1e-4 lattice, 7 mm/s), which the rest test of Bryan and Fritsch 2002 caught.
	/// @param lbm the LBM to set up
	/// @param qt the total water of the air everywhere (kg/kg)
	/// @param excess_at the thl excess of a cell at the start (K), called as excess_at(x, y, z)
	/// @return the largest start excess (K)
	template<typename Excess> double set_up(LBM& lbm, const double qt, const Excess& excess_at) const {
		double excess_max = 0.0;
		const double middle = 0.5*(double)(Nz-1u);
		for(ulong n=0ull; n<lbm.get_N(); n++) {
			uint x=0u, y=0u, z=0u;
			lbm.coordinates(n, x, y, z);
			lbm.rho[n] = (float)(1.0+3.0*(double)fz*((double)z-middle));
			lbm.Q[n] = (float)(1.0+qt/0.01);
			if(z==0u||z==Nz-1u) { lbm.flags[n] = TYPE_S; lbm.T[n] = 1.0f; continue; }
			const double e = excess_at(x, y, z);
			excess_max = std::fmax(excess_max, e);
			lbm.T[n] = (float)(1.0+e);
		}
		lbm.set_theta_0((float)env_thl);
		lbm.set_inv_prandtl_t(inv_prandtl_t);
		std::vector<float> fx(Nz, 0.0f), fy(Nz, 0.0f), rate(Nz, 0.0f), T_ref(Nz, 1.0f), p(Nz, 0.0f), thv(Nz, 0.0f);
		for(uint z=0u; z<Nz; z++) { p[z] = (float)pressure(z); thv[z] = (float)moist_air::virtual_theta(env_thl, qt, pressure(z)); }
		lbm.set_level_forcing(fx, fy, rate, T_ref);
		lbm.set_level_moist(p, thv);
		return excess_max;
	}
};
