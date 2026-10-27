// Eddy heat diffusivity: a temperature wave T = 1 + A sin(2 pi y / Ny) decays by diffusion at exp(-D k^2 t). In a
// steady Couette flow u_x(z) between two equilibrium boundary layers, the flow carries no heat across y, and the
// Smagorinsky model of SUBGRID gives the uniform eddy viscosity nu_t = (C Delta)^2 S in the bulk, S = du_x/dz,
// (C Delta)^2 = 0.76421222/(18 sqrt 2) = 0.030019 (the constant in kernel.cpp). With LBM::set_inv_prandtl_t(1/Pr_t)
// the wave must decay with D_0 + nu_t/Pr_t, and without it with D_0 alone.
// D_0 is the diffusivity of the thermal relaxation rate FluidX3D sets for alpha, wT = 1/(2 alpha + 1/2): the D3Q7
// lattice diffuses with D = c^2 (tauT - 1/2), c^2 = 1/4, so D_0 = alpha/2.
// Measured in the middle half of the channel: the shear rate from the velocity field (the equilibrium layers make it a
// few % steeper than 2U/(Nz-1)), the diffusivity from the wave's amplitude at two times.
// Passes when the measured diffusivity is within 3 % of the expected one in both runs.

#include "setup/setup.hpp"
#include "physics_check.hpp"

#include <cmath>

namespace {

/// What one run measured in the middle half of the channel, in lattice units.
struct Measured {
	double diffusivity = 0.0; ///< the decay rate of the temperature wave over k^2
	double shear = 0.0;       ///< du_x/dz
};

/// The temperature wave's decay and the shear rate, in a Couette flow with walls at u_x = -U and +U.
Measured measure(float inv_prandtl_t, float alpha, float U, uint Nz) {
	const uint Nx = 4u, Ny = 64u;
	const double k = 2.0*pi/(double)Ny; // pi: the macro of utilities.hpp
	const float A = 0.01f;
	const float zc = 0.5f*(float)(Nz-1u);
	const float S = 2.0f*U/(float)(Nz-1u); // the start profile's shear rate between the layers z = 0 and z = Nz-1
	LBM lbm(Nx, Ny, Nz, 1.0E-4f, 0.0f, 0.0f, 0.0f, 0.0f, alpha, 0.0f);
	for(ulong n=0ull; n<lbm.get_N(); n++) {
		uint x=0u, y=0u, z=0u;
		lbm.coordinates(n, x, y, z);
		lbm.u.x[n] = S*((float)z-zc); // the Couette profile from the start
		lbm.T[n] = 1.0f+A*(float)std::sin(k*(double)y);
		if(z==0u||z==Nz-1u) lbm.flags[n] = TYPE_E;
	}
	lbm.set_inv_prandtl_t(inv_prandtl_t);

	const auto middle = [&](uint z) { return 4u*z>=Nz&&4u*z<=3u*Nz; };
	const auto amplitude = [&]() { // the sine mode's amplitude over the middle half of the channel
		lbm.update_fields();
		lbm.T.read_from_device();
		double s = 0.0;
		ulong cells = 0ull;
		for(ulong n=0ull; n<lbm.get_N(); n++) {
			uint x=0u, y=0u, z=0u;
			lbm.coordinates(n, x, y, z);
			if(!middle(z)) continue;
			s += ((double)lbm.T[n]-1.0)*std::sin(k*(double)y);
			cells++;
		}
		return 2.0*s/(double)cells;
	};
	const ulong t1 = 2000ull, t2 = 30000ull;
	lbm.run(t1);
	const double A1 = amplitude();
	lbm.run(t2-t1);
	const double A2 = amplitude();

	lbm.u.read_from_device(); // the shear rate: least-squares slope of u_x over z in the middle half
	double sz = 0.0, su = 0.0, szz = 0.0, szu = 0.0, cells = 0.0;
	for(ulong n=0ull; n<lbm.get_N(); n++) {
		uint x=0u, y=0u, z=0u;
		lbm.coordinates(n, x, y, z);
		if(!middle(z)) continue;
		const double zd = (double)z, ud = (double)lbm.u.x[n];
		sz += zd; su += ud; szz += zd*zd; szu += zd*ud; cells += 1.0;
	}
	Measured m;
	m.diffusivity = std::log(A1/A2)/(k*k*(double)(t2-t1));
	m.shear = (cells*szu-sz*su)/(cells*szz-sz*sz);
	return m;
}

} // namespace

void main_setup() { // extensions: EQUILIBRIUM_BOUNDARIES TEMPERATURE SUBGRID
	const float alpha = 1.0E-4f, U = 0.15f, inv_prandtl_t = 3.0f; // Pr_t = 1/3
	const uint Nz = 98u; // tall: the faster decay next to the equilibrium layers (steeper shear there) spreads ~5 cells by t2, far from the middle half
#ifdef THERMAL_DIFFUSIVITY_FIX
	const double D0{ (double)alpha }; // wT = 1/(4 alpha + 1/2): alpha is the diffusivity (also SCALAR_FV, which needs the fix)
#else // THERMAL_DIFFUSIVITY_FIX
	const double D0{ 0.5*(double)alpha }; // wT = 1/(2 alpha + 1/2): the D3Q7 lattice diffuses with alpha/2
#endif // THERMAL_DIFFUSIVITY_FIX
	const double smagorinsky = 0.76421222/(18.0*std::sqrt(2.0)); // (C Delta)^2

	const Measured plain = measure(0.0f, alpha, U, Nz);
	std::printf("\nwithout eddy heat diffusivity: D %.4e (expected D_0 %.4e)\n", plain.diffusivity, D0);
	PhysicsCheck::report("eddy heat diffusivity: off, D_0 only", std::fabs(plain.diffusivity/D0-1.0), 0.03);

	const Measured eddy = measure(inv_prandtl_t, alpha, U, Nz);
	const double nu_t = smagorinsky*eddy.shear;
	const double expected = D0+nu_t*(double)inv_prandtl_t;
	std::printf("\nwith eddy heat diffusivity: D %.4e (expected %.4e; shear %.4e, nu_t %.4e)\n", eddy.diffusivity, expected, eddy.shear, nu_t);
	PhysicsCheck::report("eddy heat diffusivity: D_0 + nu_t/Pr_t", std::fabs(eddy.diffusivity/expected-1.0), 0.03);
}
