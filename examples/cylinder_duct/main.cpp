#include "defines.hpp"
#include "info.hpp"
#include "lbm.hpp"
#include "graphics.hpp"
#include "setup.hpp"
#include "shapes.hpp"

void main_setup() { // cylinder in rectangular duct; required extensions in defines.hpp: VOLUME_FORCE, INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const float Re = 25000.0f;
	const float D = 64.0f;
	const float u = rsqrt(3.0f);
	const float w=D, l=12.0f*D, h=3.0f*D;
	const float nu = units.nu_from_Re(Re, D, u);
	const float f = units.f_from_u_rectangular_duct(w, D, 1.0f, nu, u);
	LBM lbm(to_uint(w), to_uint(l), to_uint(h), nu, 0.0f, f, 0.0f);
	// ###################################################################################### define geometry ######################################################################################
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		lbm.u.y[n] = 0.1f*u;
		if(cylinder(x, y, z, float3(lbm.center().x, 2.0f*D, lbm.center().z), float3(Nx, 0u, 0u), 0.5f*D)) lbm.flags[n] = TYPE_S;
		if(x==0u||x==Nx-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_S; // x and z non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_LATTICE|VIS_Q_CRITERION;
	lbm.run();
} /**/
