#include "defines.hpp"
#include "info.hpp"
#include "lbm.hpp"
#include "graphics.hpp"
#include "setup.hpp"
#include "shapes.hpp"

void main_setup() { // river; required extensions in defines.hpp: FP16S, VOLUME_FORCE, SURFACE, INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	LBM lbm(128u, 384u, 96u, 0.02f, 0.0f, -0.00007f, -0.0005f, 0.01f);
	// ###################################################################################### define geometry ######################################################################################
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		const int R = 20, H = 32;
		if(z==0) lbm.flags[n] = TYPE_S;
		else if(z<H) {
			lbm.flags[n] = TYPE_F;
			lbm.u.y[n] = -0.1f;
		}
		if(cylinder(x, y, z, float3(Nx*2u/3u, Ny*2u/3u, Nz/2u)+0.5f, float3(0u, 0u, Nz), (float)R)) lbm.flags[n] = TYPE_S;
		if(cuboid(x, y, z, float3(Nx/3u, Ny/3u, Nz/2u)+0.5f, float3(2u*R, 2u*R, Nz))) lbm.flags[n] = TYPE_S;
		if(x==0u||x==Nx-1u) lbm.flags[n] = TYPE_S; // x non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = lbm.get_D()==1u ? VIS_PHI_RAYTRACE : VIS_PHI_RASTERIZE;
	lbm.run();
} /**/
