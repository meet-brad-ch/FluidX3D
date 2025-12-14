#include "defines.hpp"
#include "info.hpp"
#include "lbm.hpp"
#include "graphics.hpp"
#include "setup.hpp"
#include "shapes.hpp"

void main_setup() { // periodic faucet mass conservation test; required extensions in defines.hpp: FP16S, VOLUME_FORCE, SURFACE, INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	LBM lbm(96u, 192u, 128u, 0.02f, 0.0f, 0.0f, -0.00025f);
	// ###################################################################################### define geometry ######################################################################################
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(y>Ny*5u/6u) lbm.flags[n] = TYPE_F;
		const uint D=max(Nx, Nz), R=D/6;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u) lbm.flags[n] = TYPE_S; // x and y non periodic
		if((z==0u||z==Nz-1u) && sq(x-Nx/2)+sq(y-Nx/2)>sq(R)) lbm.flags[n] = TYPE_S; // z non periodic
		if(y<=Nx/2u+2u*R && torus_x(x, y, z, float3(Nx/2u, Nx/2u+R, Nz)+0.5f, (float)R, (float)R)) lbm.flags[n] = TYPE_S;
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_LATTICE|VIS_PHI_RASTERIZE;
	lbm.run();
} /**/
