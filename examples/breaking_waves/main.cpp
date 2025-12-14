#include "defines.hpp"
#include "info.hpp"
#include "lbm.hpp"
#include "graphics.hpp"
#include "setup.hpp"
#include "shapes.hpp"

void main_setup() { // breaking waves on beach; required extensions in defines.hpp: FP16S, VOLUME_FORCE, EQUILIBRIUM_BOUNDARIES, SURFACE, INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const float f = 0.001f; // make smaller
	const float u = 0.12f; // peak velocity of speaker membrane
	const float frequency = 0.0007f; // amplitude = u/(2.0f*pif*frequency);
	LBM lbm(128u, 640u, 96u, 0.01f, 0.0f, 0.0f, -f);
	// ###################################################################################### define geometry ######################################################################################
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		const uint H = Nz/2u;
		if(z<H) {
			lbm.flags[n] = TYPE_F;
			lbm.rho[n] = units.rho_hydrostatic(f, (float)z, (float)H);
		}
		if(plane(x, y, z, float3(lbm.center().x, 128.0f, 0.0f), float3(0.0f, -1.0f, 8.0f))) lbm.flags[n] = TYPE_S;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_S; // all non periodic
		if(y==0u && x>0u&&x<Nx-1u&&z>0u&&z<Nz-1u) lbm.flags[n] = TYPE_E;
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_LATTICE | (lbm.get_D()==1u ? VIS_PHI_RAYTRACE : VIS_PHI_RASTERIZE);
	lbm.run(0u); // initialize simulation
	while(true) { // main simulation loop
		lbm.u.read_from_device();
		const float uy = u*sinf(2.0f*pif*frequency*(float)lbm.get_t());
		const float uz = 0.5f*u*cosf(2.0f*pif*frequency*(float)lbm.get_t());
		for(uint z=1u; z<Nz-1u; z++) {
			for(uint y=0u; y<1u; y++) {
				for(uint x=1u; x<Nx-1u; x++) {
					const uint n = x+(y+z*Ny)*Nx;
					lbm.u.y[n] = uy;
					lbm.u.z[n] = uz;
				}
			}
		}
		lbm.u.write_to_device();
		lbm.run(100u);
	}
} /**/
