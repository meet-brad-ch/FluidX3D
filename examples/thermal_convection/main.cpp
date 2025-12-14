#include "defines.hpp"
#include "info.hpp"
#include "lbm.hpp"
#include "graphics.hpp"
#include "setup.hpp"
#include "shapes.hpp"

void main_setup() { // thermal convection; required extensions in defines.hpp: FP16S, VOLUME_FORCE, TEMPERATURE, INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	LBM lbm(32u, 196u, 60u, 1u, 1u, 1u, 0.02f, 0.0f, 0.0f, -0.0005f, 0.0f, 1.0f, 1.0f);
	// ###################################################################################### define geometry ######################################################################################
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(y==1) {
			lbm.T[n] = 1.8f;
			lbm.flags[n] = TYPE_T;
		} else if(y==Ny-2) {
			lbm.T[n] = 0.3f;
			lbm.flags[n] = TYPE_T;
		}
		lbm.rho[n] = units.rho_hydrostatic(0.0005f, (float)z, 0.5f*(float)Nz); // initialize density with hydrostatic pressure
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_S; // all non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_LATTICE|VIS_STREAMLINES;
	lbm.run();
	//lbm.run(1000u); lbm.u.read_from_device(); println(lbm.u.x[lbm.index(Nx/2u, Ny/2u, Nz/2u)]); wait(); // test for binary identity
} /**/