#include "defines.hpp"
#include "info.hpp"
#include "lbm.hpp"
#include "graphics.hpp"
#include "setup.hpp"
#include "shapes.hpp"

void main_setup() { // 3D Taylor-Green vortices; required extensions in defines.hpp: INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	LBM lbm(128u, 128u, 128u, 1u, 1u, 1u, 0.01f);
	// ###################################################################################### define geometry ######################################################################################
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		const float A = 0.25f;
		const uint periodicity = 1u;
		const float a=(float)Nx/(float)periodicity, b=(float)Ny/(float)periodicity, c=(float)Nz/(float)periodicity;
		const float fx = (float)x+0.5f-0.5f*(float)Nx;
		const float fy = (float)y+0.5f-0.5f*(float)Ny;
		const float fz = (float)z+0.5f-0.5f*(float)Nz;
		lbm.u.x[n] =  A*cosf(2.0f*pif*fx/a)*sinf(2.0f*pif*fy/b)*sinf(2.0f*pif*fz/c);
		lbm.u.y[n] = -A*sinf(2.0f*pif*fx/a)*cosf(2.0f*pif*fy/b)*sinf(2.0f*pif*fz/c);
		lbm.u.z[n] =  A*sinf(2.0f*pif*fx/a)*sinf(2.0f*pif*fy/b)*cosf(2.0f*pif*fz/c);
		lbm.rho[n] = 1.0f-sq(A)*3.0f/4.0f*(cosf(4.0f*pif*fx/a)+cosf(4.0f*pif*fy/b));
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_STREAMLINES;
	lbm.run();
	//lbm.run(1000u); lbm.u.read_from_device(); println(lbm.u.x[lbm.index(Nx/2u, Ny/2u, Nz/2u)]); wait(); // test for binary identity
}
