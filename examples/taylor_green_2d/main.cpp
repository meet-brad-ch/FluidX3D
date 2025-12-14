#include "defines.hpp"
#include "info.hpp"
#include "lbm.hpp"
#include "graphics.hpp"
#include "setup.hpp"
#include "shapes.hpp"

void main_setup() { // 2D Taylor-Green vortices (use D2Q9); required extensions in defines.hpp: INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	LBM lbm(1024u, 1024u, 1u, 0.02f);
	// ###################################################################################### define geometry ######################################################################################
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		const float A = 0.2f;
		const uint periodicity = 5u;
		const float a=(float)Nx/(float)periodicity, b=(float)Ny/(float)periodicity;
		const float fx = (float)x+0.5f-0.5f*(float)Nx;
		const float fy = (float)y+0.5f-0.5f*(float)Ny;
		lbm.u.x[n] =  A*cosf(2.0f*pif*fx/a)*sinf(2.0f*pif*fy/b);
		lbm.u.y[n] = -A*sinf(2.0f*pif*fx/a)*cosf(2.0f*pif*fy/b);
		lbm.rho[n] = 1.0f-sq(A)*3.0f/4.0f*(cosf(4.0f*pif*fx/a)+cosf(4.0f*pif*fy/b));
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FIELD;
	lbm.graphics.slice_mode = 3;
	lbm.run();
} /**/
