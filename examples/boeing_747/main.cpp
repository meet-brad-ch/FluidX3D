#include "defines.hpp"
#include "info.hpp"
#include "lbm.hpp"
#include "graphics.hpp"
#include "setup.hpp"
#include "shapes.hpp"

void main_setup() { // Boeing 747; required extensions in defines.hpp: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS or GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint3 lbm_N = resolution(float3(1.0f, 2.0f, 0.5f), 880u); // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
	const float lbm_Re = 1000000.0f;
	const float lbm_u = 0.075f;
	const ulong lbm_T = 10000ull;
	LBM lbm(lbm_N, units.nu_from_Re(lbm_Re, (float)lbm_N.x, lbm_u));
	// ###################################################################################### define geometry ######################################################################################
	const float size = 1.0f*lbm.size().x;
	const float3 center = float3(lbm.center().x, 0.55f*size, lbm.center().z);
	const float3x3 rotation = float3x3(float3(1, 0, 0), radians(-15.0f));
	lbm.voxelize_stl(get_resource_path("techtris_airplane.stl"), center, rotation, size); // https://www.thingiverse.com/thing:2772812/files
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(lbm.flags[n]!=TYPE_S) lbm.u.y[n] = lbm_u;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_E; // all non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_SURFACE|VIS_Q_CRITERION;
#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
	lbm.graphics.set_camera_free(float3(1.0f*(float)Nx, -0.4f*(float)Ny, 2.0f*(float)Nz), -33.0f, 42.0f, 68.0f);
	lbm.run(0u, lbm_T); // initialize simulation
	while(lbm.get_t()<lbm_T) { // main simulation loop
		if(lbm.graphics.next_frame(lbm_T, 10.0f)) lbm.graphics.write_frame(); // render enough frames 10 seconds of 60fps video
		lbm.run(1u, lbm_T);
	}
#else // GRAPHICS && !INTERACTIVE_GRAPHICS
	lbm.run();
#endif // GRAPHICS && !INTERACTIVE_GRAPHICS
} /**/
