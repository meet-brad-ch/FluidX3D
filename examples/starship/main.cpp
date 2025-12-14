#include "defines.hpp"
#include "info.hpp"
#include "lbm.hpp"
#include "graphics.hpp"
#include "setup.hpp"
#include "shapes.hpp"

void main_setup() { // Starship; required extensions in defines.hpp: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS or GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint3 lbm_N = resolution(float3(1.0f, 2.0f, 2.0f), 1000u); // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
	const float lbm_Re = 10000000.0f;
	const float lbm_u = 0.05f;
	const ulong lbm_T = 108000ull;
	LBM lbm(lbm_N, 1u, 1u, 1u, units.nu_from_Re(lbm_Re, (float)lbm_N.x, lbm_u));
	// ###################################################################################### define geometry ######################################################################################
	const float size = 1.6f*lbm.size().x;
	const float3 center = float3(lbm.center().x, lbm.center().y+0.05f*size, 0.18f*size);
	lbm.voxelize_stl(get_resource_path("StarShipV2.stl"), center, size); // https://www.thingiverse.com/thing:4912729/files
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(lbm.flags[n]!=TYPE_S) lbm.u.z[n] = lbm_u;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_E; // all non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_LATTICE|VIS_FLAG_SURFACE|VIS_Q_CRITERION;
#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
	lbm.write_status();
	lbm.run(0u, lbm_T); // initialize simulation
	while(lbm.get_t()<=lbm_T) { // main simulation loop
		if(lbm.graphics.next_frame(lbm_T, 20.0f)) {
			lbm.graphics.set_camera_free(float3(2.116744f*(float)Nx, -0.775261f*(float)Ny, 1.026577f*(float)Nz), -38.0f, 37.0f, 60.0f); // top
			lbm.graphics.write_frame(get_exe_path()+"export/top/");
			lbm.graphics.set_camera_free(float3(0.718942f*(float)Nx, 0.311263f*(float)Ny, -0.498366f*(float)Nz), 32.0f, -40.0f, 104.0f); // bottom
			lbm.graphics.write_frame(get_exe_path()+"export/bottom/");
			lbm.graphics.set_camera_free(float3(1.748119f*(float)Nx, 0.442782f*(float)Ny, 0.087945f*(float)Nz), 24.0f, 2.0f, 92.0f); // side
			lbm.graphics.write_frame(get_exe_path()+"export/side/");
		}
		lbm.run(1u, lbm_T);
	}
	lbm.write_status();
#else // GRAPHICS && !INTERACTIVE_GRAPHICS
	lbm.run();
#endif // GRAPHICS && !INTERACTIVE_GRAPHICS
} /**/
