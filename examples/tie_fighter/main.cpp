#include "defines.hpp"
#include "info.hpp"
#include "lbm.hpp"
#include "graphics.hpp"
#include "setup.hpp"
#include "shapes.hpp"

void main_setup() { // Star Wars TIE fighter; required extensions in defines.hpp: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS or GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint3 lbm_N = resolution(float3(1.0f, 2.0f, 1.0f), 1760u); // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
	const float lbm_Re = 100000.0f;
	const float lbm_u = 0.075f;
	const ulong lbm_T = 50000ull;
	const ulong lbm_dt = 28ull;
	LBM lbm(lbm_N, units.nu_from_Re(lbm_Re, (float)lbm_N.x, lbm_u));
	// ###################################################################################### define geometry ######################################################################################
	const float size = 0.65f*lbm.size().x;
	const float3 center = float3(lbm.center().x, 0.6f*size, lbm.center().z);
	const float3x3 rotation = float3x3(float3(1, 0, 0), radians(90.0f));
	Mesh* mesh = read_stl(get_resource_path("DWG_Tie_Fighter_Assembled_02.stl"), lbm.size(), center, rotation, size); // https://www.thingiverse.com/thing:2919109/files
	lbm.voxelize_mesh_on_device(mesh);
	lbm.flags.read_from_device();
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(lbm.flags[n]!=TYPE_S) lbm.u.y[n] = lbm_u;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_E; // all non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_LATTICE|VIS_FLAG_SURFACE|VIS_Q_CRITERION;
	lbm.run(0u, lbm_T); // initialize simulation
	while(lbm.get_t()<lbm_T) { // main simulation loop
#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
		if(lbm.graphics.next_frame(lbm_T, 30.0f)) {
			lbm.graphics.set_camera_free(float3(1.0f*(float)Nx, -0.4f*(float)Ny, 0.63f*(float)Nz), -33.0f, 33.0f, 80.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/t/");
			lbm.graphics.set_camera_free(float3(0.3f*(float)Nx, -1.5f*(float)Ny, -0.45f*(float)Nz), -83.0f, -10.0f, 40.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/b/");
			lbm.graphics.set_camera_free(float3(0.0f*(float)Nx, 0.57f*(float)Ny, 0.7f*(float)Nz), 90.0f, 29.5f, 80.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/f/");
			lbm.graphics.set_camera_free(float3(2.5f*(float)Nx, 0.0f*(float)Ny, 0.0f*(float)Nz), 0.0f, 0.0f, 50.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/s/");
		}
#endif // GRAPHICS && !INTERACTIVE_GRAPHICS
		lbm.run(lbm_dt, lbm_T);
		const float3x3 rotation = float3x3(float3(0.2f, 1.0f, 0.1f), radians(0.4032f)); // create rotation matrix to rotate mesh
		lbm.unvoxelize_mesh_on_device(mesh);
		mesh->rotate(rotation); // rotate mesh
		lbm.voxelize_mesh_on_device(mesh);
	}
} /**/
