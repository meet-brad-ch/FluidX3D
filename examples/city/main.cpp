#include "defines.hpp"
#include "info.hpp"
#include "lbm.hpp"
#include "graphics.hpp"
#include "setup.hpp"
#include "shapes.hpp"

void main_setup() { // city; required extensions in defines.hpp: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint L = 512u; // 2152u
	const float kmh = 1.0f;
	const float si_u = kmh/3.6f;
	const float si_x = 1000.0f;
	const float si_rho = 1.225f;
	const float si_nu = 1.48E-5f;
	const float Re = units.si_Re(si_x, si_u, si_nu);
	print_info("Re = "+to_string(Re));
	const float u = 0.07f;
	const float size = 1.7f*(float)L;
	units.set_m_kg_s(size, u, 1.0f, si_x, si_u, si_rho);
	const float nu = units.nu(si_nu);
	print_info("1s = "+to_string(units.t(1.0f)));
	LBM lbm(L, L*2u, L/2u, units.nu_from_Re(Re, (float)L, u));
	// ###################################################################################### define geometry ######################################################################################
	const float3 center = lbm.center()-float3(0.0f, 0.05f*size, 0.025f*size);
	const float3x3 rotation = float3x3(float3(0, 0, 1), radians(90.0f));
	lbm.voxelize_stl(get_resource_path("city.stl"), center, rotation, size);
	const uint N=lbm.get_N(), Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); for(uint n=0u, x=0u, y=0u, z=0u; n<N; n++, lbm.coordinates(n, x, y, z)) {
		if(lbm.flags[n]!=TYPE_S) lbm.u.y[n] = u;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_E; // all non periodic
		if(z==0u) lbm.flags[n] = TYPE_S;
	}	// ####################################################################### run simulation, export images and data ##########################################################################
	key_4 = true;
	Clock clock;
	lbm.run(0u);
	while(lbm.get_t()<108000u) {
		lbm.graphics.set_camera_free(float3(-1.088245f*(float)Nx, -0.443919f*(float)Ny, 1.717979f*(float)Nz), 215.0f, 39.0f, 70.0f);
		lbm.graphics.write_frame_png(get_exe_path()+"export/a/");
		lbm.graphics.set_camera_free(float3(0.203233f*(float)Nx, 0.036325f*(float)Ny, 0.435000f*(float)Nz), 56.0f, 45.0f, 105.0f);
		lbm.graphics.write_frame_png(get_exe_path()+"export/b/");
		lbm.graphics.set_camera_free(float3(-0.283501f*(float)Nx, -0.099679f*(float)Ny, 0.175468f*(float)Nz), 234.0f, 29.0f, 117.0f);
		lbm.graphics.write_frame_png(get_exe_path()+"export/c/");
		lbm.run(90u); // run LBM in parallel while CPU is voxelizing the next frame
	}
	write_file(get_exe_path()+"time.txt", print_time(clock.stop()));
	//lbm.run();
} /**/
