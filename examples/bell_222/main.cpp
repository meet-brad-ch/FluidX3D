#include "defines.hpp"
#include "info.hpp"
#include "lbm.hpp"
#include "graphics.hpp"
#include "setup.hpp"
#include "shapes.hpp"

void main_setup() { // Bell 222 helicopter; required extensions in defines.hpp: FP16C, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS or GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint3 lbm_N = resolution(float3(1.0f, 1.2f, 0.3f), 8000u); // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
	const float lbm_u = 0.16f;
	const float lbm_length = 0.8f*(float)lbm_N.x;
	const float si_T = 0.34483f; // 2 revolutions of the main rotor
	const ulong lbm_dt = 4ull; // revoxelize rotor every dt time steps
	const float si_length=12.85f, si_d=12.12f, si_rpm=348.0f;
	const float si_u = si_rpm/60.0f*si_d*pif;
	const float si_nu=1.48E-5f, si_rho=1.225f;
	units.set_m_kg_s(lbm_length, lbm_u, 1.0f, si_length, si_u, si_rho);
	const float lbm_nu = units.nu(si_nu);
	const ulong lbm_T = units.t(si_T);
	LBM lbm(lbm_N, 1u, 1u, 1u, lbm_nu);
	// ###################################################################################### define geometry ######################################################################################
	const string body_path = get_resource_path("Bell-222-body.stl");
	const string main_path = get_resource_path("Bell-222-main.stl");
	const string back_path = get_resource_path("Bell-222-back.stl");
	if(body_path.empty() || main_path.empty() || back_path.empty()) {
		print_info("This example requires manually splitting BELL222__FIXED.stl into body and rotor components.");
		print_info("Steps:");
		print_info("  1. Download BELL222__FIXED.stl: cd resources && python download_all_thingiverse_stl.py");
		print_info("  2. Open BELL222__FIXED.stl in Microsoft 3D Builder");
		print_info("  3. Separate fuselage, main rotor, and tail rotor into 3 meshes");
		print_info("  4. Save as Bell-222-body.stl, Bell-222-main.stl, and Bell-222-back.stl");
		print_info("  5. Place all 3 files in resources/");
		wait();
		return;
	}
	Mesh* body = read_stl(body_path); // https://www.thingiverse.com/thing:1625155/files
	Mesh* main = read_stl(main_path); // body and rotors separated with Microsoft 3D Builder
	Mesh* back = read_stl(back_path);
	const float scale = lbm_length/body->get_bounding_box_size().y; // scale body and rotors to simulation box size
	body->scale(scale);
	main->scale(scale);
	back->scale(scale);
	const float3 offset = lbm.center()-body->get_bounding_box_center(); // move body and rotors to simulation box center
	body->translate(offset);
	main->translate(offset);
	back->translate(offset);
	body->set_center(body->get_center_of_mass()); // set rotation center of mesh to its center of mass
	main->set_center(main->get_center_of_mass());
	back->set_center(back->get_center_of_mass());
	const float main_radius=0.5f*main->get_max_size(), main_omega=lbm_u/main_radius, main_domega=main_omega*(float)lbm_dt;
	const float back_radius=0.5f*back->get_max_size(), back_omega=-lbm_u/back_radius, back_domega=back_omega*(float)lbm_dt;
	lbm.voxelize_mesh_on_device(body);
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(lbm.flags[n]!=TYPE_S) lbm.u.y[n] =  0.2f*lbm_u;
		if(lbm.flags[n]!=TYPE_S) lbm.u.z[n] = -0.1f*lbm_u;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_E; // all non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_SURFACE|VIS_Q_CRITERION;
	lbm.run(0u, lbm_T); // initialize simulation
	while(lbm.get_t()<=lbm_T) { // main simulation loop
		lbm.voxelize_mesh_on_device(main, TYPE_S, main->get_center(), float3(0.0f), float3(0.0f, 0.0f, main_omega)); // revoxelize mesh on GPU
		lbm.voxelize_mesh_on_device(back, TYPE_S, back->get_center(), float3(0.0f), float3(back_omega, 0.0f, 0.0f)); // revoxelize mesh on GPU
		lbm.run(lbm_dt, lbm_T); // run dt time steps
		main->rotate(float3x3(float3(0.0f, 0.0f, 1.0f), main_domega)); // rotate mesh
		back->rotate(float3x3(float3(1.0f, 0.0f, 0.0f), back_domega)); // rotate mesh
#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
		if(lbm.graphics.next_frame(lbm_T, 10.0f)) {
			lbm.graphics.set_camera_free(float3(0.528513f*(float)Nx, 0.102095f*(float)Ny, 1.302283f*(float)Nz), 16.0f, 47.0f, 96.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/a/");
			lbm.graphics.set_camera_free(float3(0.0f*(float)Nx, -0.114244f*(float)Ny, 0.543265f*(float)Nz), 90.0f+degrees((float)lbm.get_t()/(float)lbm_dt*main_domega), 36.0f, 120.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/b/");
			lbm.graphics.set_camera_free(float3(0.557719f*(float)Nx, -0.503388f*(float)Ny, -0.591976f*(float)Nz), -43.0f, -21.0f, 75.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/c/");
			lbm.graphics.set_camera_centered(58.0f, 9.0f, 88.0f, 1.648722f);
			lbm.graphics.write_frame(get_exe_path()+"export/d/");
			lbm.graphics.set_camera_centered(0.0f, 90.0f, 100.0f, 1.100000f);
			lbm.graphics.write_frame(get_exe_path()+"export/e/");
			lbm.graphics.set_camera_free(float3(0.001612f*(float)Nx, 0.523852f*(float)Ny, 0.992613f*(float)Nz), 90.0f, 37.0f, 94.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/f/");
		}
#endif // GRAPHICS && !INTERACTIVE_GRAPHICS
	}
} /**/
