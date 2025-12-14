#include "defines.hpp"
#include "info.hpp"
#include "lbm.hpp"
#include "graphics.hpp"
#include "setup.hpp"
#include "shapes.hpp"

void main_setup() { // Cessna 172 propeller aircraft; required extensions in defines.hpp: FP16S, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS or GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint3 lbm_N = resolution(float3(1.0f, 0.8f, 0.25f), 8000u); // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
	const float lbm_u = 0.075f;
	const float lbm_width = 0.95f*(float)lbm_N.x;
	const ulong lbm_dt = 4ull; // revoxelize rotor every dt time steps
	const float si_T = 1.0f;
	const float si_width = 11.0f;
	const float si_u = 226.0f/3.6f;
	const float si_nu=1.48E-5f, si_rho=1.225f;
	units.set_m_kg_s(lbm_width, lbm_u, 1.0f, si_width, si_u, si_rho);
	const float lbm_nu = units.nu(si_nu);
	const ulong lbm_T = units.t(si_T);
	print_info("Re = "+to_string(to_uint(units.si_Re(si_width, si_u, si_nu))));
	print_info(to_string(si_T, 3u)+" seconds = "+to_string(lbm_T)+" time steps");
	LBM lbm(lbm_N, units.nu(si_nu));
	// ###################################################################################### define geometry ######################################################################################
	const string body_path = get_resource_path("Cessna-172-Skyhawk-body.stl");
	const string rotor_path = get_resource_path("Cessna-172-Skyhawk-rotor.stl");
	if(body_path.empty() || rotor_path.empty()) {
		print_info("This example requires manually splitting Airplane.stl into body and rotor components.");
		print_info("Steps:");
		print_info("  1. Download Airplane.stl: cd resources && python download_all_thingiverse_stl.py");
		print_info("  2. Open Airplane.stl in Microsoft 3D Builder");
		print_info("  3. Separate body and propeller into 2 meshes");
		print_info("  4. Save as Cessna-172-Skyhawk-body.stl and Cessna-172-Skyhawk-rotor.stl");
		print_info("  5. Place both files in resources/");
		wait();
		return;
	}
	Mesh* plane = read_stl(body_path); // https://www.thingiverse.com/thing:814319/files
	Mesh* rotor = read_stl(rotor_path); // plane and rotor separated with Microsoft 3D Builder
	const float scale = lbm_width/plane->get_bounding_box_size().x; // scale plane and rotor to simulation box size
	plane->scale(scale);
	rotor->scale(scale);
	const float3 offset = lbm.center()-plane->get_bounding_box_center(); // move plane and rotor to simulation box center
	plane->translate(offset);
	rotor->translate(offset);
	plane->set_center(plane->get_center_of_mass()); // set rotation center of mesh to its center of mass
	rotor->set_center(rotor->get_center_of_mass());
	const float lbm_radius=0.5f*rotor->get_max_size(), omega=-lbm_u/lbm_radius, domega=omega*(float)lbm_dt;
	lbm.voxelize_mesh_on_device(plane);
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(lbm.flags[n]!=TYPE_S) lbm.u.y[n] = lbm_u;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_E; // all non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_SURFACE|VIS_Q_CRITERION;
	lbm.run(0u, lbm_T); // initialize simulation
	while(lbm.get_t()<=lbm_T) { // main simulation loop
		lbm.voxelize_mesh_on_device(rotor, TYPE_S, rotor->get_center(), float3(0.0f), float3(0.0f, omega, 0.0f)); // revoxelize mesh on GPU
		lbm.run(lbm_dt, lbm_T); // run dt time steps
		rotor->rotate(float3x3(float3(0.0f, 1.0f, 0.0f), domega)); // rotate mesh
#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
		if(lbm.graphics.next_frame(lbm_T, 5.0f)) {
			lbm.graphics.set_camera_free(float3(0.192778f*(float)Nx, -0.669183f*(float)Ny, 0.657584f*(float)Nz), -77.0f, 27.0f, 100.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/a/");
			lbm.graphics.set_camera_free(float3(0.224926f*(float)Nx, -0.594332f*(float)Ny, -0.277894f*(float)Nz), -65.0f, -14.0f, 100.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/b/");
			lbm.graphics.set_camera_free(float3(-0.000000f*(float)Nx, 0.650189f*(float)Ny, 1.461048f*(float)Nz), 90.0f, 40.0f, 100.0f);
			lbm.graphics.write_frame(get_exe_path()+"export/c/");
		}
#endif // GRAPHICS && !INTERACTIVE_GRAPHICS
	}
} /**/
