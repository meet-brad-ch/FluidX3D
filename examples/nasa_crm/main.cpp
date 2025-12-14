#include "defines.hpp"
#include "info.hpp"
#include "lbm.hpp"
#include "graphics.hpp"
#include "setup.hpp"
#include "shapes.hpp"

void main_setup() { // NASA Common Research Model; required extensions in defines.hpp: FP16C, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint3 lbm_N = resolution(float3(1.0f, 1.5f, 1.0f/3.0f), 2000u); // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
	const float Re = 10000000.0f;
	const float u = 0.075f;
	LBM lbm(lbm_N, units.nu_from_Re(Re, (float)lbm_N.x, u));
	// ###################################################################################### define geometry ######################################################################################
	const string stl_path = get_resource_path("crm-hl_reference_ldg.stl");
	if(stl_path.empty()) {
		print_info("This example requires the NASA Common Research Model (CRM) high-lift geometry.");
		print_info("Steps:");
		print_info("  1. Download .stp file from https://commonresearchmodel.larc.nasa.gov/high-lift-crm/high-lift-crm-geometry/assembled-geometry/");
		print_info("  2. Convert .stp to .stl using https://imagetostl.com/convert/file/stp/to/stl");
		print_info("  3. Save as crm-hl_reference_ldg.stl in resources/");
		wait();
		return;
	}
	Mesh* half = read_stl(stl_path, lbm.size(), float3(0.0f), float3x3(float3(0, 0, 1), radians(90.0f)), 1.0f*lbm_N.x);
	half->translate(float3(-0.5f*(half->pmax.x-half->pmin.x), 0.0f, 0.0f));
	Mesh* mesh = new Mesh(2u*half->triangle_number, float3(0.0f));
	for(uint i=0u; i<half->triangle_number; i++) {
		mesh->p0[i] = half->p0[i];
		mesh->p1[i] = half->p1[i];
		mesh->p2[i] = half->p2[i];
	}
	half->rotate(float3x3(float3(1, 0, 0), radians(180.0f))); // mirror-copy half
	for(uint i=0u; i<half->triangle_number; i++) {
		mesh->p0[half->triangle_number+i] = -half->p0[i];
		mesh->p1[half->triangle_number+i] = -half->p1[i];
		mesh->p2[half->triangle_number+i] = -half->p2[i];
	}
	delete half;
	mesh->find_bounds();
	mesh->rotate(float3x3(float3(1, 0, 0), radians(-10.0f)));
	mesh->translate(float3(0.0f, 0.0f, -0.5f*(mesh->pmin.z+mesh->pmax.z)));
	mesh->translate(float3(0.0f, -0.5f*lbm.size().y+mesh->pmax.y+0.5f*(lbm.size().x-(mesh->pmax.x-mesh->pmin.x)), 0.0f));
	mesh->translate(lbm.center());
	lbm.voxelize_mesh_on_device(mesh);
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(lbm.flags[n]!=TYPE_S) lbm.u.y[n] = u;
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==0u||z==Nz-1u) lbm.flags[n] = TYPE_E; // all non periodic
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_SURFACE|VIS_Q_CRITERION;
	lbm.run();
} /**/
