// SDF voxelization example - aerodynamics of a cow using pre-computed signed distance field
//
// This example demonstrates voxelize_sdf() as an alternative to voxelize_stl().
// SDF voxelization uses trilinear interpolation for smooth boundaries.
//
// To generate the SDF file from Cow_t.stl:
//   1. Clone SDFGenFast: git clone https://github.com/meet-brad-ch/SDFGenFast
//   2. Build: cd tools && ./configure_cmake.bat Release && ./build_with_vs.bat SDFGen Release
//   3. Run: SDFGen.exe Cow_t.stl 128
//      This creates Cow_t_sdf_128x428x258.sdf with proportional Y/Z dimensions
//   4. Place the .sdf file in the resources/ directory
//
// SDF file format (binary, little-endian):
//   Header (36 bytes): int32 Nx, Ny, Nz; float32 bounds_min[3], bounds_max[3]
//   Data: float32[Nx*Ny*Nz] signed distance values (negative=inside, positive=outside)

#include "defines.hpp"
#include "info.hpp"
#include "lbm.hpp"
#include "graphics.hpp"
#include "setup.hpp"
#include "shapes.hpp"

void main_setup() { // aerodynamics of a cow using SDF; required extensions in defines.hpp: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS or GRAPHICS
	// ################################################################## define simulation box size, viscosity and volume force ###################################################################
	const uint3 lbm_N = resolution(float3(1.0f, 2.0f, 1.0f), 1000u); // input: simulation box aspect ratio and VRAM occupation in MB, output: grid resolution
	const float si_u = 1.0f;
	const float si_length = 2.4f;
	const float si_T = 10.0f;
	const float si_nu=1.48E-5f, si_rho=1.225f;
	const float lbm_length = 0.65f*(float)lbm_N.y;
	const float lbm_u = 0.075f;
	units.set_m_kg_s(lbm_length, lbm_u, 1.0f, si_length, si_u, si_rho);
	const float lbm_nu = units.nu(si_nu);
	const ulong lbm_T = units.t(si_T);
	print_info("Re = "+to_string(to_uint(units.si_Re(si_length, si_u, si_nu))));
	LBM lbm(lbm_N, lbm_nu);
	// ###################################################################################### define geometry ######################################################################################
	// Use SDF file instead of STL - SDF generated from Cow_t.stl using SDFGen
	// Apply same rotation as original cow example: 180 deg around X, then 180 deg around Z
	const float3x3 rotation = float3x3(float3(1, 0, 0), radians(180.0f))*float3x3(float3(0, 0, 1), radians(180.0f));

	// Load SDF to compute bounds (matching what read_stl does)
	const SDF* sdf = read_sdf(get_resource_path("Cow_t_sdf_128x428x258.sdf"));
	const float3 sdf_world_size = sdf->get_world_size();
	const float sdf_max_dim = fmax(fmax(sdf_world_size.x, sdf_world_size.y), sdf_world_size.z);
	const float world_to_lbm = lbm_length / sdf_max_dim;

	// Get SDF bounding box corners in world space, rotate them, find new AABB
	const float3 sdf_center = sdf->get_bounding_box_center();
	const float3 half = 0.5f * sdf_world_size;
	float3 pmin_rot(FLT_MAX), pmax_rot(-FLT_MAX);
	for(int i = 0; i < 8; i++) {
		const float3 corner = float3(
			(i&1) ? half.x : -half.x,
			(i&2) ? half.y : -half.y,
			(i&4) ? half.z : -half.z
		);
		const float3 rotated = rotation * corner; // rotate around origin
		pmin_rot = float3(fmin(pmin_rot.x, rotated.x), fmin(pmin_rot.y, rotated.y), fmin(pmin_rot.z, rotated.z));
		pmax_rot = float3(fmax(pmax_rot.x, rotated.x), fmax(pmax_rot.y, rotated.y), fmax(pmax_rot.z, rotated.z));
	}

	// Scale to LBM units, with mesh centered at lbm.center() (like read_stl does)
	const float3 pmin_lbm = lbm.center() + pmin_rot * world_to_lbm;
	const float3 pmax_lbm = lbm.center() + pmax_rot * world_to_lbm;

	// Apply same translation as original cow example: mesh->translate(float3(0, 1-pmin.y+0.1*lbm_length, 1-pmin.z))
	const float3 translation = float3(0.0f, 1.0f - pmin_lbm.y + 0.1f*lbm_length, 1.0f - pmin_lbm.z);
	const float3 center = lbm.center() + translation;

	delete sdf; // Free SDF, voxelize_sdf will reload it
	lbm.voxelize_sdf(get_resource_path("Cow_t_sdf_128x428x258.sdf"), center, rotation, lbm_length);
	const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz(); parallel_for(lbm.get_N(), [&](ulong n) { uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
		if(z==0u) lbm.flags[n] = TYPE_S; // solid floor
		if(lbm.flags[n]!=TYPE_S) lbm.u.y[n] = lbm_u; // initialize y-velocity everywhere except in solid cells
		if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==Nz-1u) lbm.flags[n] = TYPE_E; // all other simulation box boundaries are inflow/outflow
	}); // ####################################################################### run simulation, export images and data ##########################################################################
	lbm.graphics.visualization_modes = VIS_FLAG_SURFACE|VIS_Q_CRITERION;
#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
	lbm.graphics.set_camera_centered(-40.0f, 20.0f, 78.0f, 1.25f);
	lbm.run(0u, lbm_T); // initialize simulation
	while(lbm.get_t()<=lbm_T) { // main simulation loop
		if(lbm.graphics.next_frame(lbm_T, 10.0f)) lbm.graphics.write_frame();
		lbm.run(1u, lbm_T);
	}
#else // GRAPHICS && !INTERACTIVE_GRAPHICS
	lbm.run();
#endif // GRAPHICS && !INTERACTIVE_GRAPHICS
} /**/
