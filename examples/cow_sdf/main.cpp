// Aerodynamics of a cow, voxelized from a precomputed signed distance field (SDF)
//
// SDF voxelization interpolates the distance field trilinearly, for smooth surfaces: an alternative to an STL.
// The model is the SDF file; its grid's extent is the model's size. To generate the SDF file from Cow_t.stl:
//   1. Clone SDFGenFast: git clone https://github.com/meet-brad-ch/SDFGenFast
//   2. Build: cd tools && ./configure_cmake.bat Release && ./build_with_vs.bat SDFGen Release
//   3. Run: SDFGen.exe Cow_t.stl 128
//      This creates Cow_t_sdf_128x428x258.sdf with proportional Y/Z dimensions
//   4. Place the .sdf file in the resources/ directory
//
// SDF file format (binary, little-endian):
//   Header (36 bytes): int32 Nx, Ny, Nz; float32 bounds_min[3], bounds_max[3]
//   Data: float32[Nx*Ny*Nz] signed distance values (negative=inside, positive=outside)

#include "setup/setup.hpp"

void main_setup() { // extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS or GRAPHICS
	const Speed flow_velocity = 1.0_mps;
	const Length cow_length = 2.4_m; // the SDF grid's length, along Y
	const Length domain_length = cow_length / 0.65f; // the cow is 65 % of the domain length

	Simulation sim(Domain::around(Model("Cow_t_sdf_128x428x258.sdf").rotation(180_deg, 0_deg, 180_deg).length(cow_length))
		.size(0.5f * domain_length, domain_length, 0.5f * domain_length)
		.gap_to_inlet(0.1f * cow_length) // the cow's nose
		.on_floor()
		.vram(1000_mb),
		Fluid::AIR, flow_velocity, LatticeMach(0.13f)); // the original's lattice speed 0.075

	sim.boundaries()
		.set_solid_floor()
		.set_open_boundaries()
		.initialize_velocity_y(flow_velocity)
		.apply();

	sim.graphics()
		.show_surface()
		.show_vortices()
		.apply();

	sim.video()
		.add(CameraView::orbit(-40_deg, 20_deg).field_of_view(78_deg).view_height(domain_length / 1.25f))
		.set_length(10.0_s);
	sim.run_for(10.0_s);
}
