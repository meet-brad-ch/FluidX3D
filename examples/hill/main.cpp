// Wind over hill terrain with an atmospheric boundary layer profile
// Terrain: resources/hill.stl (Unity terrain export, 4000 m x 4000 m, 702 m high)
// Voxelization: the STL is converted to a cached SDF (smooth terrain, no vertical-slice artifacts)

#include "setup/setup.hpp"

void main_setup() { // extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS or GRAPHICS
	const Speed wind_speed = 10.0_kmh; // reference wind speed at the reference height
	const Length wind_reference_height = 100.0_m;
	const float32_t wind_profile_alpha = 0.25f; // power-law exponent for suburban terrain

	Simulation sim(Domain::around(Model("hill.stl")) // sized around the terrain; SDF voxelization by default
		.clearances(2_m, 500_m, 100_m) // below, above, on each side
		.cell_size(8_m)
		.vram_limit(20000_mb),
		Fluid::AIR, wind_speed, LatticeMach(0.121f)); // the original's lattice speed 0.07

	sim.boundaries()
		.set_solid_floor()
		.set_open_boundaries()
		.set_wind_profile_power_law(wind_speed, wind_reference_height, wind_profile_alpha)
		.set_wind_direction(Face::Y_MIN)
		.apply();

	sim.graphics()
		.show_surface()
		.show_vortices()
		.set_camera(CameraView::orbit(-40_deg, 25_deg).field_of_view(70_deg)) // orbits the domain center (above the terrain center)
		.apply();

	sim.run();
}
