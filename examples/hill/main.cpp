// Wind over hill terrain with an atmospheric boundary layer profile, using Setup API
// Terrain: resources/hill.stl (Unity terrain export, 4000 m x 4000 m, 702 m high)
// Voxelization: the STL is converted to a cached SDF (smooth terrain, no vertical-slice artifacts)

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() { // required extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS or GRAPHICS
	const float32_t wind_speed_mps = 10.0f/3.6f; // reference wind speed (10 km/h) at the reference height
	const float32_t wind_reference_height_m = 100.0f;
	const float32_t wind_profile_alpha = 0.25f; // power-law exponent for suburban terrain

	SimulationSetup sim(Domain::around(Model("hill.stl")) // sized around the terrain; SDF voxelization by default
		.clearances(2_m, 500_m, 100_m) // below, above, on each side
		.cell_size(8_m)
		.max_vram(20000_mb));

	sim.setup();
	sim.configure_units(wind_speed_mps, Fluid::AIR, 0.07f);
	sim.print_reynolds_number(Fluid::AIR);

	LBM lbm = sim.create_lbm(Fluid::AIR);
	sim.voxelize(lbm);

	BoundaryBuilder(lbm)
		.set_solid_floor()
		.set_open_boundaries()
		.set_wind_profile_power_law(wind_speed_mps, wind_reference_height_m, wind_profile_alpha)
		.set_wind_direction(Face::Y_MIN)
		.apply();

	GraphicsConfig(lbm)
		.show_surface()
		.show_vortices()
		.apply();
	lbm.graphics.set_camera_centered(-40.0f, 25.0f, 70.0f, 1.0f); // orbits the domain center (above the terrain center)

	lbm.run();
}
