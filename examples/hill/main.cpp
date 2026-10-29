/// @file main.cpp
/// @brief Wind over hill terrain: an atmospheric boundary layer (a power-law wind profile) over a 4 x 4 km terrain
/// 702 m high (interactive or video graphics).
///
/// Terrain: resources/hill.stl (a terrain export in metres). Voxelization: the STL is converted to a cached SDF
/// (smooth terrain, no vertical-slice artifacts).

#include "setup/setup.hpp"

/// @brief Sets up the terrain with clearances around it in 8 m cells (at most 20000 MiB) and a 10 km/h wind at 100 m
/// from the south, and runs until the window is closed.
///
/// The lattice Mach number gives the original's lattice speed 0.07.
void main_setup() { // extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS or GRAPHICS
	using namespace si::symbols;
	const si::Speed wind_speed = 10.0 * kmh; // reference wind speed at the reference height
	const si::Length wind_reference_height = 100.0 * m;
	const double wind_profile_alpha = 0.25; // power-law exponent for suburban terrain
	const si::Length clearance_below = 2.0 * m;
	const si::Length clearance_above = 500.0 * m;
	const si::Length clearance_side = 100.0 * m; // on each side
	const si::Length cell = 8.0 * m;
	const si::MemoryAmount vram_limit = 20000.0 * MiB;

	const Model terrain("hill.stl");
	Domain domain = Domain::around(terrain); // sized around the terrain; SDF voxelization by default
	domain.clearances(clearance_below, clearance_above, clearance_side);
	domain.cell_size(cell);
	domain.vram_limit(vram_limit);
	const si::LatticeMach mach(0.121); // the original's lattice speed 0.07
	Simulation sim(domain, Fluid::AIR, wind_speed, mach);

	sim.boundaries()
		.set_solid_floor()
		.set_open_boundaries()
		.set_wind_profile_power_law(wind_speed, wind_reference_height, wind_profile_alpha)
		.set_wind_direction(Face::Y_MIN)
		.apply();

	const si::Angle camera_azimuth = -40.0 * deg;
	const si::Angle camera_elevation = 25.0 * deg;
	const si::Angle camera_field_of_view = 70.0 * deg;
	const CameraView camera_view = CameraView::orbit(camera_azimuth, camera_elevation).field_of_view(camera_field_of_view); // orbits the domain center (above the terrain center)
	sim.graphics()
		.show_surface()
		.show_vortices()
		.set_camera(camera_view)
		.apply();

	sim.run();
}
