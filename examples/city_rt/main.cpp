// City wind in real time: an atmospheric boundary layer over a city, slow enough to watch it develop, using Setup API
//
// Required extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS
// STL from: resources/city.stl (the same city as the city example)

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() {
	const Length city_size = 1000.0_m;       // city block size
	const Speed wind_speed = 1.0_kmh;        // at the reference height
	const Length reference_height = 100.0_m; // of the wind profile

	// the city (its size along Y) is 85 % of the domain length
	const Length domain_width = city_size / 1.7f;
	SimulationSetup sim(Domain::around(Model("city.stl").rotation(0_deg, 0_deg, 90_deg).length(city_size))
		.size(domain_width, 2.0f * domain_width, 0.5f * domain_width)
		.model_offset(0_m, -0.05f * city_size, -0.025f * city_size)
		.cell_size(domain_width / 512.0f)); // 512 x 1024 x 256 cells, as the original

	sim.setup();
	sim.configure_units(wind_speed, Fluid::AIR, LatticeMach(0.121f)); // the original's lattice speed 0.07
	sim.print_reynolds_number(Fluid::AIR);

	LBM lbm = sim.create_lbm(Fluid::AIR);
	sim.voxelize(lbm);

	// power-law profile u(z) = u_ref*(z/z_ref)^alpha, alpha = 0.25 for urban/suburban terrain
	BoundaryBuilder(lbm)
		.set_solid_floor()
		.set_open_boundaries()
		.set_wind_profile_power_law(wind_speed, reference_height, 0.25f)
		.set_wind_direction(Face::Y_MIN)
		.apply();

	const Length W = domain_width; // the camera from the domain's origin corner
	GraphicsConfig(lbm)
		.show_surface()
		.show_vortices()
		.set_camera(CameraView::at({ -0.588245f * W, 0.112162f * W, 1.10899f * W }, 215_deg, 39_deg).field_of_view(70_deg))
		.apply();

	lbm.run(); // interactive: rotate with the mouse, start and pause with P
}
