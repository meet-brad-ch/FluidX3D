/// @file main.cpp
/// @brief City wind in real time: an atmospheric boundary layer over a city, slow enough (1 km/h) to watch it
/// develop (interactive graphics).
///
/// Extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS
/// STL from: resources/city.stl (the same city as the city example)

#include "setup/setup.hpp"

/// @brief Sets up the city example's domain with a 1 km/h wind at 100 m from the south and runs until the window is
/// closed: rotate with the mouse, start and pause with P.
///
/// The lattice Mach number gives the original's lattice speed 0.07.
void main_setup() {
	using namespace si::symbols;
	const si::Length city_size = 1000.0 * m;       // city block size
	const si::Speed wind_speed = 1.0 * kmh;        // at the reference height
	const si::Length reference_height = 100.0 * m; // of the wind profile
	const double urban_exponent = 0.25;            // the power law's exponent for urban and suburban terrain

	// the city (its size along Y) is 85 % of the domain length
	const double city_in_width = 1.7;      // the city is 1.7 domain widths: 85 % of the domain's length of 2 widths
	const double cells_across = 512.0;     // 512 x 1024 x 256 cells, as the original
	const si::Length domain_width = city_size / city_in_width;
	const si::Length domain_length = 2.0 * domain_width;
	const si::Length domain_height = 0.5 * domain_width;
	const double offset_y_in_sizes = -0.05;  // the model's offset along Y from the domain's middle, in city sizes
	const double offset_z_in_sizes = -0.025; // the model's offset along Z from the domain's middle, in city sizes
	const si::Length model_offset_y = offset_y_in_sizes * city_size;
	const si::Length model_offset_z = offset_z_in_sizes * city_size;
	const si::Length cell = domain_width / cells_across;
	const si::Angle yaw = 90.0 * deg; // turns the model's size along Y
	Model model("city.stl");
	model.rotation(0.0 * deg, 0.0 * deg, yaw).length(city_size);
	Domain domain = Domain::around(model);
	domain.size(domain_width, domain_length, domain_height);
	domain.model_offset(0.0 * m, model_offset_y, model_offset_z);
	domain.cell_size(cell);
	const si::LatticeMach mach(0.121); // the original's lattice speed 0.07
	Simulation sim(domain, Fluid::AIR, wind_speed, mach);

	// power-law profile u(z) = u_ref*(z/z_ref)^alpha
	sim.boundaries()
		.set_solid_floor()
		.set_open_boundaries()
		.set_wind_profile_power_law(wind_speed, reference_height, urban_exponent)
		.set_wind_direction(Face::Y_MIN)
		.apply();

	const si::Length W = domain_width; // the camera from the domain's origin corner
	const si::Position camera_position{ -0.588245 * W, 0.112162 * W, 1.10899 * W };
	const si::Angle camera_azimuth = 215.0 * deg;
	const si::Angle camera_elevation = 39.0 * deg;
	const si::Angle camera_field_of_view = 70.0 * deg;
	const CameraView camera_view = CameraView::at(camera_position, camera_azimuth, camera_elevation).field_of_view(camera_field_of_view);
	sim.graphics()
		.show_surface()
		.show_vortices()
		.set_camera(camera_view)
		.apply();

	sim.run(); // interactive: rotate with the mouse, start and pause with P
}
