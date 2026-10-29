/// @file main.cpp
/// @brief City wind: an atmospheric boundary layer (a power-law wind profile) over a 1 km city block, recorded from
/// three cameras.
///
/// Extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID
/// STL from: resources/city.stl

#include "setup/setup.hpp"

/// @brief The video's cameras: their positions, as parts of the domain's width from the domain's origin
/// corner, and their directions, as the original.
class Cameras {
public:
	/// @brief Adds the cameras to the video.
	/// @param video the simulation's video recorder
	/// @param reference the domain's width: the camera positions are parts of it
	static void add_to(VideoRecorder& video, si::Length reference) {
		using namespace si::symbols;
		const si::Length W = reference; // camera positions from the domain's origin corner
		const si::Position camera_a{ -0.588245 * W, 0.112162 * W, 1.10899 * W };
		const si::Position camera_b{ 0.703233 * W, 1.07265 * W, 0.4675 * W };
		const si::Position camera_c{ 0.216499 * W, 0.800642 * W, 0.337734 * W };
		const si::Angle a_azimuth = 215.0 * deg;
		const si::Angle a_elevation = 39.0 * deg;
		const si::Angle a_field_of_view = 70.0 * deg;
		const si::Angle b_azimuth = 56.0 * deg;
		const si::Angle b_elevation = 45.0 * deg;
		const si::Angle b_field_of_view = 105.0 * deg;
		const si::Angle c_azimuth = 234.0 * deg;
		const si::Angle c_elevation = 29.0 * deg;
		const si::Angle c_field_of_view = 117.0 * deg;
		const CameraView view_a = CameraView::at(camera_a, a_azimuth, a_elevation).field_of_view(a_field_of_view);
		const CameraView view_b = CameraView::at(camera_b, b_azimuth, b_elevation).field_of_view(b_field_of_view);
		const CameraView view_c = CameraView::at(camera_c, c_azimuth, c_elevation).field_of_view(c_field_of_view);
		video.add("a", view_a);
		video.add("b", view_b);
		video.add("c", view_c);
	}
};

/// @brief Sets up the city (85 % of the domain's length) in 512 x 1024 x 256 cells with a 10 m/s wind at 100 m from
/// the south, and records a 30 s video of 1 minute of simulated time.
///
/// The lattice Mach number gives the original's lattice speed 0.07.
void main_setup() {
	using namespace si::symbols;
	const si::Length city_size = 1000.0 * m;       // city block size
	const si::Speed wind_speed = 10.0 * m / s;     // at the reference height
	const si::Length reference_height = 100.0 * m; // of the wind profile
	const double urban_exponent = 0.25;            // the power law's exponent for urban and suburban terrain
	const si::Duration simulation_time = au::minutes(1.0);
	const si::Duration video_length = 30.0 * s;

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

	sim.graphics()
		.show_surface()
		.show_vortices()
		.apply();

	VideoRecorder& video = sim.video();
	Cameras::add_to(video, domain_width);
	video.set_length(video_length);
	sim.run_for(simulation_time);
}
