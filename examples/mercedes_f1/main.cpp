/// @file main.cpp
/// @brief Mercedes F1 W14 car: the airflow at 100 km/h around the car on the ground, with turning wheels, recorded
/// from four cameras.
///
/// Extensions: FP16S, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SUBGRID
/// Model from: https://downloadfree3d.com/3d-models/vehicles/sports-car/mercedes-f1-w14/
/// Note: Requires manually separating body and wheels.
///
/// This example places the meshes by hand for a precise ground contact. The wheels are voxelized once with their
/// angular velocity (no re-voxelization).

#include "setup/setup.hpp"
#include "setup/domain/model_placement.hpp"

#include <memory>

/// @brief The car's three meshes (the body, the front wheels and the back wheels), placed by hand on the floor and
/// voxelized once, the wheels with their angular velocity.
class HandPlacedCar {
public:
	/// @brief Loads the three meshes from resources/ and scales them to the lattice.
	/// @param scale the cells per STL unit, as the body's model placement (ModelPlacement::cells_per_unit())
	explicit HandPlacedCar(float scale)
		: body_(read_stl(get_resource_path("mercedesf1-body.stl")))
		, front_wheels_(read_stl(get_resource_path("mercedesf1-front-wheels.stl")))
		, back_wheels_(read_stl(get_resource_path("mercedesf1-back-wheels.stl"))) {
		body_->scale(scale);
		front_wheels_->scale(scale);
		back_wheels_->scale(scale);
	}

	/// @brief Moves the three meshes together, so that the car is centred across the domain, its front is near the
	/// inlet and its back wheels touch the floor's top, and puts each mesh's centre at its centre of mass.
	/// @param lbm the lattice (its centre, in cells)
	void place(const LBM& lbm) {
		const float offset_x = lbm.center().x - body_->get_bounding_box_center().x;
		const float wheel_size = back_wheels_->get_min_size();
		const float offset_y = front_gap_cells - body_->pmin.y + front_gap_in_wheel_sizes * wheel_size;
		const float offset_z = floor_top_cells - back_wheels_->pmin.z;
		const float3 offset(offset_x, offset_y, offset_z);
		body_->translate(offset);
		front_wheels_->translate(offset);
		back_wheels_->translate(offset);
		body_->set_center(body_->get_center_of_mass());
		front_wheels_->set_center(front_wheels_->get_center_of_mass());
		back_wheels_->set_center(back_wheels_->get_center_of_mass());
	}

	/// @brief Voxelizes the body as a solid and the wheels as solids that turn with the car's speed on their tread
	/// (omega = v / r, in lattice units). Call place() first.
	/// @param lbm the lattice
	/// @param wheel_speed the car's speed in lattice units: the speed of the wheels' tread
	void voxelize(LBM& lbm, float wheel_speed) const {
		const float wheel_radius = 0.5f * back_wheels_->get_min_size();
		const float omega = wheel_speed / wheel_radius;
		const float3 no_velocity(0.0f);
		const float3 wheel_rotation(omega, 0.0f, 0.0f);
		lbm.voxelize_mesh_on_device(body_.get());
		lbm.voxelize_mesh_on_device(front_wheels_.get(), TYPE_S, front_wheels_->get_center(), no_velocity, wheel_rotation);
		lbm.voxelize_mesh_on_device(back_wheels_.get(), TYPE_S, back_wheels_->get_center(), no_velocity, wheel_rotation);
	}

private:
	static constexpr float front_gap_cells = 1.0f;           ///< the body's front from the inlet, in cells (lattice units), before the wheel part below
	static constexpr float front_gap_in_wheel_sizes = 0.25f; ///< the added gap to the inlet, in the back wheels' smallest extents (a ratio)
	static constexpr float floor_top_cells = 4.0f;           ///< the back wheels' lowest point, in cells (lattice units): the floor's top

	std::unique_ptr<Mesh> body_{};         ///< the body's mesh, in cells
	std::unique_ptr<Mesh> front_wheels_{}; ///< the front wheels' mesh, in cells
	std::unique_ptr<Mesh> back_wheels_{};  ///< the back wheels' mesh, in cells
};

/// @brief The video's cameras: their positions, as parts of the domain's length from the domain's origin
/// corner, and their directions, as the original.
class Cameras {
public:
	/// @brief Adds the cameras to the video.
	/// @param video the simulation's video recorder
	/// @param reference the domain's length: the camera positions are parts of it
	static void add_to(VideoRecorder& video, si::Length reference) {
		using namespace si::symbols;
		const si::Length D = reference; // camera positions from the domain's origin corner
		const si::Position camera_a{ 0.639673 * D, 0.18435 * D, 0.207361 * D };
		const si::Position camera_b{ 0.528439 * D, 0.728191 * D, 0.414903 * D };
		const si::Position camera_c{ 0.360325 * D, -0.089529 * D, 0.146352 * D };
		const si::Angle a_azimuth = -27.0 * deg;
		const si::Angle a_elevation = 19.0 * deg;
		const si::Angle b_azimuth = 19.0 * deg;
		const si::Angle b_elevation = 53.0 * deg;
		const si::Angle c_azimuth = -72.0 * deg;
		const si::Angle c_elevation = 16.0 * deg;
		const si::Angle c_field_of_view = 86.0 * deg;
		const CameraView view_a = CameraView::at(camera_a, a_azimuth, a_elevation);
		const CameraView view_b = CameraView::at(camera_b, b_azimuth, b_elevation);
		const CameraView view_c = CameraView::at(camera_c, c_azimuth, c_elevation).field_of_view(c_field_of_view);
		const double orbit_height_divisor = 1.648722; // the circling camera's view height is the domain length over this
		const si::Length orbit_height = D / orbit_height_divisor;
		const si::Angle start_azimuth = 75.0 * deg; // in front of the car
		const si::Angle sweep = 235.0 * deg;        // to its rear, during the video
		const si::Angle circling_elevation = -5.0 * deg;
		const auto circling = [=](float progress) { // circles the car from its front to its rear during the video, as the original
			const double progress_ratio = static_cast<double>(progress);
			const si::Angle azimuth = start_azimuth - progress_ratio * sweep;
			const CameraView view = CameraView::orbit(azimuth, circling_elevation).view_height(orbit_height);
			return view;
		};
		video.add("a", view_a);
		video.add("b", view_b);
		video.add("c", view_c);
		video.add("d", circling);
	}
};

/// @brief Sets up the car (5.5 m long, 80 % of the domain) in a 4000 MiB grid, places its body and wheels on the
/// floor, and records a 30 s video of 0.25 s of simulated time.
///
/// Ends the program with print_error() and the preparation steps when a mesh is missing from resources/.
void main_setup() {
	using namespace si::symbols;
	const si::Length car_length = 5.5 * m;
	const si::Length car_width = 2.0 * m;
	const si::Speed car_speed = 100.0 * kmh;
	const si::Duration simulation_time = 0.25 * s;
	const si::Duration video_length = 30.0 * s;

	// the car (its length along Y) is 80 % of the domain length
	const double car_in_length = 0.8;
	const si::Length domain_length = car_length / car_in_length;
	const si::Length domain_width = 0.5 * domain_length;
	const double height_in_lengths = 0.25;
	const si::Length domain_height = height_in_lengths * domain_length;
	const si::MemoryAmount vram = 4000.0 * MiB;
	Model car("mercedesf1-body.stl");
	car.length(car_length);
	car.needs({ "mercedesf1-front-wheels.stl", "mercedesf1-back-wheels.stl" });
	car.instructions({ "This example requires manually preparing Mercedes F1 W14 model.",
	                   "Steps:",
	                   "  1. Download from https://downloadfree3d.com/3d-models/vehicles/sports-car/mercedes-f1-w14/",
	                   "  2. Open in Microsoft 3D Builder",
	                   "  3. Separate body and wheels into 3 meshes",
	                   "  4. Remove decals, convert to .stl",
	                   "  5. Edit geometry: remove front wheel fenders, adjust right back wheel",
	                   "  6. Save as mercedesf1-body.stl, mercedesf1-front-wheels.stl, mercedesf1-back-wheels.stl",
	                   "  7. Place all 3 files in resources/" });
	Domain domain = Domain::around(car);
	domain.size(domain_width, domain_length, domain_height);
	domain.vram(vram);
	Simulation sim(domain, Fluid::AIR, car_speed);
	sim.skip_model_voxelization(); // the body and the wheels are voxelized by hand below
	const double reynolds = au::as_raw_number(car_width * car_speed / Fluid::AIR.kinematic_viscosity);
	const uint reynolds_whole = to_uint(static_cast<float>(reynolds));
	const std::string reynolds_text = to_string(reynolds_whole);
	print_info("Re over the car's width = " + reynolds_text);

	// the body and the wheels, in cells, with the same scale as the body's model placement
	const float scale = ModelPlacement::of(sim.plan()).cells_per_unit(); // cells per STL unit, as the body's
	HandPlacedCar hand_placed(scale);
	hand_placed.place(sim.lbm());
	const float wheel_speed = sim.converter().velocity(car_speed);
	hand_placed.voxelize(sim.lbm(), wheel_speed);

	// solid floor, open elsewhere
	sim.boundaries()
		.set_solid_floor()
		.set_open_boundaries()
		.initialize_velocity_y(car_speed)
		.apply();

	sim.graphics()
		.show_surface()
		.show_vortices()
		.apply();

	VideoRecorder& video = sim.video();
	Cameras::add_to(video, domain_length);
	video.set_length(video_length);
	sim.run_for(simulation_time);
}
