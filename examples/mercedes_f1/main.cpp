// Mercedes F1 W14 car
//
// Required extensions: FP16S, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SUBGRID
// Model from: https://downloadfree3d.com/3d-models/vehicles/sports-car/mercedes-f1-w14/
// Note: Requires manually separating body and wheels.
//
// Note: This example uses manual mesh handling for precise ground placement.
// Wheels are voxelized once with angular velocity (no re-voxelization needed).

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() {
	const float32_t car_length_m = 5.5f;
	const float32_t car_width_m = 2.0f;
	const float32_t car_speed_mps = 100.0f / 3.6f;  // 100 km/h
	const float32_t simulation_time_s = 0.25f;

	// Check for required STL files
	const string body_path = get_resource_path("mercedesf1-body.stl");
	const string front_path = get_resource_path("mercedesf1-front-wheels.stl");
	const string back_path = get_resource_path("mercedesf1-back-wheels.stl");
	if(body_path.empty() || front_path.empty() || back_path.empty()) {
		print_info("This example requires manually preparing Mercedes F1 W14 model.");
		print_info("Steps:");
		print_info("  1. Download from https://downloadfree3d.com/3d-models/vehicles/sports-car/mercedes-f1-w14/");
		print_info("  2. Open in Microsoft 3D Builder");
		print_info("  3. Separate body and wheels into 3 meshes");
		print_info("  4. Remove decals, convert to .stl");
		print_info("  5. Edit geometry: remove front wheel fenders, adjust right back wheel");
		print_info("  6. Save as mercedesf1-body.stl, mercedesf1-front-wheels.stl, mercedesf1-back-wheels.stl");
		print_info("  7. Place all 3 files in resources/");
		wait();
		return;
	}

	// Configure domain using body geometry (Y axis = car length)
	SimulationSetup sim(SimulationConfig("mercedesf1-body.stl")
		.set_domain_aspect_ratio(1.0f, 2.0f, 0.5f)
		.set_vram_mb(4000u)
		.set_geometry_scale(0.8f)
		.set_reference_axis(SimulationConfig::ReferenceAxis::Y));

	sim.setup();
	sim.configure_units_with_length(car_length_m, car_speed_mps, Fluid::AIR);

	// Print Reynolds number based on car width
	const float32_t Re = units.si_Re(car_width_m, car_speed_mps, Fluid::AIR.kinematic_viscosity);
	print_info("Re = " + to_string(to_uint(Re)));

	LBM lbm = sim.create_lbm(Fluid::AIR);
	const float32_t scale = sim.get_mesh_scale_factor();

	// Load all meshes and apply same scale
	Mesh* body = read_stl(body_path);
	Mesh* front_wheels = read_stl(front_path);
	Mesh* back_wheels = read_stl(back_path);

	body->scale(scale);
	front_wheels->scale(scale);
	back_wheels->scale(scale);

	// Calculate offset to position car with wheels touching floor (z=4 cells clearance)
	const float3 offset = float3(
		lbm.center().x - body->get_bounding_box_center().x,
		1.0f - body->pmin.y + 0.25f * back_wheels->get_min_size(),
		4.0f - back_wheels->pmin.z
	);

	body->translate(offset);
	front_wheels->translate(offset);
	back_wheels->translate(offset);

	body->set_center(body->get_center_of_mass());
	front_wheels->set_center(front_wheels->get_center_of_mass());
	back_wheels->set_center(back_wheels->get_center_of_mass());

	// Calculate wheel angular velocity (omega = v / r)
	const float32_t lbm_u = sim.to_lbm_velocity(car_speed_mps);
	const float32_t lbm_radius = 0.5f * back_wheels->get_min_size();
	const float32_t omega = lbm_u / lbm_radius;

	// Voxelize all parts (wheels with angular velocity - static, no re-voxelization needed)
	lbm.voxelize_mesh_on_device(body);
	lbm.voxelize_mesh_on_device(front_wheels, TYPE_S, front_wheels->get_center(), float3(0.0f), float3(omega, 0.0f, 0.0f));
	lbm.voxelize_mesh_on_device(back_wheels, TYPE_S, back_wheels->get_center(), float3(0.0f), float3(omega, 0.0f, 0.0f));

	// Configure boundaries - solid floor, open elsewhere
	BoundaryBuilder(lbm)
		.set_solid_floor()
		.set_open_boundaries()
		.initialize_velocity_y(car_speed_mps)
		.apply();

	// Configure graphics
	GraphicsConfig(lbm)
		.show_surface()
		.show_vortices()
		.apply();

	// Run simulation
	const uint64_t lbm_T = sim.to_lbm_timesteps(simulation_time_s);
	print_info(to_string(simulation_time_s, 2u) + " seconds = " + to_string(lbm_T) + " time steps");

#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
	VideoRecorder()
		.add("a", CameraConfig()
			.set_free_position(0.779346f, -0.315650f, 0.329444f)
			.set_angles(-27.0f, 19.0f)
			.set_fov(100.0f))
		.add("b", CameraConfig()
			.set_free_position(0.556877f, 0.228191f, 1.159613f)
			.set_angles(19.0f, 53.0f)
			.set_fov(100.0f))
		.add("c", CameraConfig()
			.set_free_position(0.220650f, -0.589529f, 0.085407f)
			.set_angles(-72.0f, 16.0f)
			.set_fov(86.0f))
		.set_video_length_s(30.0f)
		.record(lbm, simulation_time_s, units);
#else
	lbm.run();
#endif

	delete body;
	delete front_wheels;
	delete back_wheels;
}
