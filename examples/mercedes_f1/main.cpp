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
	const Length car_length = 5.5_m;
	const float32_t car_width_m = 2.0f;
	const Speed car_speed = 100.0_kmh;
	const Duration simulation_time = 0.25_s;

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

	// the car (its length along Y) is 80 % of the domain length
	const Length domain_length = car_length / 0.8f;
	SimulationSetup sim(Domain::around(Model("mercedesf1-body.stl").length(car_length))
		.size(0.5f * domain_length, domain_length, 0.25f * domain_length)
		.vram(4000_mb));

	sim.setup();
	sim.configure_units(car_speed, Fluid::AIR);

	// Print Reynolds number based on car width
	const float32_t Re = units.si_Re(car_width_m, car_speed.si(), Fluid::AIR.kinematic_viscosity.si());
	print_info("Re = " + to_string(to_uint(Re)));

	LBM lbm = sim.create_lbm(Fluid::AIR);
	const float32_t scale = ModelPlacement::of(sim.get_results()).cells_per_unit(); // cells per STL unit, as the body's

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
	const float32_t lbm_u = sim.to_lbm_velocity(car_speed);
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
		.initialize_velocity_y(car_speed)
		.apply();

	// Configure graphics
	GraphicsConfig(lbm)
		.show_surface()
		.show_vortices()
		.apply();

	// Run simulation
#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
	const Length D = domain_length; // camera positions from the domain's origin corner
	VideoRecorder()
		.add("a", CameraView::at({ 0.639673f * D, 0.18435f * D, 0.207361f * D }, -27_deg, 19_deg))
		.add("b", CameraView::at({ 0.528439f * D, 0.728191f * D, 0.414903f * D }, 19_deg, 53_deg))
		.add("c", CameraView::at({ 0.360325f * D, -0.089529f * D, 0.146352f * D }, -72_deg, 16_deg).field_of_view(86_deg))
		.add("d", [D](float progress) { // circles the car from its front to its rear during the video, as the original
			return CameraView::orbit(75_deg - progress * 235_deg, -5_deg).view_height(D / 1.648721f);
		})
		.set_video_length(30.0_s)
		.record(lbm, simulation_time);
#else
	lbm.run();
#endif

	delete body;
	delete front_wheels;
	delete back_wheels;
}
