// Ahmed body aerodynamics with drag coefficient calculation
//
// Extensions: FP16C, FORCE_FIELD, EQUILIBRIUM_BOUNDARIES, SUBGRID
//
// STL file required: ahmed_25deg_m.stl
// Download from: https://github.com/nathanrooy/ahmed-bluff-body-cfd/blob/master/geometry/ahmed_25deg_m.stl
// Note: The downloaded file is ASCII format - convert to binary STL before use.

#include "setup/setup.hpp"

void main_setup() {
	const Speed flow_velocity = 60.0_mps;
	const Duration simulation_time = 0.25_s;
	const Length body_width = 0.389_m, body_height = 0.288_m, body_length = 1.044_m;
	const Area frontal_area = body_width * body_height + 2.0f * 0.05_m * 0.03_m; // with the stilts

	// as the original: 6 body widths wide, 6 body lengths long, 2.5 widths of air above the body; its front 1.37 m from
	// the inlet, on the floor
	Simulation sim(Domain::around(Model("ahmed_25deg_m.stl").rotation(0_deg, 0_deg, 90_deg).length(body_length))
		.size(6.0f * body_width, 6.0f * body_length, 2.5f * body_width + body_height)
		.gap_to_inlet(0.5f * (3.0f * body_length - body_width))
		.on_floor()
		.vram(10000_mb),
		Fluid::AIR, flow_velocity, LatticeMach(0.0866f)); // the original's lattice speed 0.05
	sim.measure_forces();

	sim.boundaries()
		.set_solid_floor()
		.set_open_boundaries()
		.initialize_velocity_y(flow_velocity)
		.apply();

	sim.graphics()
		.show_surface()
		.show_density_field()
		.set_slice_mode(SliceMode::X)
		.apply();

	ForceAnalyzer& forces = sim.forces()
		.set_reference_area(frontal_area)
		.set_reference_velocity(flow_velocity)
		.set_fluid(Fluid::AIR)
		.set_flow_direction(Axis::Y);
	const Position center_of_mass = forces.center_of_mass();
	print_info("Center of mass: " + to_string(center_of_mass.x.si(), 3u) + " m, " + to_string(center_of_mass.y.si(), 3u) + " m, " +
	           to_string(center_of_mass.z.si(), 3u) + " m");

	sim.every_step([&](Duration) {
		Clock clock;
		print_info("Cd = " + to_string(forces.drag_coefficient(), 3u) + ", t = " + to_string(clock.stop(), 3u));
	});
	sim.run_for(simulation_time);
}
