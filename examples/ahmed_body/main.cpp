/// @file main.cpp
/// @brief Ahmed body aerodynamics: the airflow at 60 m/s around the 25-degree Ahmed body on the floor, with its drag
/// coefficient printed every time step.
///
/// Extensions: FP16C, FORCE_FIELD, EQUILIBRIUM_BOUNDARIES, SUBGRID
///
/// STL file required: ahmed_25deg_m.stl
/// Download from: https://github.com/nathanrooy/ahmed-bluff-body-cfd/blob/master/geometry/ahmed_25deg_m.stl
/// Note: The downloaded file is ASCII format - convert to binary STL before use.

#include "setup/setup.hpp"

/// @brief The report of the force on the body: the body's centre of mass once, then the drag coefficient at every
/// time step with the wall time of its measurement.
class DragReport {
public:
	/// @brief Sets the force analysis's reference values (air, flow along Y) and prints the body's centre of mass. Ends
	/// the program with print_error() when the area or the speed is not more than 0 (ForceAnalyzer's setters).
	/// @param forces the simulation's force analysis; it must live as long as the report
	/// @param frontal_area the body's frontal area with the stilts, more than 0 m²
	/// @param flow_velocity the free-stream speed, more than 0 m/s
	DragReport(ForceAnalyzer& forces, si::Area frontal_area, si::Speed flow_velocity) : forces_(forces) {
		using namespace si::symbols;
		forces_.set_reference_area(frontal_area)
			.set_reference_velocity(flow_velocity)
			.set_fluid(Fluid::AIR)
			.set_flow_direction(Axis::Y);
		const si::Position center_of_mass = forces_.center_of_mass();
		const float x_m = static_cast<float>(center_of_mass.x.in(m));
		const float y_m = static_cast<float>(center_of_mass.y.in(m));
		const float z_m = static_cast<float>(center_of_mass.z.in(m));
		const std::string x_text = to_string(x_m, 3u);
		const std::string y_text = to_string(y_m, 3u);
		const std::string z_text = to_string(z_m, 3u);
		print_info("Center of mass: " + x_text + " m, " + y_text + " m, " + z_text + " m");
	}

	/// @brief Prints the drag coefficient of the current time step and the wall time in seconds that its measurement
	/// took.
	void print_step() const {
		const Clock clock;
		const float drag_coefficient = static_cast<float>(forces_.drag_coefficient());
		const std::string cd_text = to_string(drag_coefficient, 3u);
		const double seconds = clock.stop();
		const std::string time_text = to_string(seconds, 3u);
		print_info("Cd = " + cd_text + ", t = " + time_text);
	}

private:
	ForceAnalyzer& forces_; ///< the simulation's force analysis
};

/// @brief Sets up the body (1.044 m long) in a 10000 MiB grid as the original's domain, measures the force on it,
/// and runs 0.25 s of simulated time.
///
/// The frontal area includes the stilts; the lattice Mach number gives the original's lattice speed 0.05.
void main_setup() {
	using namespace si::symbols;
	const si::Speed flow_velocity = 60.0 * m / s;
	const si::Duration simulation_time = 0.25 * s;
	const si::Length body_width = 0.389 * m;
	const si::Length body_height = 0.288 * m;
	const si::Length body_length = 1.044 * m;
	const double stilt_count = 2.0;
	const si::Length stilt_height = 0.05 * m;   // the body's ground clearance
	const si::Length stilt_diameter = 0.03 * m;
	const si::Area stilts_area = stilt_count * stilt_height * stilt_diameter;
	const si::Area frontal_area = body_width * body_height + stilts_area; // with the stilts
	const si::Angle yaw = 90.0 * deg; // turns the model's length along Y

	// as the original: 6 body widths wide, 6 body lengths long, 2.5 widths of air above the body; its front 1.37 m from
	// the inlet, on the floor
	const double widths_across = 6.0;
	const double lengths_along = 6.0;
	const double widths_above = 2.5;
	const si::Length domain_width = widths_across * body_width;
	const si::Length domain_length = lengths_along * body_length;
	const si::Length domain_height = widths_above * body_width + body_height;
	const double inlet_gap_in_lengths = 3.0; // the gap is half of 3 body lengths less one body width: 1.37 m
	const si::Length inlet_gap = 0.5 * (inlet_gap_in_lengths * body_length - body_width);
	const si::MemoryAmount vram = 10000.0 * MiB;
	Model model("ahmed_25deg_m.stl");
	model.rotation(0.0 * deg, 0.0 * deg, yaw).length(body_length);
	Domain domain = Domain::around(model);
	domain.size(domain_width, domain_length, domain_height);
	domain.gap_to_inlet(inlet_gap);
	domain.on_floor();
	domain.vram(vram);
	const si::LatticeMach mach(0.0866); // the original's lattice speed 0.05
	Simulation sim(domain, Fluid::AIR, flow_velocity, mach);
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

	const DragReport report(sim.forces(), frontal_area, flow_velocity);
	sim.every_step([&](si::Duration) {
		report.print_step();
	});
	sim.run_for(simulation_time);
}
