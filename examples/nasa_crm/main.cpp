/// @file main.cpp
/// @brief The NASA Common Research Model (CRM) in its high-lift form: the airflow around the full aircraft at 80 m/s
/// and -10 degrees angle of attack (interactive graphics).
///
/// Extensions: FP16C, EQUILIBRIUM_BOUNDARIES, SUBGRID
/// Model from: https://commonresearchmodel.larc.nasa.gov/high-lift-crm/
///
/// A half-model STL, mirrored into the full symmetric aircraft (Model::mirrored()).

#include "setup/setup.hpp"

/// @brief Sets up the full-scale aircraft (62 m long) in a 2000 MiB grid as long as it, and runs until the window
/// is closed.
///
/// Ends the program with print_error() and the download instructions when the STL is missing from resources/.
void main_setup() {
	using namespace si::symbols;
	const si::Length aircraft_length = 62.0 * m;   // full-scale CRM length
	const si::Speed flow_velocity = 80.0 * m / s;  // approach speed (about 155 knots)
	const si::Angle yaw = 90.0 * deg;              // turns the model's length along Y
	const si::Angle angle_of_attack = -10.0 * deg;

	// half model, mirrored into the full aircraft; the aircraft (Y) is as long as the domain
	const double lengths_per_width = 1.5;  // the domain is 1.5 times as long as it is wide
	const double lengths_per_height = 4.5; // the domain is 4.5 times as long as it is high
	const si::Length domain_width = aircraft_length / lengths_per_width;
	const si::Length domain_height = aircraft_length / lengths_per_height;
	const si::MemoryAmount vram = 2000.0 * MiB;
	Model half_model("crm-hl_reference_ldg.stl");
	half_model.rotation(0.0 * deg, 0.0 * deg, yaw).angle_of_attack(angle_of_attack).length(aircraft_length).mirrored(Axis::X);
	half_model.instructions({ "This example requires the NASA Common Research Model (CRM) high-lift geometry.",
	                          "Steps:",
	                          "  1. Download .stp file from https://commonresearchmodel.larc.nasa.gov/high-lift-crm/high-lift-crm-geometry/assembled-geometry/",
	                          "  2. Convert .stp to .stl using https://imagetostl.com/convert/file/stp/to/stl",
	                          "  3. Save as crm-hl_reference_ldg.stl in resources/" });
	Domain domain = Domain::around(half_model);
	domain.size(domain_width, aircraft_length, domain_height);
	domain.vram(vram);
	Simulation sim(domain, Fluid::AIR, flow_velocity);

	sim.boundaries()
		.set_open_boundaries()
		.initialize_velocity_y(flow_velocity)
		.apply();

	sim.graphics()
		.show_surface()
		.show_vortices()
		.apply();

	sim.run();
}
