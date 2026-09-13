// NASA Common Research Model (CRM)
//
// Extensions: FP16C, EQUILIBRIUM_BOUNDARIES, SUBGRID
// Model from: https://commonresearchmodel.larc.nasa.gov/high-lift-crm/
//
// A half-model STL, mirrored into the full symmetric aircraft (Model::mirrored()), at -10 degrees angle of attack.

#include "setup/setup.hpp"

void main_setup() {
	const Length aircraft_length = 62.0_m; // full-scale CRM length
	const Speed flow_velocity = 80.0_mps;  // approach speed (about 155 knots)

	// Check for the required STL file
	if(get_resource_path("crm-hl_reference_ldg.stl").empty()) {
		print_info("This example requires the NASA Common Research Model (CRM) high-lift geometry.");
		print_info("Steps:");
		print_info("  1. Download .stp file from https://commonresearchmodel.larc.nasa.gov/high-lift-crm/high-lift-crm-geometry/assembled-geometry/");
		print_info("  2. Convert .stp to .stl using https://imagetostl.com/convert/file/stp/to/stl");
		print_info("  3. Save as crm-hl_reference_ldg.stl in resources/");
		wait();
		return;
	}

	// half model, mirrored into the full aircraft; the aircraft (Y) is as long as the domain
	Simulation sim(Domain::around(Model("crm-hl_reference_ldg.stl").rotation(0_deg, 0_deg, 90_deg).angle_of_attack(-10_deg)
			.length(aircraft_length).mirrored(Axis::X))
		.size(aircraft_length / 1.5f, aircraft_length, aircraft_length / 4.5f)
		.vram(2000_mb),
		Fluid::AIR, flow_velocity);

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
