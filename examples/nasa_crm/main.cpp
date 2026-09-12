// NASA Common Research Model (CRM)
//
// Required extensions: FP16C, EQUILIBRIUM_BOUNDARIES, SUBGRID
// Model from: https://commonresearchmodel.larc.nasa.gov/high-lift-crm/
//
// Note: This example loads a half-model STL and automatically mirrors it
// to create the full symmetric aircraft using set_mirror_plane().
// The angle of attack is -10 degrees.

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() {
	const Length aircraft_length = 62.0_m;       // full-scale CRM length
	const Speed flow_velocity = 80.0_mps;        // Approach speed (~155 knots)

	// Check for required STL file
	const string stl_path = get_resource_path("crm-hl_reference_ldg.stl");
	if(stl_path.empty()) {
		print_info("This example requires the NASA Common Research Model (CRM) high-lift geometry.");
		print_info("Steps:");
		print_info("  1. Download .stp file from https://commonresearchmodel.larc.nasa.gov/high-lift-crm/high-lift-crm-geometry/assembled-geometry/");
		print_info("  2. Convert .stp to .stl using https://imagetostl.com/convert/file/stp/to/stl");
		print_info("  3. Save as crm-hl_reference_ldg.stl in resources/");
		wait();
		return;
	}

	// half model, mirrored into the full aircraft; the aircraft (Y) is as long as the domain
	SimulationSetup sim(Domain::around(Model("crm-hl_reference_ldg.stl").rotation(0_deg, 0_deg, 90_deg).angle_of_attack(-10_deg)
			.length(aircraft_length).mirrored(Axis::X))
		.size(aircraft_length / 1.5f, aircraft_length, aircraft_length / 4.5f)
		.vram(2000_mb));

	sim.setup();
	sim.configure_units(flow_velocity, Fluid::AIR);
	sim.print_reynolds_number(Fluid::AIR);

	// Create LBM
	LBM lbm = sim.create_lbm(Fluid::AIR);

	// Voxelize geometry (automatically mirrors the half-model)
	sim.voxelize(lbm);

	// Configure boundaries - all open with uniform velocity
	BoundaryBuilder(lbm)
		.set_all_open()
		.initialize_velocity_y(flow_velocity)
		.apply();

	// Configure graphics
	GraphicsConfig(lbm)
		.show_surface()
		.show_vortices()
		.apply();

	// Run simulation (interactive mode)
	lbm.run();
}
