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
	const float32_t aircraft_length_m = 62.0f;   // Full-scale CRM length
	const float32_t flow_velocity_mps = 80.0f;   // Approach speed (~155 knots)

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

	// Configure simulation with half-model mirroring
	SimulationSetup sim(SimulationConfig("crm-hl_reference_ldg.stl")
		.set_domain_aspect_ratio(1.0f, 1.5f, 1.0f / 3.0f)
		.set_vram_mb(2000u)
		.set_rotation_deg(0.0f, 0.0f, 90.0f)
		.set_mirror_plane(SimulationConfig::MirrorPlane::X)
		.set_angle_of_attack_deg(-10.0f));

	sim.setup();
	sim.configure_units_with_length(aircraft_length_m, flow_velocity_mps, Fluid::AIR);
	sim.print_reynolds_number(Fluid::AIR);

	// Create LBM
	LBM lbm = sim.create_lbm(Fluid::AIR);

	// Voxelize geometry (automatically mirrors the half-model)
	sim.voxelize(lbm);

	// Configure boundaries - all open with uniform velocity
	BoundaryBuilder(lbm)
		.set_all_open()
		.initialize_velocity_y(flow_velocity_mps)
		.apply();

	// Configure graphics
	GraphicsConfig(lbm)
		.show_surface()
		.show_vortices()
		.apply();

	// Run simulation (interactive mode)
	lbm.run();
}
