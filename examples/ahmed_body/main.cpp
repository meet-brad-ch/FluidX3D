// Ahmed body aerodynamics with drag coefficient calculation
//
// Required extensions: FP16C, FORCE_FIELD, EQUILIBRIUM_BOUNDARIES, SUBGRID
//
// STL file required: ahmed_25deg_m.stl
// Download from: https://github.com/nathanrooy/ahmed-bluff-body-cfd/blob/master/geometry/ahmed_25deg_m.stl
// Note: The downloaded file is ASCII format - convert to binary STL before use.

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() {
	const float32_t flow_velocity_mps = 60.0f;
	const float32_t simulation_time_s = 0.25f;
	const float32_t frontal_area_m2 = 0.389f * 0.288f + 2.0f * 0.05f * 0.03f;

	SimulationSetup sim(Domain::around(Model("ahmed_25deg_m.stl").rotation(0_deg, 0_deg, 90_deg))
		.clearances(0_m, 1_m, 2.6_m) // below, above, on each side
		.vram(10000_mb));

	sim.setup();
	sim.configure_units(flow_velocity_mps, Fluid::AIR, 0.05f);
	sim.enable_force_tracking();
	sim.print_reynolds_number(Fluid::AIR);

	const auto& r = sim.get_results();
	LBM lbm(r.Nx, r.Ny, r.Nz, sim.to_lbm_viscosity(Fluid::AIR.kinematic_viscosity));
	sim.voxelize(lbm);

	BoundaryBuilder(lbm)
		.set_solid_floor()
		.set_open_boundaries()
		.initialize_velocity_y(flow_velocity_mps)
		.apply();

	GraphicsConfig(lbm)
		.show_surface()
		.show_density_field()
		.set_slice_mode(SliceMode::X)
		.apply();

#ifdef FORCE_FIELD
	ForceAnalyzer forces(lbm);
	forces.set_reference_area(frontal_area_m2)
	      .set_reference_velocity(flow_velocity_mps)
	      .set_fluid_density(Fluid::AIR.density)
	      .set_flow_direction(Axis::Y);

	print_info("Center of mass: " + to_string(forces.get_center_of_mass_lbm().x, 2u) + ", " +
	           to_string(forces.get_center_of_mass_lbm().y, 2u) + ", " +
	           to_string(forces.get_center_of_mass_lbm().z, 2u));
#endif

	const uint64_t lbm_T = sim.to_lbm_timesteps(simulation_time_s);
	lbm.run(0u, lbm_T);

	while(lbm.get_t() <= lbm_T) {
#ifdef FORCE_FIELD
		Clock clock;
		print_info("Cd = " + to_string(forces.get_drag_coefficient(), 3u) + ", t = " + to_string(clock.stop(), 3u));
#endif
		lbm.run(1u, lbm_T);
	}
}
