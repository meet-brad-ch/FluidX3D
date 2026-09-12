// Electric Ducted Fan (EDF)
//
// Required extensions: FP16S, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SUBGRID
// STL from: https://www.thingiverse.com/thing:3014759/files
//
// Note: This example uses MovingPartsManager for the rotor.
// Rotor has different Y offset (-0.41) compared to stator (-0.2).

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() {
	const Length fan_diameter = 0.09_m;          // 90 mm EDF
	const float32_t tip_speed_mps = 100.0f;      // Blade tip speed
	const float32_t inlet_velocity_mps = 30.0f;  // 30% of tip speed
	const float32_t simulation_time_s = 0.5f;
	const uint32_t update_interval = 4u;

	// the stator (its size along Y) is 98 % of the domain length
	const Length domain_length = fan_diameter / 0.98f;
	SimulationSetup sim(Domain::around(Model("edf_v39.stl").rotation(0_deg, 0_deg, 180_deg).length(fan_diameter))
		.size(domain_length / 1.5f, domain_length, domain_length / 1.5f)
		.model_offset(0_m, -0.2f * fan_diameter, 0_m) // stator position
		.vram(8000_mb));

	sim.setup();
	sim.configure_units(tip_speed_mps, Fluid::AIR);
	sim.print_reynolds_number(Fluid::AIR);

	// Create LBM
	LBM lbm = sim.create_lbm(Fluid::AIR);

	// Voxelize stator
	sim.voxelize(lbm);

	// Configure boundaries - open with inlet velocity
	BoundaryBuilder(lbm)
		.set_all_open()
		.initialize_velocity_y(inlet_velocity_mps)
		.apply();

	// Configure rotor with different Y offset (difference from stator: -0.41 - (-0.2) = -0.21)
	MovingPartsManager parts(sim, lbm);
	parts.add(MovingPart("edf_v391.stl")
		.set_rotation_axis(RotationAxis::Y)
		.set_tip_speed_mps(tip_speed_mps)
		.set_offset_ratio(0.0f, -0.21f, 0.0f)
		.set_update_interval(update_interval));
	parts.initialize();

	// Configure graphics
	GraphicsConfig(lbm)
		.show_surface()
		.show_vortices()
		.apply();

	// Run simulation
	const uint64_t total_timesteps = sim.to_lbm_timesteps(simulation_time_s);
	print_info(to_string(simulation_time_s, 2u) + " seconds = " + to_string(total_timesteps) + " time steps");

#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
	lbm.run(0u, total_timesteps);
	while(lbm.get_t() < total_timesteps) {
		parts.update();
		lbm.run(update_interval, total_timesteps);

		if(lbm.graphics.next_frame(total_timesteps, 30.0f)) {
			// Dynamic camera that pans during simulation
			const float32_t progress = (float32_t)lbm.get_t() / (float32_t)total_timesteps;
			lbm.graphics.set_camera_centered(-70.0f + 100.0f * progress, 2.0f, 60.0f, 1.284025f);
			lbm.graphics.write_frame();
		}
	}
#else
	parts.run(simulation_time_s, units);
#endif
}
