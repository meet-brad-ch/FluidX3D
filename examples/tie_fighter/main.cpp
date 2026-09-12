// Star Wars TIE Fighter (tumbling)
//
// Required extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID
// STL from: https://www.thingiverse.com/thing:2919109/files
//
// Demonstrates tumbling simulation using MovingPartsManager with set_tumble().
// The mesh rotates around an arbitrary axis while flowing through the domain.

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() {
	const Length height = 8.8_m;                 // TIE/LN (Wookieepedia): the wing panels' height (Z), the longest side
	const Speed flow_velocity = 50.0_mps;        // Flow velocity
	const Duration simulation_time = 5.0_s;      // 5 seconds of tumbling
	const uint32_t update_interval = 28u;

	// the height (Z) is 65 % of the domain width; the fighter's center is about 0.6 heights from the inlet
	const Length domain_width = height / 0.65f;
	SimulationSetup sim(Domain::around(Model("DWG_Tie_Fighter_Assembled_02.stl").rotation(90_deg, 0_deg, 0_deg).length(height, Axis::Z))
		.size(domain_width, 2.0f * domain_width, domain_width)
		.model_offset(0_m, -0.94f * height, 0_m)
		.vram(1760_mb));

	sim.setup();
	sim.configure_units(flow_velocity, Fluid::AIR, 0.075f);
	sim.print_reynolds_number(Fluid::AIR);

	// Create LBM
	LBM lbm = sim.create_lbm(Fluid::AIR);

	// Configure tumbling using MovingPartsManager
	// Uses the same geometry as the main config, with arbitrary axis rotation
	MovingPartsManager parts(sim, lbm);
	parts.add(MovingPart(sim.model_file())
		.set_tumble(float3(0.2f, 1.0f, 0.1f), radians(0.4032f))
		.set_update_interval(update_interval));
	parts.initialize();

	// Configure boundaries - open with inlet velocity
	BoundaryBuilder(lbm)
		.set_open_boundaries()
		.initialize_velocity_y(flow_velocity)
		.apply();

	// Configure graphics (includes lattice visualization for tumbling effect)
	GraphicsConfig(lbm)
		.show_lattice()
		.show_surface()
		.show_vortices()
		.apply();

	// Run simulation
	const uint64_t total_timesteps = sim.to_lbm_timesteps(simulation_time);
	print_info(to_string(simulation_time.si(), 1u) + " seconds = " + to_string(total_timesteps) + " time steps");

#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)
	const Length W = domain_width; // camera positions from the domain's origin corner
	VideoRecorder()
		.add("t", CameraView::at({ 1.5f * W, 0.2f * W, 1.13f * W }, -33_deg, 33_deg).field_of_view(80_deg))
		.add("b", CameraView::at({ 0.8f * W, -2.0f * W, 0.05f * W }, -83_deg, -10_deg).field_of_view(40_deg))
		.add("f", CameraView::at({ 0.5f * W, 2.14f * W, 1.2f * W }, 90_deg, 29.5_deg).field_of_view(80_deg))
		.add("s", CameraView::at({ 3.0f * W, 1.0f * W, 0.5f * W }).field_of_view(50_deg))
		.set_video_length(30.0_s)
		.record(lbm, simulation_time, parts);
#else
	parts.run(simulation_time);
#endif
}
