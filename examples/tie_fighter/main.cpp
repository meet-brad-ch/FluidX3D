// Star Wars TIE Fighter (tumbling)
//
// Extensions: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID
// STL from: https://www.thingiverse.com/thing:2919109/files
//
// The domain's model tumbles as a moving part (MovingPart::set_tumble()) while the air flows past it.

#include "setup/setup.hpp"

void main_setup() {
	const Length height = 8.8_m;              // TIE/LN (Wookieepedia): the wing panels' height (Z), the longest side
	const Speed flow_velocity = 50.0_mps;
	const Duration simulation_time = 5.0_s;   // 5 seconds of tumbling
	const Duration update_interval = 2.2_ms;  // 28 time steps, as the original

	// the height (Z) is 65 % of the domain width; the fighter's center is about 0.6 heights from the inlet
	const Length domain_width = height / 0.65f;
	Simulation sim(Domain::around(Model("DWG_Tie_Fighter_Assembled_02.stl").rotation(90_deg, 0_deg, 0_deg).length(height, Axis::Z))
		.size(domain_width, 2.0f * domain_width, domain_width)
		.model_offset(0_m, -0.94f * height, 0_m)
		.vram(1760_mb),
		Fluid::AIR, flow_velocity, LatticeMach(0.13f)); // the original's lattice speed 0.075
	sim.skip_model_voxelization(); // the fighter tumbles: a moving part, not a static solid

	sim.parts()
		.add(MovingPart(sim.model_file())
			.set_tumble(float3(0.2f, 1.0f, 0.1f), 180_deg / 1.0_s) // half a turn per second (the original: 0.4° per 28 time steps)
			.set_update_interval(update_interval))
		.initialize();

	sim.boundaries()
		.set_open_boundaries()
		.initialize_velocity_y(flow_velocity)
		.apply();

	sim.graphics()
		.show_flags()
		.show_surface()
		.show_vortices()
		.apply();

	const Length W = domain_width; // camera positions from the domain's origin corner
	sim.video()
		.add("t", CameraView::at({ 1.5f * W, 0.2f * W, 1.13f * W }, -33_deg, 33_deg).field_of_view(80_deg))
		.add("b", CameraView::at({ 0.8f * W, -2.0f * W, 0.05f * W }, -83_deg, -10_deg).field_of_view(40_deg))
		.add("f", CameraView::at({ 0.5f * W, 2.14f * W, 1.2f * W }, 90_deg, 29.5_deg).field_of_view(80_deg))
		.add("s", CameraView::at({ 3.0f * W, 1.0f * W, 0.5f * W }).field_of_view(50_deg))
		.set_length(30.0_s);
	sim.run_for(simulation_time);
}
