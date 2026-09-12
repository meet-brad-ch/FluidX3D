#include "defines.hpp"
#include "info.hpp"
#include "lbm.hpp"
#include "graphics.hpp"
#include "setup.hpp"
#include "shapes.hpp"

void main_setup() { // hydraulic jump; required extensions: FP16S, VOLUME_FORCE, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SURFACE, SUBGRID, INTERACTIVE_GRAPHICS
	// physical parameters (SI units)
	const float domain_x_m = 0.96f;
	const float domain_y_m = 3.52f;
	const float domain_z_m = 0.96f;

	const float socket_length_m = domain_y_m * 3.0f / 20.0f;
	const float socket_height_m = domain_z_m * 2.0f / 5.0f;
	const float water_height_m = domain_z_m * 3.0f / 5.0f;

	const float flow_rate_m3ps = 0.25f;
	const float inlet_area_m2 = domain_x_m * (water_height_m - socket_height_m);
	const float outlet_area_m2 = domain_x_m * socket_height_m;
	const float inlet_velocity_mps = flow_rate_m3ps / inlet_area_m2;
	const float outlet_velocity_mps = flow_rate_m3ps / outlet_area_m2;

	// simulation setup
	SimulationSetup sim(SimulationConfig()
		.set_domain_size_m(domain_x_m, domain_y_m, domain_z_m)
		.set_vram_mb(208u));

	sim.setup();
	sim.configure_units(inlet_velocity_mps, Fluid::WATER);

	// create LBM for free surface simulation
	LBM lbm = sim.create_lbm_surface(Fluid::WATER, 9.81f);

	// convert SI dimensions to LBM cells
	const uint Nx = lbm.get_Nx(), Ny = lbm.get_Ny();
	const uint socket_y = (uint)sim.to_lbm_length(socket_length_m);
	const uint socket_z = (uint)sim.to_lbm_length(socket_height_m);
	const uint water_z = (uint)sim.to_lbm_length(water_height_m);

	// convert SI velocities to LBM
	const float u_inlet_lbm = sim.to_lbm_velocity(inlet_velocity_mps);
	const float u_outlet_lbm = sim.to_lbm_velocity(outlet_velocity_mps);

	// configure free surface with all geometry
	SurfaceBuilder(lbm)
		// initial water level
		.set_water_level_cells(water_z)
		.set_gravity_lbm(sim.to_lbm_acceleration(9.81f))
		.initialize_hydrostatic()
		// solid walls (sides and bottom, not top)
		.set_solid_faces({Face::X_MIN, Face::X_MAX, Face::Y_MIN, Face::Z_MIN})
		// socket obstruction
		.add_solid_block_cells(0, Nx, 0, socket_y, 0, socket_z)
		// inlet (velocity boundary above socket at y=1)
		.add_velocity_inlet(
			[=](uint x, uint y, uint z) {
				return y == 1u && x > 0u && x < Nx-1u && z >= socket_z && z < water_z;
			}, u_inlet_lbm, Axis::Y)
		// outlet (equilibrium boundary at y=Ny-1)
		.add_equilibrium_outlet(
			[=](uint x, uint y, uint z) {
				return y == Ny-1u && x > 0u && x < Nx-1u && z > 0u;
			}, u_outlet_lbm, Axis::Y)
		.apply();

	SurfaceBuilder(lbm).configure_visualization();

	lbm.run();
} /**/
