#include "defines.hpp"
#include "info.hpp"
#include "lbm.hpp"
#include "graphics.hpp"
#include "setup.hpp"
#include "shapes.hpp"

void main_setup() { // thermal convection; required extensions: FP16S, VOLUME_FORCE, TEMPERATURE, INTERACTIVE_GRAPHICS
	// physical parameters (SI units)
	const float domain_x_m = 0.1f;     // 10cm wide
	const float domain_y_m = 0.6f;     // 60cm long (flow direction)
	const float domain_z_m = 0.2f;     // 20cm tall

	const float T_hot_K = 350.0f;      // hot wall: 350K (77°C)
	const float T_cold_K = 300.0f;     // cold wall: 300K (27°C)
	const float delta_T = T_hot_K - T_cold_K;

	// buoyancy velocity scale for natural convection: u ~ sqrt(g * beta * dT * L)
	const float u_buoyancy = sqrtf(9.81f * Fluid::AIR.thermal_expansion * delta_T * domain_z_m);

	// simulation setup (domain-only, no geometry)
	SimulationSetup sim(SimulationConfig()
		.set_domain_size_m(domain_x_m, domain_y_m, domain_z_m)
		.set_vram_mb(2000u));

	sim.setup();
	sim.configure_units(u_buoyancy, Fluid::AIR);

	// create LBM with thermal parameters (air properties, gravity in -Z)
	LBM lbm = sim.create_lbm_thermal(Fluid::AIR, 9.81f, Axis::Z);

	// thermal boundary conditions (SI temperatures in Kelvin)
	ThermalBuilder(lbm)
		.set_hot_wall(Face::Y_MIN, T_hot_K)
		.set_cold_wall(Face::Y_MAX, T_cold_K)
		.set_gravity_axis(Axis::Z)
		.initialize_hydrostatic_pressure()
		.apply();

	// solid walls
	BoundaryBuilder(lbm)
		.set_solid_walls()
		.apply();

	// graphics
	GraphicsConfig(lbm)
		.show_surface()
		.show_streamlines()
		.apply();

	lbm.run();
} /**/
