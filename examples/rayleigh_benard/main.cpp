#include "defines.hpp"
#include "info.hpp"
#include "lbm.hpp"
#include "graphics.hpp"
#include "setup.hpp"
#include "shapes.hpp"

void main_setup() { // Rayleigh-Benard convection; required extensions: FP16S, VOLUME_FORCE, TEMPERATURE, INTERACTIVE_GRAPHICS
	// physical parameters (SI units)
	const Length domain_size = 0.2_m;  // side of the square domain
	const Length height = 0.05_m;      // height (also for the Rayleigh number)

	const float T_hot_K = 330.0f;      // hot bottom: 330K (57°C)
	const float T_cold_K = 300.0f;     // cold top: 300K (27°C)
	const float delta_T = T_hot_K - T_cold_K;

	// buoyancy velocity scale for natural convection: u ~ sqrt(g * beta * dT * L)
	const float u_buoyancy = sqrtf(9.81f * Fluid::AIR.thermal_expansion * delta_T * height.si());

	// simulation setup (domain-only, no geometry)
	SimulationSetup sim(Domain::box(domain_size, domain_size, height).vram(2000_mb));

	sim.setup();
	sim.configure_units(u_buoyancy, Fluid::AIR);

	// create LBM with thermal parameters (air properties, gravity in -Z)
	LBM lbm = sim.create_lbm_thermal(Fluid::AIR, 9.81f, Axis::Z);

	// thermal boundary conditions with random perturbation to trigger instability
	ThermalBuilder(lbm)
		.set_hot_wall(Face::Z_MIN, T_hot_K)
		.set_cold_wall(Face::Z_MAX, T_cold_K)
		.set_gravity_axis(Axis::Z)
		.initialize_hydrostatic_pressure()
		.initialize_random_perturbation(0.015f)
		.apply();

	// solid floor and ceiling only (lateral walls periodic)
	BoundaryBuilder(lbm)
		.set_solid_floor()
		.set_solid_ceiling()
		.apply();

	// graphics
	GraphicsConfig(lbm)
		.show_surface()
		.show_streamlines()
		.apply();

	lbm.run();
} /**/
