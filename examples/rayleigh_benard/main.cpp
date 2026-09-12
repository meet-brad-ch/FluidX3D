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

	const Temperature T_hot = 330.0_K;  // hot bottom (57°C)
	const Temperature T_cold = 300.0_K; // cold top (27°C)
	const Temperature delta_T = T_hot - T_cold;

	// buoyancy velocity scale for natural convection: u ~ sqrt(g * beta * dT * L)
	const Speed u_buoyancy = sqrt(9.81_mps2 * Fluid::AIR.thermal_expansion * delta_T * height);

	// simulation setup (domain-only, no geometry)
	SimulationSetup sim(Domain::box(domain_size, domain_size, height).vram(2000_mb));

	sim.setup();
	sim.configure_units(u_buoyancy, Fluid::AIR);
	sim.configure_temperatures(T_cold, T_hot); // 0.5 and 1.5 in lattice units

	// create LBM with thermal parameters (air properties, gravity in -Z)
	LBM lbm = sim.create_lbm_thermal(Fluid::AIR, 9.81_mps2, Axis::Z);

	// thermal boundary conditions with random perturbation to trigger instability
	ThermalBuilder(lbm, sim.temperature_scale())
		.set_hot_wall(Face::Z_MIN, T_hot)
		.set_cold_wall(Face::Z_MAX, T_cold)
		.set_gravity_axis(Axis::Z)
		.initialize_hydrostatic_pressure()
		.initialize_random_perturbation(0.15f*u_buoyancy) // 0.015 in lattice units, as the original
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
