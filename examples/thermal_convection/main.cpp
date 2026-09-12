#include "defines.hpp"
#include "info.hpp"
#include "lbm.hpp"
#include "graphics.hpp"
#include "setup.hpp"
#include "shapes.hpp"

void main_setup() { // thermal convection; required extensions: FP16S, VOLUME_FORCE, TEMPERATURE, INTERACTIVE_GRAPHICS
	// physical parameters (SI units)
	const Length domain_x = 0.1_m;     // wide
	const Length domain_y = 0.6_m;     // long (flow direction)
	const Length domain_z = 0.2_m;     // tall

	const Temperature T_hot = 350.0_K;  // hot wall (77°C)
	const Temperature T_cold = 300.0_K; // cold wall (27°C)
	const Temperature delta_T = T_hot - T_cold;

	// buoyancy velocity scale for natural convection: u ~ sqrt(g * beta * dT * L)
	const Speed u_buoyancy = sqrt(9.81_mps2 * Fluid::AIR.thermal_expansion * delta_T * domain_z);

	// simulation setup (domain-only, no geometry)
	SimulationSetup sim(Domain::box(domain_x, domain_y, domain_z).vram(2000_mb));

	sim.setup();
	sim.configure_units(u_buoyancy, Fluid::AIR);
	sim.configure_temperatures(T_cold, T_hot); // 0.5 and 1.5 in lattice units

	// create LBM with thermal parameters (air properties, gravity in -Z)
	LBM lbm = sim.create_lbm_thermal(Fluid::AIR, 9.81_mps2, Axis::Z);

	// thermal boundary conditions (SI temperatures in Kelvin)
	ThermalBuilder(lbm, sim.temperature_scale())
		.set_hot_wall(Face::Y_MIN, T_hot)
		.set_cold_wall(Face::Y_MAX, T_cold)
		.set_gravity_axis(Axis::Z)
		.initialize_hydrostatic_pressure()
		.apply();

	// all six faces solid, as the original
	BoundaryBuilder(lbm)
		.set_solid_floor()
		.set_solid_ceiling()
		.set_solid_walls()
		.apply();

	// graphics
	GraphicsConfig(lbm)
		.show_surface()
		.show_streamlines()
		.apply();

	lbm.run();
} /**/
