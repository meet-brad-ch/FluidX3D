// A bubble bursting at a water surface, using Setup API

#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() { // bursting bubble; required extensions: FP16C, VOLUME_FORCE, SURFACE, INTERACTIVE_GRAPHICS
	const Length d = 4.0_mm; // the bubble's diameter
	const FluidProperties& water = Fluid::WATER;
	const SurfaceTension surface_tension = 0.072_Npm;

	SimulationSetup sim(Domain::box(4.0f * d, 4.0f * d, 3.0f * d).vram(1000_mb)); // 275 x 275 x 206 cells, as the original
	sim.setup();
	// no flow sets the units, but the bubble's capillary speed sqrt(sigma/(rho*d)), in lattice units the original's with
	// the lattice surface tension 0.0003
	const float cells_per_diameter = 0.25f * (float)sim.get_results().Nx;
	sim.configure_units(sqrt(surface_tension / (water.density * d)), water, sqrtf(0.0003f / cells_per_diameter));

	LBM lbm = sim.create_lbm_surface(water, 9.81_mps2, surface_tension);

	// water 2 diameters deep, the bubble just below its surface
	const Length depth = 2.0f * d;
	SurfaceBuilder(lbm)
		.set_water_level(depth)
		.add_gas(Shape::sphere({ 2.0f * d, 2.0f * d, depth - 0.5f * d }, 0.5f * d))
		.set_solid_faces({ Face::Z_MIN })
		.apply();

	GraphicsConfig(lbm)
		.show_free_surface()
		.apply();

	lbm.run();
} /**/
