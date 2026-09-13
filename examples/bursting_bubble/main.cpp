// A bubble bursting at a water surface

#include "setup/setup.hpp"

void main_setup() { // extensions: FP16C, VOLUME_FORCE, SURFACE, INTERACTIVE_GRAPHICS
	const Length d = 4.0_mm; // the bubble's diameter
	const float cells_per_diameter = 68.75f; // 275 x 275 x 206 cells, as the original
	const FluidProperties& water = Fluid::WATER;
	const SurfaceTension surface_tension = 0.072_Npm;

	// no flow sets the units, but the bubble's capillary speed sqrt(sigma/(rho*d)); on the lattice it is the original's,
	// sqrt(sigma/d) with the lattice surface tension 0.0003 and d in cells
	Simulation sim(Domain::box(4.0f * d, 4.0f * d, 3.0f * d).cell_size(d / cells_per_diameter),
	               water, sqrt(surface_tension / (water.density * d)), LatticeMach(sqrtf(3.0f * 0.0003f / cells_per_diameter)));
	sim.set_gravity(9.81_mps2).set_surface_tension(surface_tension);

	// water 2 diameters deep, the bubble just below its surface
	const Length depth = 2.0f * d;
	sim.surface()
		.set_water_level(depth)
		.add_gas(Shape::sphere({ 2.0f * d, 2.0f * d, depth - 0.5f * d }, 0.5f * d))
		.set_solid_faces({ Face::Z_MIN })
		.apply();

	sim.graphics()
		.show_free_surface()
		.apply();

	sim.run();
}
