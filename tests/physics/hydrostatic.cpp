// Hydrostatic pressure: a closed column of glycerol at rest under gravity, against dp/dz = -rho*g.
// Passes when the pressure difference between two heights, in pascals, is within 1 % of rho*g*dz.

#include "setup/setup.hpp"
#include "physics_check.hpp"

void main_setup() { // extensions: VOLUME_FORCE
	const Length height = 10.0_cm, width = 2.5_cm;
	const Length cell = height / 64.0f;                // 16 x 16 x 64 cells
	const Acceleration g = 9.81_mps2;
	// glycerol at 20 °C: its viscosity damps the sound waves of the start
	const FluidProperties glycerol { .density = 1260.0_kgpm3, .kinematic_viscosity = 1.12E-3_m2ps };

	Simulation sim(Domain::box(width, width, height).cell_size(cell), glycerol, sqrt(g * height)); // the speed of a fall through the column
	sim.set_gravity(g);

	sim.boundaries()
		.set_solid_box()
		.apply();

	// the mean pressure of the liquid in the layer of cells at a height, in Pa
	const auto layer_pressure = [&](const FieldReader& fields, Length z) {
		const Length layer = fields.cell_at({ 0.5f * width, 0.5f * width, z }).center.z; // the center of the layer containing z
		double pressure_sum = 0.0;
		uint32_t cells = 0u;
		fields.for_each_cell([&](const FieldReader::Cell& c) {
			if(c.solid || fabs((c.center.z - layer).si()) > 0.1f * cell.si()) return;
			pressure_sum += (double)c.pressure.si();
			cells++;
		});
		return pressure_sum / (double)cells;
	};
	const Length z_low = 0.25f * height, z_high = 0.75f * height;
	const double expected = glycerol.density.si() * g.si() * (sim.fields().cell_at({ 0.5f * width, 0.5f * width, z_high }).center.z -
	                                                          sim.fields().cell_at({ 0.5f * width, 0.5f * width, z_low }).center.z).si(); // Pa

	double P1 = 0.0, P2 = 0.0;
	sim.every(0.1_s, [&](Duration t) {
		const FieldReader fields = sim.fields();
		const double P0 = layer_pressure(fields, z_low) - layer_pressure(fields, z_high);
		if((t > 1.0_s && converged(P2, P1, P0, 1E-5)) || t > 60.0_s) { // settled (or not settling)
			PhysicsCheck::report("hydrostatic pressure difference", fabs(P0 / expected - 1.0), 0.01);
			sim.stop();
		}
		P2 = P1;
		P1 = P0;
	});
	sim.run();
}
