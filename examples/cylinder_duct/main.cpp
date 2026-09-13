// Cylinder in a rectangular duct, driven by a pressure gradient

#include "setup/setup.hpp"

void main_setup() { // extensions: VOLUME_FORCE, INTERACTIVE_GRAPHICS
	const Length diameter = 1.0_cm; // the cylinder's, also the duct's width
	const Length width = diameter, length = 12.0f * diameter, height = 3.0f * diameter;
	const float reynolds = 25000.0f;
	const KinematicViscosity viscosity = Fluid::WATER.kinematic_viscosity;
	const Speed center_speed = reynolds * viscosity / diameter; // 2.5 m/s in water

	Simulation sim(Domain::box(width, length, height).cell_size(diameter / 64.0f), // 64 x 768 x 192 cells
	               Fluid::WATER, center_speed, LatticeMach(1.0f)); // the lattice speed of sound, as the original

	// the pressure gradient (per density) of laminar flow at this center speed through a square duct as wide as the
	// cylinder, as the original
	const Acceleration drive = Acceleration::from_si(units.f_from_u_rectangular_duct(width.si(), diameter.si(), 1.0f, viscosity.si(), center_speed.si()));
	sim.set_body_force({ Acceleration{}, drive, Acceleration{} });

	sim.boundaries()
		.set_solid_faces({ Face::X_MIN, Face::X_MAX, Face::Z_MIN, Face::Z_MAX }) // periodic along Y
		.add_solid(Shape::cylinder({ 0.5f * width, 2.0f * diameter, 0.5f * height }, Axis::X, 0.5f * diameter, width))
		.initialize_velocity_y(0.1f * center_speed)
		.apply();

	sim.graphics()
		.show_flags()
		.show_vortices()
		.apply();

	sim.run();
}
