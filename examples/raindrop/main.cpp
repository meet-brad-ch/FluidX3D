/// @file main.cpp
/// @brief Raindrop impact on sea water: a 7 mm drop hits a pool at its terminal speed and 20 degrees from the
/// vertical (free surface, interactive or video graphics).

#include "setup/setup.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <string>

namespace {

/// @brief One row of the original's drop table: a drop diameter and its impact speed, the terminal speed of a drop of
/// that diameter in air.
struct DropSize {
	si::Length diameter{ au::ZERO };  ///< the drop's diameter
	si::Speed impact_speed{ au::ZERO }; ///< the drop's impact (terminal) speed
};

/// @brief The drop's dimensionless numbers, printed with print_info() before the run.
class DropNumbers {
public:
	/// @brief Prints the drop's diameter in m and its Weber, Froude, capillary and Bond numbers, each with 6 decimals.
	/// @param D the drop's diameter
	/// @param impact_speed the drop's speed at the impact
	/// @param fluid the pool's and the drop's fluid (its density and kinematic viscosity)
	/// @param surface_tension the fluid's surface tension, more than 0
	/// @param gravity the gravitational acceleration, more than 0
	static void print(si::Length D, si::Speed impact_speed, const FluidProperties& fluid, si::SurfaceTension surface_tension, si::Acceleration gravity) {
		using namespace si::symbols;
		const si::Density rho = fluid.density;
		const si::KinematicViscosity nu = fluid.kinematic_viscosity;
		const double weber = au::as_raw_number(D * impact_speed * impact_speed * rho / surface_tension); // We = d u² rho / sigma
		const double froude = au::as_raw_number(impact_speed / au::sqrt(D * gravity));                   // Fr = u / sqrt(d g)
		const double capillary = au::as_raw_number(impact_speed * rho * nu / surface_tension);           // Ca = u rho nu / sigma
		const double bond = au::as_raw_number(D * D * rho * gravity / surface_tension);                  // Bo = d² rho g / sigma
		const double D_m = D.in(m);
		print_number("D = ", D_m);
		print_number("We = ", weber);
		print_number("Fr = ", froude);
		print_number("Ca = ", capillary);
		print_number("Bo = ", bond);
	}

private:
	static constexpr uint printed_decimals = 6u; ///< the decimals of each printed number

	/// @brief Prints one number with its label, as a float with printed_decimals decimals.
	/// @param label the text before the number, for example "We = "
	/// @param value the number
	static void print_number(const std::string& label, double value) {
		const float value_f = static_cast<float>(value);
		const std::string text = label + to_string(value_f, printed_decimals);
		print_info(text);
	}
};

/// @brief The video's four cameras: two orbits from above the side (n, p), one at the level of the pool (o) and one
/// from straight above (t).
class RaindropCameras {
public:
	/// @brief Adds the four cameras to the video and sets the video's length to 20 s.
	/// @param video the simulation's video (Simulation::video())
	static void add(VideoRecorder& video) {
		using namespace si::symbols;
		const si::Angle n_azimuth = -30.0 * deg;
		const si::Angle n_elevation = 20.0 * deg;
		const si::Angle p_azimuth = 10.0 * deg;
		const si::Angle p_elevation = 40.0 * deg;
		const si::Angle o_azimuth = 0.0 * deg;
		const si::Angle o_elevation = 0.0 * deg; // at the level of the pool
		const si::Angle t_azimuth = 0.0 * deg;
		const si::Angle t_elevation = 90.0 * deg; // straight down
		const si::Angle narrow_field = 45.0 * deg;
		const si::Duration video_length = 20.0 * s;
		video.add("n", CameraView::orbit(n_azimuth, n_elevation))
			.add("p", CameraView::orbit(p_azimuth, p_elevation))
			.add("o", CameraView::orbit(o_azimuth, o_elevation).field_of_view(narrow_field))
			.add("t", CameraView::orbit(t_azimuth, t_elevation).field_of_view(narrow_field))
			.set_length(video_length);
	}
};

} // namespace

/// @brief Sets up a pool 2 drop diameters deep in a 4000 MiB grid (419 x 419 x 356 cells, as the original), prints
/// the drop's Weber, Froude, capillary and Bond numbers, and records 3 ms of simulated time.
///
/// The lattice Mach number gives the original's lattice speed 0.05. The sides and the top above the pool drain the
/// splashes that reach them.
/// @throws std::out_of_range when select_drop_size is not a row of the drop table
void main_setup() { // extensions: FP16C, VOLUME_FORCE, EQUILIBRIUM_BOUNDARIES, SURFACE, INTERACTIVE_GRAPHICS or GRAPHICS
	using namespace si::symbols;
	// drop diameters and their impact (terminal) speeds; row 13 is for validation
	const std::size_t select_drop_size = 12u;
	const si::Speed mps = 1.0 * m / s;
	const std::array<DropSize, 14u> drop_sizes{ {
		{ 1.0 * mm, 4.50 * mps }, { 1.5 * mm, 5.80 * mps }, { 2.0 * mm, 6.80 * mps }, { 2.5 * mm, 7.55 * mps },
		{ 3.0 * mm, 8.10 * mps }, { 3.5 * mm, 8.45 * mps }, { 4.0 * mm, 8.80 * mps }, { 4.5 * mm, 9.05 * mps },
		{ 5.0 * mm, 9.20 * mps }, { 5.5 * mm, 9.30 * mps }, { 6.0 * mm, 9.40 * mps }, { 6.5 * mm, 9.45 * mps },
		{ 7.0 * mm, 9.55 * mps }, { 4.1 * mm, 7.21 * mps },
	} };
	const DropSize drop_size = drop_sizes.at(select_drop_size);
	const si::Length D = drop_size.diameter;
	const si::Speed impact_speed = drop_size.impact_speed;
	const si::Angle inclination = 20.0 * deg; // of the impact, 0 = vertical
	const si::Duration simulation_time = 0.003 * s;
	const si::Acceleration gravity = 9.81 * m / (s * s);
	const FluidProperties sea_water{ .density = 1024.8103 * kg / (m * m * m), .kinematic_viscosity = 1.0508E-6 * m * m / s }; // at 20 °C and 35 g/l salinity
	const si::SurfaceTension surface_tension = 73.81E-3 * N / m;

	const si::Length width = 5.0 * D;
	const si::Length height = 4.25 * D;
	const si::MemoryAmount vram = 4000.0 * MiB;
	Domain domain = Domain::box(width, width, height); // 419 x 419 x 356 cells, as the original
	domain.vram(vram);
	const si::LatticeMach mach(0.0866); // the original's lattice speed 0.05
	Simulation sim(domain, sea_water, impact_speed, mach);
	sim.set_gravity(gravity).set_surface_tension(surface_tension);
	DropNumbers::print(D, impact_speed, sea_water, surface_tension, gravity);

	// a pool 2 diameters deep; the drop 3 cells above it, falling at the inclination toward the domain's center
	const si::Length depth = 2.0 * D;
	const si::Length cell = sim.converter().cell();
	const double sine = std::sin(inclination.in(rad));
	const double cosine = std::cos(inclination.in(rad));
	const si::Position drop{ 0.5 * width, 0.5 * width - D * (sine / cosine), depth + 0.5 * D + 3.0 * cell };
	const si::Velocity drop_velocity{ si::Speed{}, sine * impact_speed, -cosine * impact_speed };
	// the sides and the top, above the pool: splashes reaching them leave the domain
	const Shape inner_box = Shape::box({ cell, cell, 0.0 * m }, { width - cell, width - cell, height - cell });
	const Shape above_pool = Shape::box({ 0.0 * m, 0.0 * m, depth + 0.25 * D }, { width, width, height });
	const Shape drain = (!inner_box) & above_pool;
	sim.surface()
		.set_water_level(depth)
		.add_water(Shape::sphere(drop, 0.5 * D), drop_velocity)
		.set_solid_faces({ Face::Z_MIN })
		.add_drain(drain)
		.apply();

	sim.graphics()
		.show_free_surface()
		.apply();

	RaindropCameras::add(sim.video());
	sim.run_for(simulation_time);
}
