#pragma once

/// @file setup.hpp
/// @brief The Setup API: one include for all components. A simulation in physical units (see SETUP_API.md and the
/// examples):
/// @code
/// using namespace si::symbols;
/// Simulation sim(Domain::around(Model("mesh.stl")).clearances(1.0 * m, 2.0 * m, 1.0 * m).vram(2000.0 * MiB), Fluid::AIR, 1.0 * m / s);
/// sim.boundaries().set_solid_floor().set_open_boundaries().initialize_velocity_y(1.0 * m / s).apply();
/// sim.graphics().show_surface().show_vortices().apply();
/// sim.run_for(10.0 * s);
/// @endcode
/// simulation.hpp includes the builders and components of the extensions the build has (SURFACE, TEMPERATURE,
/// FORCE_FIELD, PARTICLES, GRAPHICS).

#include "setup/core/types.hpp"
#include "setup/core/fluids.hpp"
#include "setup/core/time_step_rule.hpp"
#include "setup/core/diffusivity.hpp"
#include "setup/core/turbulent_prandtl.hpp"
#include "setup/core/scalar_transport_rules.hpp"
#include "setup/core/boussinesq_buoyancy.hpp"
#include "setup/units/units.hpp"
#include "setup/units/vector3.hpp"
#include "setup/units/lattice_mach.hpp"
#include "setup/units/physical_constants.hpp"
#include "setup/units/lattice_converter.hpp"
#include "setup/units/moist_air_physics.hpp"
#include "setup/boundaries/boundary_flags.hpp"
#include "setup/domain/domain.hpp"
#include "setup/domain/run_estimate.hpp"
#include "setup/domain/level_grid.hpp"
#include "setup/atmosphere/vertical_profile.hpp"
#include "setup/atmosphere/sounding.hpp"
#include "setup/atmosphere/moist_column.hpp"
#include "setup/atmosphere/wind_target.hpp"
#include "setup/atmosphere/sponge_profile.hpp"
#include "setup/atmosphere/start_noise.hpp"
#include "setup/atmosphere/heated_patch.hpp"
#include "setup/atmosphere/boundary_layer_diagnostics.hpp"
#include "setup/atmosphere/run_checks.hpp"
#include "setup/atmosphere/wind_nudging.hpp"
#include "setup/atmosphere/atmosphere_grid.hpp"
#include "setup/atmosphere/cbl_description.hpp"
#include "setup/atmosphere/lattice_parameters.hpp"
#include "setup/domain/shape.hpp"
#include "setup/graphics/camera_view.hpp"
#include "setup/moving/moving_part.hpp"
#include "setup/simulation/simulation.hpp"
