#pragma once

/// @file setup.hpp
/// @brief The Setup API: one include for all components. A simulation in physical units (see SETUP_API.md and the
/// examples):
/// @code
/// Simulation sim(Domain::around(Model("mesh.stl")).clearances(1_m, 2_m, 1_m).vram(2000_mb), Fluid::AIR, 1.0_mps);
/// sim.boundaries().set_solid_floor().set_open_boundaries().initialize_velocity_y(1.0_mps).apply();
/// sim.graphics().show_surface().show_vortices().apply();
/// sim.run_for(10.0_s);
/// @endcode
/// simulation.hpp includes the builders and components of the extensions the build has (SURFACE, TEMPERATURE,
/// FORCE_FIELD, PARTICLES, GRAPHICS).

#include "setup/core/types.hpp"
#include "setup/core/quantity.hpp"
#include "setup/core/fluids.hpp"
#include "setup/boundaries/boundary_flags.hpp"
#include "setup/domain/domain.hpp"
#include "setup/domain/shape.hpp"
#include "setup/graphics/camera_view.hpp"
#include "setup/moving/moving_part.hpp"
#include "setup/simulation/simulation.hpp"
