#pragma once

// Setup API: one include for all components. Typical flow (see porting_guide.md and the examples):
//   SimulationSetup sim(Domain::around(Model("mesh.stl")).clearances(1_m, 2_m, 1_m).vram(2000_mb));
//   sim.setup();
//   sim.configure_units(1.0_mps, Fluid::AIR); // reference velocity
//   LBM lbm = sim.create_lbm(Fluid::AIR);
//   sim.voxelize(lbm);
//   BoundaryBuilder(lbm).set_solid_floor().set_open_boundaries().initialize_velocity_y(1.0_mps).apply();
//   GraphicsConfig(lbm).show_surface().show_vortices().apply();
//   lbm.run();

#include "setup/core/types.hpp"
#include "setup/core/fluids.hpp"
#include "setup/core/boundary_utils.hpp"

#include "setup/config/camera_config.hpp"

#include "setup/domain/lattice.hpp"
#include "setup/domain/geometry_scaler.hpp"
#include "setup/domain/domain_plan.hpp"
#include "setup/domain/domain.hpp"
#include "setup/domain/model_placement.hpp"
#include "setup/simulation/mesh_loader.hpp"
#include "setup/simulation/simulation_setup.hpp"

#include "setup/boundaries/boundary_flags.hpp"
#include "setup/boundaries/boundary_builder.hpp"

#ifdef TEMPERATURE
#include "setup/boundaries/thermal_builder.hpp"
#endif // TEMPERATURE

#ifdef GRAPHICS
#include "setup/graphics/graphics_config.hpp"
#include "setup/graphics/video_recorder.hpp"
#endif // GRAPHICS

#include "setup/moving/moving_part.hpp"
#include "setup/moving/moving_parts_manager.hpp"

#include "setup/sdf/sdf_generator.hpp"

#ifdef FORCE_FIELD
#include "setup/analysis/force_analyzer.hpp"
#endif // FORCE_FIELD

#ifdef SURFACE
#include "setup/surface/surface_builder.hpp"
#include "setup/surface/wave_boundary.hpp"
#endif // SURFACE

#ifdef PARTICLES
#include "setup/particles/particle_manager.hpp"
#endif // PARTICLES
