#pragma once

// Setup API: one include for all components. Typical flow (see porting_guide.md and the examples):
//   SimulationSetup sim(SimulationConfig("mesh.stl").set_vram_mb(2000u));
//   sim.setup();
//   sim.configure_units(1.0f, Fluid::AIR); // reference velocity in m/s
//   LBM lbm = sim.create_lbm(Fluid::AIR);
//   sim.voxelize(lbm);
//   BoundaryBuilder(lbm).set_solid_floor().set_open_boundaries().initialize_velocity_y(1.0f).apply();
//   GraphicsConfig(lbm).show_surface().show_vortices().apply();
//   lbm.run();

#include "core/types.hpp"
#include "core/fluids.hpp"
#include "core/boundary_utils.hpp"

#include "config/camera_config.hpp"
#include "config/simulation_config.hpp"

#include "simulation/geometry_scaler.hpp"
#include "simulation/mesh_loader.hpp"
#include "simulation/simulation_setup.hpp"

#include "boundaries/boundary_flags.hpp"
#include "boundaries/boundary_builder.hpp"

#ifdef TEMPERATURE
#include "boundaries/thermal_utils.hpp"
#include "boundaries/thermal_builder.hpp"
#endif // TEMPERATURE

#ifdef GRAPHICS
#include "graphics/graphics_config.hpp"
#include "graphics/video_recorder.hpp"
#endif // GRAPHICS

#include "moving/moving_part.hpp"
#include "moving/moving_parts_manager.hpp"

#include "sdf/sdf_generator.hpp"

#ifdef FORCE_FIELD
#include "analysis/force_analyzer.hpp"
#endif // FORCE_FIELD

#ifdef SURFACE
#include "surface/surface_builder.hpp"
#include "surface/wave_boundary.hpp"
#endif // SURFACE

#ifdef PARTICLES
#include "particles/particle_manager.hpp"
#endif // PARTICLES
