#pragma once

/**
 * @file setup.hpp
 * @brief Single include header for the FluidX3D Setup API
 *
 * Include this header to get access to all Setup API components:
 * - SimulationConfig: Geometry and VRAM configuration
 * - SimulationSetup: Main setup class with units integration
 * - MovingPart: Configuration for moving parts (propellers, wheels)
 * - MovingPartsManager: Runtime management of moving parts
 * - BoundaryBuilder: Fluent boundary condition API
 * - GraphicsConfig: Visualization configuration
 * - VideoRecorder: Multi-camera video recording
 * - ForceAnalyzer: Cd/Cl coefficient calculations
 * - SDFGenerator: Standalone SDF generation
 * - Fluid: Common fluid property presets
 *
 * @par Example:
 * @code
 * #include "setup/setup.hpp"
 *
 * void main_setup() {
 *     SimulationSetup sim(SimulationConfig("mesh.stl").set_vram_mb(2000u));
 *     auto results = sim.setup();
 *     sim.configure_units(1.0f, Fluid::AIR);
 *
 *     LBM lbm = sim.create_lbm(Fluid::AIR.kinematic_viscosity);
 *     sim.voxelize(lbm);
 *
 *     BoundaryBuilder(lbm)
 *         .set_solid_floor()
 *         .set_open_boundaries()
 *         .apply();
 *
 *     GraphicsConfig(lbm)
 *         .show_surface()
 *         .show_vortices()
 *         .apply();
 *
 *     lbm.run();
 * }
 * @endcode
 */

// Core types and utilities
#include "core/types.hpp"
#include "core/fluent_builder.hpp"
#include "core/fluids.hpp"
#include "core/unit_helpers.hpp"
#include "core/boundary_utils.hpp"

// Configuration constants and classes
#include "config/camera_presets.hpp"
#include "config/camera_config.hpp"
#include "config/simulation_config.hpp"

// Simulation components
#include "simulation/geometry_scaler.hpp"
#include "simulation/mesh_loader.hpp"
#include "simulation/simulation_setup.hpp"

// Boundary configuration
#include "boundaries/boundary_flags.hpp"
#include "boundaries/boundary_builder.hpp"

// ThermalBuilder requires TEMPERATURE extension
#ifdef TEMPERATURE
#include "boundaries/thermal_utils.hpp"
#include "boundaries/thermal_builder.hpp"
#endif

// Graphics and visualization (requires GRAPHICS extension)
#ifdef GRAPHICS
#include "graphics/graphics_config.hpp"
#include "graphics/video_recorder.hpp"
#endif

// Moving parts
#include "moving/moving_part.hpp"
#include "moving/moving_parts_manager.hpp"

// SDF generation
#include "sdf/sdf_generator.hpp"

// ForceAnalyzer requires FORCE_FIELD extension
#ifdef FORCE_FIELD
#include "analysis/force_analyzer.hpp"
#endif

// SurfaceBuilder and WaveBoundary require SURFACE extension
#ifdef SURFACE
#include "surface/surface_builder.hpp"
#include "surface/wave_boundary.hpp"
#endif

// ParticleManager requires PARTICLES extension
#ifdef PARTICLES
#include "particles/particle_manager.hpp"
#endif
