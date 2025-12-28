#pragma once

#include "core/types.hpp"
#include "config/simulation_config.hpp"

/**
 * @file mesh_loader.hpp
 * @brief Mesh loading and transformation utilities for FluidX3D simulations
 *
 * Provides standalone functions for loading, scaling, and transforming meshes
 * that can be used by SimulationSetup and MovingPartsManager.
 */

namespace MeshLoader {

/**
 * @struct MeshParams
 * @brief Parameters for mesh loading and transformation
 */
struct MeshParams {
    string stl_path;                ///< Path to STL file
    float3 center_lbm{};            ///< Center position in LBM coordinates
    float3x3 rotation_matrix{};     ///< Rotation matrix to apply
    float32_t lbm_reference_size{}; ///< Reference size in LBM units (for scaling)
};

/**
 * @brief Load and transform a mesh with standard scaling and positioning
 *
 * Loads an STL file, applies rotation, scales to match the reference size,
 * and positions at the specified center.
 *
 * @param params Mesh loading parameters
 * @return Pointer to loaded and transformed Mesh (caller owns memory)
 *
 * @par Example:
 * @code
 * MeshLoader::MeshParams params;
 * params.stl_path = "path/to/mesh.stl";
 * params.center_lbm = float3(100.0f, 150.0f, 50.0f);
 * params.rotation_matrix = float3x3(float3(1,0,0), radians(90.0f));
 * params.lbm_reference_size = 80.0f;
 *
 * Mesh* mesh = MeshLoader::load(params);
 * // Use mesh...
 * delete mesh;
 * @endcode
 */
inline Mesh* load(const MeshParams& params) {
    Mesh* mesh = read_stl(params.stl_path, 1.0f, params.rotation_matrix);
    mesh->scale(params.lbm_reference_size / mesh->get_max_size());
    mesh->translate(params.center_lbm - mesh->get_bounding_box_center());
    mesh->set_center(mesh->get_center_of_mass());
    return mesh;
}

/**
 * @brief Load and mirror a mesh across a symmetry plane
 *
 * Creates a full mesh from a half-model STL by mirroring across the
 * specified plane. Used for symmetric aircraft and other geometry.
 *
 * @param params Mesh loading parameters
 * @param mirror_plane Mirror plane (X, Y, or Z)
 * @return Pointer to mirrored Mesh (caller owns memory)
 */
inline Mesh* load_mirrored(const MeshParams& params,
                           SimulationConfig::MirrorPlane mirror_plane) {
    if(mirror_plane == SimulationConfig::MirrorPlane::NONE) {
        return load(params);
    }

    // Load half mesh with rotation applied
    Mesh* half = read_stl(params.stl_path, 1.0f, params.rotation_matrix);
    half->scale(params.lbm_reference_size / half->get_max_size());

    // Translate so symmetry plane aligns with mesh edge
    switch(mirror_plane) {
        case SimulationConfig::MirrorPlane::X:
            half->translate(float3(-0.5f * (half->pmax.x - half->pmin.x), 0.0f, 0.0f));
            break;
        case SimulationConfig::MirrorPlane::Y:
            half->translate(float3(0.0f, -0.5f * (half->pmax.y - half->pmin.y), 0.0f));
            break;
        case SimulationConfig::MirrorPlane::Z:
            half->translate(float3(0.0f, 0.0f, -0.5f * (half->pmax.z - half->pmin.z)));
            break;
        default: break;
    }

    // Create full mesh with double the triangles
    Mesh* mesh = new Mesh(2u * half->triangle_number, float3(0.0f));

    // Copy original triangles
    for(uint32_t i = 0u; i < half->triangle_number; i++) {
        mesh->p0[i] = half->p0[i];
        mesh->p1[i] = half->p1[i];
        mesh->p2[i] = half->p2[i];
    }

    // Determine mirror axis and rotate half 180 degrees around it
    float3 mirror_axis;
    switch(mirror_plane) {
        case SimulationConfig::MirrorPlane::X: mirror_axis = float3(1, 0, 0); break;
        case SimulationConfig::MirrorPlane::Y: mirror_axis = float3(0, 1, 0); break;
        case SimulationConfig::MirrorPlane::Z: mirror_axis = float3(0, 0, 1); break;
        default: mirror_axis = float3(1, 0, 0); break;
    }
    half->rotate(float3x3(mirror_axis, radians(180.0f)));

    // Negate and append mirrored triangles
    for(uint32_t i = 0u; i < half->triangle_number; i++) {
        mesh->p0[half->triangle_number + i] = -half->p0[i];
        mesh->p1[half->triangle_number + i] = -half->p1[i];
        mesh->p2[half->triangle_number + i] = -half->p2[i];
    }

    delete half;
    mesh->find_bounds();

    // Center mesh vertically (Z)
    mesh->translate(float3(0.0f, 0.0f, -0.5f * (mesh->pmin.z + mesh->pmax.z)));

    // Position in domain
    mesh->translate(params.center_lbm);
    mesh->set_center(mesh->get_center_of_mass());

    return mesh;
}

/**
 * @brief Calculate mesh scale factor from SI to LBM units
 *
 * @param lbm_reference_size Reference dimension in LBM units (cells)
 * @param si_reference_size Reference dimension in SI units (meters)
 * @return Scale factor in cells per meter
 */
inline float32_t calculate_scale_factor(float32_t lbm_reference_size,
                                         float32_t si_reference_size) {
    return lbm_reference_size / si_reference_size;
}

} // namespace MeshLoader
