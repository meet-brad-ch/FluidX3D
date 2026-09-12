#pragma once

#include "core/types.hpp"
#include "config/simulation_config.hpp"

// Loads meshes scaled and placed like the SimulationSetup geometry.
namespace MeshLoader {

struct MeshParams {
    string stl_path;
    float3 center_lbm{};            // cells
    float3x3 rotation_matrix{};
    float32_t lbm_reference_size{}; // longest dimension in cells
};

// rotated, scaled to lbm_reference_size and centered at center_lbm (caller deletes)
inline Mesh* load(const MeshParams& params) {
    Mesh* mesh = read_stl(params.stl_path, 1.0f, params.rotation_matrix);
    mesh->scale(params.lbm_reference_size / mesh->get_max_size());
    mesh->translate(params.center_lbm - mesh->get_bounding_box_center());
    mesh->set_center(mesh->get_center_of_mass());
    return mesh;
}

// full mesh from a half model, mirrored across mirror_plane (caller deletes)
inline Mesh* load_mirrored(const MeshParams& params,
                           SimulationConfig::MirrorPlane mirror_plane) {
    if(mirror_plane == SimulationConfig::MirrorPlane::NONE) {
        return load(params);
    }

    Mesh* half = read_stl(params.stl_path, 1.0f, params.rotation_matrix);
    half->scale(params.lbm_reference_size / half->get_max_size());

    // align the symmetry plane with the mesh edge
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

    Mesh* mesh = new Mesh(2u * half->triangle_number, float3(0.0f));
    for(uint32_t i = 0u; i < half->triangle_number; i++) {
        mesh->p0[i] = half->p0[i];
        mesh->p1[i] = half->p1[i];
        mesh->p2[i] = half->p2[i];
    }

    // mirrored copy: rotate 180 degrees around the mirror axis, then negate
    float3 mirror_axis;
    switch(mirror_plane) {
        case SimulationConfig::MirrorPlane::X: mirror_axis = float3(1, 0, 0); break;
        case SimulationConfig::MirrorPlane::Y: mirror_axis = float3(0, 1, 0); break;
        case SimulationConfig::MirrorPlane::Z: mirror_axis = float3(0, 0, 1); break;
        default: mirror_axis = float3(1, 0, 0); break;
    }
    half->rotate(float3x3(mirror_axis, radians(180.0f)));
    for(uint32_t i = 0u; i < half->triangle_number; i++) {
        mesh->p0[half->triangle_number + i] = -half->p0[i];
        mesh->p1[half->triangle_number + i] = -half->p1[i];
        mesh->p2[half->triangle_number + i] = -half->p2[i];
    }

    delete half;
    mesh->find_bounds();
    mesh->translate(float3(0.0f, 0.0f, -0.5f * (mesh->pmin.z + mesh->pmax.z))); // center vertically
    mesh->translate(params.center_lbm);
    mesh->set_center(mesh->get_center_of_mass());
    return mesh;
}

} // namespace MeshLoader
