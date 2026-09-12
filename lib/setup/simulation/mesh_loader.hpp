#pragma once

#include "setup/core/types.hpp"
#include "setup/domain/domain_plan.hpp"
#include <memory>

/// Meshes placed like the planned model.
class MeshLoader {
public:
    /// @brief The full mesh of a half model: rotated and scaled as planned, completed by its mirror image across the
    /// plane normal to mirror, and centered on the planned center.
    static std::unique_ptr<Mesh> load_mirrored(const DomainPlan& plan, Axis mirror) {
        const std::unique_ptr<Mesh> half(read_stl(plan.stl_path, 1.0f, plan.rotation_matrix));
        half->scale(plan.lbm_reference_size / half->get_max_size());

        // the symmetry plane at the mesh edge
        const float3 axis(mirror == Axis::X ? 1.0f : 0.0f, mirror == Axis::Y ? 1.0f : 0.0f, mirror == Axis::Z ? 1.0f : 0.0f);
        half->translate(-0.5f * axis * (half->pmax - half->pmin));

        auto mesh = std::make_unique<Mesh>(2u * half->triangle_number, float3(0.0f));
        for(uint32_t i = 0u; i < half->triangle_number; i++) {
            mesh->p0[i] = half->p0[i];
            mesh->p1[i] = half->p1[i];
            mesh->p2[i] = half->p2[i];
        }

        // the mirrored copy: rotated by 180 degrees about the mirror axis, then negated
        half->rotate(float3x3(axis, radians(180.0f)));
        for(uint32_t i = 0u; i < half->triangle_number; i++) {
            mesh->p0[half->triangle_number + i] = -half->p0[i];
            mesh->p1[half->triangle_number + i] = -half->p1[i];
            mesh->p2[half->triangle_number + i] = -half->p2[i];
        }

        mesh->find_bounds();
        mesh->translate(float3(0.0f, 0.0f, -0.5f * (mesh->pmin.z + mesh->pmax.z))); // centered vertically
        mesh->translate(plan.center_lbm);
        mesh->set_center(mesh->get_center_of_mass());
        return mesh;
    }
};
