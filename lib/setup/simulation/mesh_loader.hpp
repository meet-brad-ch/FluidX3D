#pragma once

#include "setup/core/types.hpp"
#include "setup/domain/domain_plan.hpp"
#include "setup/units/units.hpp"
#include <memory>

/// @brief Loads meshes placed like the planned model (internal, for Simulation's voxelization).
class MeshLoader {
public:
    /// The rotation about the mirror axis that, with the negation, makes the mirror image.
    static constexpr si::Angle half_turn{ au::degrees(180.0) };

    /// @brief Makes the full mesh of a half model: the half is rotated and scaled as planned, then completed by its
    /// mirror image across the plane normal to an axis, and moved to the planned centre.
    ///
    /// The half is read as the core's own half-model setup reads it (read_stl() with repositioning, as upstream's
    /// nasa_crm): centred on the origin, its longest side scaled to the planned length. It is then shifted by half its
    /// extent along the mirror axis, so its face on the symmetry plane lies at 0, and mirrored there: the two halves
    /// meet at the plane wherever the STL's own origin is. The whole mesh is centred along z and moved to the planned
    /// centre.
    ///
    /// read_stl() ends the program with print_error() when the plan's file does not exist or is corrupt.
    /// @param plan the plan, with the model's file, rotation, scale and centre
    /// @param mirror the axis normal to the symmetry plane
    /// @return the full mesh, in cells (lattice coordinates)
    static std::unique_ptr<Mesh> load_mirrored(const DomainPlan& plan, Axis mirror) {
        // the core's repositioning read: the half centred on the origin, its longest side scaled to the planned length
        const float3 fit_box{ 1.0f }; // read_stl() uses the box only to fit a size of 0; the size here is above 0
        const float3 origin{ 0.0f };
        const std::unique_ptr<Mesh> half(read_stl(plan.stl_path, fit_box, origin, plan.rotation_matrix, plan.lbm_reference_size));

        // the symmetry plane at the mesh edge: the half shifted by half its extent along the mirror axis only. (The
        // core's float3 * float3 is a dot product, so the shift is built component by component.)
        const float along_x{ mirror == Axis::X ? 1.0f : 0.0f };
        const float along_y{ mirror == Axis::Y ? 1.0f : 0.0f };
        const float along_z{ mirror == Axis::Z ? 1.0f : 0.0f };
        const float3 axis{ along_x, along_y, along_z };
        const float3 extent{ half->pmax - half->pmin };
        const float3 shift{ -0.5f * along_x * extent.x, -0.5f * along_y * extent.y, -0.5f * along_z * extent.z };
        half->translate(shift);

        const uint32_t full_triangles{ 2u * half->triangle_number };
        std::unique_ptr<Mesh> mesh{ std::make_unique<Mesh>(full_triangles, float3(0.0f)) };
        for(uint32_t i = 0u; i < half->triangle_number; i++) {
            mesh->p0[i] = half->p0[i];
            mesh->p1[i] = half->p1[i];
            mesh->p2[i] = half->p2[i];
        }

        // the mirrored copy: rotated by 180 degrees about the mirror axis, then negated
        const float half_turn_deg{ static_cast<float>(half_turn.in(au::degrees)) }; // 180.0f exactly
        const float half_turn_rad{ radians(half_turn_deg) }; // the core's float conversion, as before
        half->rotate(float3x3(axis, half_turn_rad));
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
