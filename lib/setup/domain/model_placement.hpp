#pragma once
#include "utilities.hpp"
#include "setup/domain/domain_plan.hpp"
#include <memory>
#include <string>

/// @brief The transform the core's voxelize_stl() gives the planned model, for other meshes of the same assembly.
///
/// voxelize_stl() rotates the model, scales its longest side to the voxel size and moves its bounding box center to
/// the planned center. Meshes loaded with the same transform (a rotor, the wheels of a car) keep their place relative
/// to the model, as in the CAD file they come from.
class ModelPlacement {
public:
    /// @brief The placement of the plan's model (reads its STL).
    /// @throws SetupError if the plan has no model.
    static ModelPlacement of(const DomainPlan& plan);

    /// @param rotated_model the model, rotated but not scaled or moved (read_stl(path, 1.0f, rotation))
    /// @param rotation      the rotation the model was read with
    /// @param voxel_size    the model's longest side in cells
    /// @param center        the model's bounding box center in cells
    ModelPlacement(const Mesh& rotated_model, const float3x3& rotation, float voxel_size, const float3& center);

    /// An STL of the same assembly with this transform (read_stl() exits if it cannot read the file).
    std::unique_ptr<Mesh> load(const std::string& path) const;

    /// @brief An STL in other coordinates than the model's: rotated and scaled like it, its own bounding box center
    /// moved to the model's plus an offset.
    /// @param offset in cells
    std::unique_ptr<Mesh> load_centered(const std::string& path, const float3& offset) const;

    float cells_per_unit() const { return cells_per_unit_; }       ///< cells per STL unit
    const float3x3& rotation() const { return rotation_; }
    const float3& center() const { return center_; }                ///< the model's bounding box center in cells
    const float3& translation() const { return translation_; }      ///< in cells, after rotation and scaling

private:
    float3x3 rotation_;
    float cells_per_unit_;
    float3 center_;
    float3 translation_;
};
