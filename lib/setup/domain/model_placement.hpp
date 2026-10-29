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
    /// @brief Gives the placement of the plan's model: it reads the model's STL with the plan's rotation and takes
    /// the plan's voxel size and centre.
    ///
    /// read_stl() ends the program with print_error() when the file cannot be read as an STL.
    /// @param plan a domain plan with a model (DomainPlanner::plan() of a Domain::around() domain)
    /// @return the placement
    /// @throws SetupError when the plan has no STL path, or the file does not exist
    static ModelPlacement of(const DomainPlan& plan);

    /// @brief Makes the placement from a rotated model and the place it gets in the lattice.
    /// @param rotated_model the model, rotated but not scaled or moved (read_stl(path, 1.0f, rotation))
    /// @param rotation the rotation that the model was read with
    /// @param voxel_size the model's longest side in cells, more than 0
    /// @param center the model's bounding box centre in cells (lattice coordinates)
    ModelPlacement(const Mesh& rotated_model, const float3x3& rotation, float voxel_size, const float3& center);

    /// @brief Reads an STL of the same assembly with the model's transform, so that it keeps its place relative to
    /// the model. read_stl() ends the program with print_error() when the file does not exist or is corrupt.
    /// @param path the STL file
    /// @return the mesh, rotated, scaled and moved like the model, in cells
    std::unique_ptr<Mesh> load(const std::string& path) const;

    /// @brief Reads an STL whose coordinates are not the model's: it is rotated and scaled like the model, and its
    /// own bounding box centre is moved to the model's centre plus an offset. read_stl() ends the program with
    /// print_error() when the file does not exist or is corrupt.
    /// @param path the STL file
    /// @param offset the offset of the mesh's centre from the model's centre, in cells
    /// @return the mesh, in cells
    std::unique_ptr<Mesh> load_centered(const std::string& path, const float3& offset) const;

    /// @brief Gives the scale of the meshes.
    /// @return the cells per STL unit
    float cells_per_unit() const { return cells_per_unit_; }

    /// @brief Gives the rotation of the meshes.
    /// @return the rotation that the meshes are read with
    const float3x3& rotation() const { return rotation_; }

    /// @brief Gives the model's place in the lattice.
    /// @return the model's bounding box centre in cells
    const float3& center() const { return center_; }

    /// @brief Gives the move of the meshes.
    /// @return the move in cells that comes after the rotation and the scaling
    const float3& translation() const { return translation_; }

private:
    float3x3 rotation_{ 1.0f }; ///< the rotation that the meshes are read with
    float cells_per_unit_{};    ///< the scale: cells per STL unit
    float3 center_{};           ///< the model's bounding box centre in cells
    float3 translation_{};      ///< the move in cells, after the rotation and the scaling
};
