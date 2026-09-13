#pragma once
#include "utilities.hpp"
#include "setup/core/types.hpp"
#include "setup/domain/domain.hpp"
#include "setup/domain/lattice.hpp"
#include <optional>

/// @brief The simulation domain computed from a Domain: the grid, the model's placement, the reference sizes, and the
/// file to voxelize (Simulation::plan()).
struct DomainPlan {
    float3 stl_size_si{};           ///< the model's size in m (without a model: the domain's)
    uint32_t Nx{};                  ///< the grid's cells along X
    uint32_t Ny{};                  ///< the grid's cells along Y
    uint32_t Nz{};                  ///< the grid's cells along Z
    uint3 gpus = uint3(1u, 1u, 1u); ///< the grid's split among GPUs (Domain::gpus())
    uint3 base_grid{};              ///< the model's size in cells without clearances (the SDF resolution)
    float3 center_lbm{};            ///< the model's center in the core's cell coordinates
    float3x3 rotation_matrix{};     ///< the model's rotation, with its angle of attack
    float32_t lbm_reference_size{}; ///< the reference length in cells
    float32_t si_reference_size{};  ///< the reference length in m
    string stl_path;                ///< the model's file, resolved in resources/ (empty without a model)
    string voxelization_path;       ///< the file to voxelize: the STL, or its cached SDF
    bool voxelize_sdf = false;      ///< whether voxelization_path is an SDF
    float32_t voxel_size{};         ///< the model's longest side in cells, as voxelize_stl() and voxelize_sdf() expect
    std::optional<Axis> mirror;     ///< a half model, completed by its mirror image (Model::mirrored())
};

/// @brief Computes the DomainPlan of a Domain: a box without a model, a model with clearances() around it, or a model
/// in a size() of the domain.
class DomainPlanner {
public:
    /// @brief Plans the grid and the model's placement.
    /// @param domain the domain
    /// @param lattice the example's device memory per cell
    /// @return the plan
    /// @throws SetupError for conflicting settings, a missing geometry file or a grid over the memory limit
    static DomainPlan plan(const Domain& domain, LatticeMemory lattice);

    /// @brief Sets the voxelization file and kernel; with clearances() the STL is converted to a cached SDF at the
    /// base grid resolution.
    /// @param plan the plan to complete
    /// @param domain its domain
    static void choose_voxelization(DomainPlan& plan, const Domain& domain);

private:
    /// @brief Domain::box(): the grid from the size in metres.
    /// @param domain the domain
    /// @param lattice the memory per cell
    /// @return the plan
    static DomainPlan plan_box(const Domain& domain, LatticeMemory lattice);

    /// @brief Domain::around(model).clearances(): the model's STL in metres plus the clearances.
    /// @param domain the domain
    /// @param model its model
    /// @param lattice the memory per cell
    /// @return the plan
    static DomainPlan plan_with_clearances(const Domain& domain, const Model& model, LatticeMemory lattice);

    /// @brief Domain::around(model).size(): the grid from the size in metres, the model scaled to its real length and placed.
    /// @param domain the domain
    /// @param model its model
    /// @param lattice the memory per cell
    /// @return the plan
    static DomainPlan plan_in_size(const Domain& domain, const Model& model, LatticeMemory lattice);

    /// @brief The grid of a box: vram(): the largest grid with the aspect ratio that fits; cell_size(): whole cells of
    /// the cell size along each side of size_m (rounded), which must fit the memory limit.
    /// @param domain the domain, with its resolution setting
    /// @param aspect the sides' proportions
    /// @param size_m the sides in metres
    /// @param lattice the memory per cell
    /// @return the grid
    /// @throws SetupError if the grid does not fit the memory limit
    static uint3 grid_for_resolution(const Domain& domain, const float3& aspect, const float3& size_m, LatticeMemory lattice);

    /// @brief The component of a vector along an axis.
    /// @tparam Vector a type with x, y and z
    /// @param v the vector
    /// @param axis the axis
    /// @return the component
    template<typename Vector> static auto along(const Vector& v, Axis axis) {
        return axis == Axis::X ? v.x : axis == Axis::Y ? v.y : v.z;
    }

    /// @brief The domain's center in the core's cell coordinates (cell i's center is at i), as the core's lbm.center().
    /// @param cells the grid
    /// @return the center
    static float3 center_of(const uint3& cells) {
        return float3(0.5f * (float32_t)cells.x - 0.5f, 0.5f * (float32_t)cells.y - 0.5f, 0.5f * (float32_t)cells.z - 0.5f);
    }

    /// A model's bounding box in its own units, turned as it is voxelized.
    struct ModelBox {
        float3 size;           ///< after the whole rotation
        float3 unpitched_size; ///< after Model::rotation(), before the angle of attack: where Model::length() is measured
        float32_t scale_size;  ///< what voxelize_stl() scales (the longest side after the rotation), or voxelize_sdf() (the grid's longest side)
    };

    /// @brief The bounding box of a model's file.
    /// @param model the model, for its rotation
    /// @param path the model's file: an STL, or an SDF, whose grid is the model's box (as read_sdf() computes it)
    /// @return the box
    static ModelBox model_box(const Model& model, const std::string& path);
};
