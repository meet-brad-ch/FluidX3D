#pragma once
#include "utilities.hpp"
#include "setup/core/types.hpp"
#include "setup/domain/domain.hpp"
#include "setup/domain/lattice.hpp"
#include <optional>

// The simulation domain computed from a Domain: grid size, geometry placement, reference sizes, and the file to
// voxelize (Simulation::plan()).
struct DomainPlan {
    float3 stl_size_si{};           // geometry size in m (without geometry: the domain size)
    uint32_t Nx{}, Ny{}, Nz{};      // domain size in cells
    uint3 gpus = uint3(1u, 1u, 1u); // the grid's split among GPUs (Domain::gpus())
    uint3 base_grid{};              // geometry size in cells without clearances (SDF resolution)
    float3 center_lbm{};            // geometry center in cells
    float3x3 rotation_matrix{};
    float32_t lbm_reference_size{}; // reference length in cells
    float32_t si_reference_size{};  // reference length in m
    string stl_path;                // geometry file, resolved in resources/ (empty without geometry)
    string voxelization_path;       // file for voxelize(): the STL, or its cached SDF
    bool voxelize_sdf = false;
    float32_t voxel_size{};         // the geometry's longest side in cells, as voxelize_stl/voxelize_sdf expect
    std::optional<Axis> mirror;     // a half model, completed by its mirror image (Model::mirrored())
};

/// Computes the DomainPlan of a Domain: a box without a model, a model with clearances() around it, or a model in a
/// size() of the domain.
class DomainPlanner {
public:
    /// @param lattice the example's device memory per cell
    /// @throws SetupError for conflicting settings, a missing geometry file or a grid over the memory limit
    static DomainPlan plan(const Domain& domain, LatticeMemory lattice);

    /// The voxelization file and kernel; with clearances() the STL is converted to a cached SDF at the base grid resolution.
    static void choose_voxelization(DomainPlan& plan, const Domain& domain);

private:
    static DomainPlan plan_box(const Domain& domain, LatticeMemory lattice);
    static DomainPlan plan_with_clearances(const Domain& domain, const Model& model, LatticeMemory lattice);
    static DomainPlan plan_in_size(const Domain& domain, const Model& model, LatticeMemory lattice);

    /// vram(): the largest grid with the aspect ratio that fits; cell_size(): whole cells of the cell size along each
    /// side of size_m (rounded), which must fit the memory limit (throws SetupError)
    static uint3 grid_for_resolution(const Domain& domain, const float3& aspect, const float3& size_m, LatticeMemory lattice);

    template<typename Vector> static auto along(const Vector& v, Axis axis) { // the component along an axis
        return axis == Axis::X ? v.x : axis == Axis::Y ? v.y : v.z;
    }

    /// The domain's center in the core's cell coordinates (cell i's center is at i), as the core's lbm.center()
    static float3 center_of(const uint3& cells) {
        return float3(0.5f * (float32_t)cells.x - 0.5f, 0.5f * (float32_t)cells.y - 0.5f, 0.5f * (float32_t)cells.z - 0.5f);
    }

    /// A model's bounding box in its own units, turned as it is voxelized
    struct ModelBox {
        float3 size;           ///< after the whole rotation
        float3 unpitched_size; ///< after Model::rotation(), before the angle of attack: where Model::length() is measured
        float32_t scale_size;  ///< what voxelize_stl() scales (the longest side after the rotation), or voxelize_sdf() (the grid's longest side)
    };
    /// @param path the model's file: an STL, or an SDF, whose grid is the model's box (as read_sdf() computes it)
    static ModelBox model_box(const Model& model, const std::string& path);
};
