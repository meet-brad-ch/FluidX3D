#pragma once
#include "utilities.hpp"
#include "setup/core/types.hpp"
#include "setup/domain/domain.hpp"
#include "setup/domain/lattice.hpp"
#include "setup/core/scalar_transport_rules.hpp"
#include "setup/units/units.hpp"
#include "setup/units/vector3.hpp"
#include <optional>
#include <string>

/// @file domain_plan.hpp
/// @brief The grid and the model's placement that a Domain gives (DomainPlan), and how they are computed
/// (DomainPlanner).

/// @brief The simulation domain computed from a Domain: the grid, the model's placement, the reference sizes and the
/// file to voxelize (Simulation::plan()).
///
/// It holds the core's numbers: cell counts, the core's cell coordinates and floats in metres, as the voxelizer and
/// the core's units take them.
struct DomainPlan {
    si::Position stl_size{};        ///< the model's size after its rotation (without a model: the domain's size), the core's float metres
    uint32_t Nx{};                  ///< the grid's cells along X
    uint32_t Ny{};                  ///< the grid's cells along Y
    uint32_t Nz{};                  ///< the grid's cells along Z
    uint3 gpus = uint3(1u, 1u, 1u); ///< the grid's split among GPUs along each axis (Domain::gpus(), or the planner's split)
    uint3 base_grid{};              ///< the model's size in cells without the clearances (the SDF's resolution; 0 without a model)
    float3 center_lbm{};            ///< the model's centre in the core's cell coordinates (cell i's centre is at i)
    float3x3 rotation_matrix{};     ///< the model's rotation, with its angle of attack (the identity without a model)
    float32_t lbm_reference_size{}; ///< the reference length in cells: the model's length, or the domain's longest side
    si::Length reference_length{};  ///< the reference length: the model's length, or the domain's longest side (the core's float metres)
    string stl_path{};              ///< the model's file, resolved in resources/ (empty without a model)
    string voxelization_path{};     ///< the file to voxelize: the STL, or its cached SDF
    bool voxelize_sdf = false;      ///< true when voxelization_path is an SDF
    float32_t voxel_size{};         ///< the model's longest side in cells, as voxelize_stl() and voxelize_sdf() expect
    std::optional<Axis> mirror{};   ///< the axis normal to the symmetry plane of a half model (Model::mirrored()), or none
};

/// @brief Computes the DomainPlan of a Domain: a box without a model, a model with clearances() around it, or a model
/// in a size() of the domain.
///
/// The physical lengths become the core's floats in metres where the planner starts (metres()); the rest of the
/// planning is in the core's float order, so a grid and a voxelization are the same as the core's own.
class DomainPlanner {
public:
    /// @brief Plans the grid and the model's placement.
    ///
    /// A lattice whose DDF buffer would be larger than a device accepts (LatticeMemory::max_buffer) is split into
    /// domains (split_for_buffers()); the domains then run on the devices there are, all on one card if it is alone.
    /// @param domain the box, its model, its resolution and its GPUs
    /// @param lattice the build's device memory per cell, its DDF buffer per cell and the devices' buffer limit
    /// @return the plan
    /// @throws SetupError for conflicting settings (Domain::validate()), a memory amount that is not whole MiB, a
    /// missing or unreadable geometry file, a cell larger than the domain, a grid over the memory limit, or a DDF
    /// buffer over the device's buffer limit that no split fits
    static DomainPlan plan(const Domain& domain, LatticeMemory lattice);

    /// @brief Sets the file to voxelize and its kind. With clearances() an STL is converted to a cached SDF at the
    /// base grid's resolution.
    /// @param plan the plan to complete: its voxelization_path and voxelize_sdf are set
    /// @param domain the plan's domain
    /// @throws SetupError when the SDF cannot be made (an STL that cannot be read, a base grid without cells, a cache
    /// file that cannot be written); the SDF cache prints the cause
    static void choose_voxelization(DomainPlan& plan, const Domain& domain);

    /// @brief Gives the reason why a plan cannot run with the build's per-level forcing: LEVEL_FORCING indexes its
    /// levels by the domain's local z, so a split along z is refused. The core refuses only at the first
    /// set_level_forcing(), after the allocation; this refuses at once. The planner's own splits are along x or y.
    /// @param plan a planned grid with its split among GPUs
    /// @param build the build's extensions
    /// @return the reason, or none when the plan can run
    static std::optional<std::string> level_forcing_refusal(const DomainPlan& plan, const TransportBuild& build);

    /// @brief Gives the cells of one domain, its halo included, as the core sizes it (LBM::LBM()).
    /// @param grid the whole grid
    /// @param gpus the domains along each axis, at least 1 each
    /// @param halo_layers the halo layers on each side of a cut (LatticeMemory::halo_layers)
    /// @return the cells: each axis's share of the grid, plus 2 halo_layers along each split axis, multiplied
    static std::uint64_t domain_cells(GridSize grid, uint3 gpus, std::uint32_t halo_layers);

private:
    /// @brief Splits the lattice into domains so that each domain's DDF buffer is within the device's limit.
    ///
    /// With gpus() set (other than 1, 1, 1) the split is the caller's, and it is refused when a domain's buffer is
    /// still over the limit. Otherwise a lattice that fits stays whole. One that does not fit is split along its
    /// longest horizontal axis (the smallest faces to exchange) into the fewest domains that fit, and the grid along
    /// that axis is trimmed to a multiple of them at its far side (at most domains - 1 cells), so the core does not
    /// trim it again; the model's centre stays where it is. A domain holds its share of the cells and
    /// LatticeMemory::halo_layers on each side along a split axis, as the core builds it (LBM::LBM()). The planner
    /// never splits along z.
    /// @param plan the plan, its grid planned; its gpus and grid are set
    /// @param domain the domain, with its gpus()
    /// @param lattice the DDF buffer's bytes per cell and the devices' buffer limit
    /// @throws SetupError when a domain's buffer is over the limit with the caller's split, or with any split
    static void split_for_buffers(DomainPlan& plan, const Domain& domain, LatticeMemory lattice);

    /// @brief Splits the grid along its longest horizontal axis into the fewest domains whose DDF buffers fit the
    /// limit, and trims the grid along that axis to a multiple of them.
    /// @param plan the plan, its grid planned; its gpus and grid are set
    /// @param lattice the DDF buffer's bytes per cell and the halo layers
    /// @param limit the devices' buffer limit, more than 0
    /// @throws SetupError when no split along that axis fits the limit
    static void split_along_longest(DomainPlan& plan, LatticeMemory lattice, si::MemoryAmount limit);

    /// @brief Gives the size of one domain's DDF buffer.
    /// @param grid the whole grid
    /// @param gpus the domains along each axis
    /// @param lattice the DDF buffer's bytes per cell and the halo layers
    /// @return the domain's cells times the bytes per cell
    static si::MemoryAmount domain_buffer_memory(GridSize grid, uint3 gpus, LatticeMemory lattice);

    /// @brief Gives a memory amount in whole MB for a message, as the core prints its MB.
    /// @param amount a memory amount
    /// @return the whole MiB (the core's MB), rounded down, as text
    static std::string whole_mb_text(si::MemoryAmount amount);

    /// @brief Gives a length as the core's float in metres: where the planner's float arithmetic starts.
    /// @param x a length
    /// @return the length in m, as a float
    static float metres(si::Length x);

    /// @brief Gives three lengths as the core's floats in metres.
    /// @param x the length along X
    /// @param y the length along Y
    /// @param z the length along Z
    /// @return the lengths in m, as a float3
    static float3 metres(si::Length x, si::Length y, si::Length z);

    /// @brief Plans Domain::box(): the grid from the size in metres.
    /// @param domain a box without a model, with its resolution
    /// @param lattice the memory per cell
    /// @return the plan
    /// @throws SetupError when the domain has no size, for a cell larger than the domain, or a grid over the memory
    /// limit (grid_for_resolution())
    static DomainPlan plan_box(const Domain& domain, LatticeMemory lattice);

    /// @brief Plans Domain::around(model).clearances(): the model's STL in metres plus the clearances.
    /// @param domain a domain around a model, with its clearances and its resolution
    /// @param model the domain's model, its STL in metres
    /// @param lattice the memory per cell
    /// @return the plan
    /// @throws SetupError when the geometry cannot be read or the grid does not fit (GeometryScaler)
    static DomainPlan plan_with_clearances(const Domain& domain, const Model& model, LatticeMemory lattice);

    /// @brief Plans Domain::around(model).size(): the grid from the size in metres, the model scaled to its real
    /// length and placed.
    /// @param domain a domain around a model, with its size, its placement and its resolution
    /// @param model the domain's model, with its Model::length()
    /// @param lattice the memory per cell
    /// @return the plan
    /// @throws SetupError when the domain has no size or the model no Model::length(), for a missing or unreadable
    /// geometry file, a cell larger than the domain or a grid over the memory limit
    static DomainPlan plan_in_size(const Domain& domain, const Model& model, LatticeMemory lattice);

    /// @brief Gives the model's centre in a size() domain: at the gaps (X centred, the bounding box minimum at the
    /// gap to the inlet and the floor), or centred and moved by model_offset().
    /// @param domain the domain, with its gaps or model offset
    /// @param domain_centre the domain's centre in the core's cell coordinates
    /// @param half_size half the model's size in cells
    /// @param length the model's length in m
    /// @param length_cells the model's length in cells
    /// @return the centre in the core's cell coordinates
    static float3 model_centre(const Domain& domain, const float3& domain_centre, const float3& half_size, float32_t length, float32_t length_cells);

    /// @brief Gives the grid of a box. With vram(): the largest grid with the aspect ratio that fits. With
    /// cell_size(): whole cells of the cell size along each side of size_m (rounded to the nearest), which must fit
    /// the memory limit.
    /// @param domain the domain, with its resolution setting
    /// @param aspect the sides' proportions, the longest 1
    /// @param size_m the sides in m
    /// @param lattice the memory per cell
    /// @return the cells along X, Y and Z
    /// @throws SetupError when a side is less than half a cell, or the grid does not fit the memory limit
    static uint3 grid_for_resolution(const Domain& domain, const float3& aspect, const float3& size_m, LatticeMemory lattice);

    /// @brief Gives the component of a float vector (sizes in m or in mesh units) along an axis.
    /// @param v the vector
    /// @param axis the coordinate axis of the component
    /// @return the component
    static float along(const float3& v, Axis axis) {
        float component{ v.z };
        if(axis == Axis::X) {
            component = v.x;
        } else if(axis == Axis::Y) {
            component = v.y;
        }
        return component;
    }

    /// @brief Gives the component of a cell count vector along an axis.
    /// @param v the cells along X, Y and Z
    /// @param axis the coordinate axis of the component
    /// @return the component
    static uint32_t along(const uint3& v, Axis axis) {
        uint32_t component{ v.z };
        if(axis == Axis::X) {
            component = v.x;
        } else if(axis == Axis::Y) {
            component = v.y;
        }
        return component;
    }

    /// @brief Gives the domain's centre in the core's cell coordinates (cell i's centre is at i), as the core's
    /// lbm.center().
    /// @param cells the grid
    /// @return the centre: half the cells minus half a cell along each axis
    static float3 center_of(const uint3& cells) {
        const float x{ 0.5f * static_cast<float32_t>(cells.x) - 0.5f };
        const float y{ 0.5f * static_cast<float32_t>(cells.y) - 0.5f };
        const float z{ 0.5f * static_cast<float32_t>(cells.z) - 0.5f };
        const float3 centre{ x, y, z };
        return centre;
    }

    /// A model's bounding box in its own units, turned as it is voxelized.
    struct ModelBox {
        float3 size{};           ///< the box after the whole rotation
        float3 unpitched_size{}; ///< the box after Model::rotation(), before the angle of attack: where Model::length() is measured
        float32_t scale_size{};  ///< what voxelize_stl() scales (the longest side after the rotation), or voxelize_sdf() (the grid's longest side)
    };

    /// @brief Gives the bounding box of a model's file.
    /// @param model the model, for its rotation
    /// @param path the model's file: an STL, or an SDF, whose grid is the model's box (as read_sdf() computes it)
    /// @return the box
    /// @throws SetupError when an SDF file's header cannot be read or has a grid without cells
    static ModelBox model_box(const Model& model, const std::string& path);

    /// @brief Gives the size of a box turned about its centre: the extent of its 8 turned corners.
    /// @param world the box's size
    /// @param rotation the rotation of the model's file, about the box's centre
    /// @return the turned box's size along X, Y and Z
    static float3 turned_size(const float3& world, const float3x3& rotation);
};
