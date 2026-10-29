#pragma once
#include "setup/core/types.hpp"
#include "setup/core/setup_error.hpp"
#include "setup/domain/lattice.hpp"
#include "setup/units/units.hpp"
#include "setup/units/vector3.hpp"
#include "utilities.hpp"

/// @brief Scale between an STL geometry in metres and the lattice (internal, for DomainPlanner's clearances mode).
///
/// The cell size comes from a VRAM budget (for the geometry plus its clearances) or is given; the domain size and the
/// geometry's center follow in cells. The geometry is measured after its rotation, as the core voxelizes it. The
/// reference length (units, Reynolds number) is its extent along Y.
///
/// The interface takes and gives physical quantities. Inside, the lengths are float metres: the core's read_stl()
/// measures the mesh in floats, and the cell counts come from the same float operations as before the units port, so
/// the grid stays the same.
class GeometryScaler {
public:
    /// The fluid around the geometry.
    struct Clearances {
        si::Length bottom{ au::ZERO }; ///< below the geometry
        si::Length top{ au::ZERO };    ///< above it
        si::Length sides{ au::ZERO };  ///< on each of the four sides
    };

    /// @brief Finds the cell size so that the geometry with its clearances fits a VRAM budget.
    /// @param stl_path the STL file
    /// @param budget the device memory budget, more than 0
    /// @param clearances the fluid around the geometry
    /// @param lattice the memory per cell
    /// @param rotation the geometry's rotation
    /// @throws SetupError when the STL file does not exist or cannot be read, or the budget gives no grid
    /// (LatticeMemory::grid_for_memory())
    GeometryScaler(
        const string& stl_path,
        si::MemoryAmount budget,
        const Clearances& clearances,
        LatticeMemory lattice,
        const float3x3& rotation = float3x3(1.0f)
    );

    /// @brief Takes a given cell size; the grid with its clearances must fit a memory limit.
    /// @param stl_path the STL file
    /// @param cell the edge of one cubic cell, more than 0
    /// @param limit the device memory that the grid may use
    /// @param clearances the fluid around the geometry
    /// @param lattice the memory per cell
    /// @param rotation the geometry's rotation
    /// @throws SetupError when the STL file does not exist or cannot be read, or the grid does not fit the limit
    GeometryScaler(
        const string& stl_path,
        si::Length cell,
        si::MemoryAmount limit,
        const Clearances& clearances,
        LatticeMemory lattice,
        const float3x3& rotation = float3x3(1.0f)
    );

    /// @brief Gives a length in whole cells, rounded to the nearest as the core's to_uint() rounds.
    /// @param length a length, 0 or more
    /// @return the length in whole cells
    uint32_t meters_to_cells(si::Length length) const {
        const float length_m{ metres(length) };
        const uint32_t whole_cells{ cells_of(length_m) };
        return whole_cells;
    }

    /// @brief Gives the size of the geometry's bounding box after the rotation.
    /// @return the bounding box's extents along X, Y and Z
    si::Position stl_size() const {
        const si::Position size{ length_of(stl_size_m_.x), length_of(stl_size_m_.y), length_of(stl_size_m_.z) };
        return size;
    }

    /// @brief Gives the size of the geometry's bounding box in whole cells, without the clearances.
    /// @return the cells along X, Y and Z
    uint3 stl_cells() const { return stl_size_lbm_; }

    /// @brief Gives the reference length (the geometry's extent along Y) in cells.
    /// @return the reference length in cells
    float32_t reference_cells() const { return lbm_reference_size_; }

    /// @brief Gives the reference length: the geometry's extent along Y.
    /// @return the reference length
    si::Length reference_length() const {
        const si::Length length{ length_of(si_reference_size_m_) };
        return length;
    }

    /// @brief Gives the cell size.
    /// @return the edge of one cubic cell
    si::Length cell_size() const {
        const si::Length cell{ length_of(meters_per_cell_) };
        return cell;
    }

    /// @brief Gives the domain's size: the geometry plus the clearances, each rounded to whole cells.
    /// @param clearances the fluid around the geometry
    /// @return the geometry plus the clearances, in cells
    uint3 calculate_domain_size(const Clearances& clearances) const;

    /// @brief Gives the geometry's centre in the domain: centred in X and Y, on the bottom clearance.
    /// @param domain_size the domain in cells
    /// @param clearances the fluid around the geometry
    /// @return the geometry's centre in the core's cell coordinates (cell i's centre is at i)
    float3 calculate_center(const uint3& domain_size, const Clearances& clearances) const;

private:
    /// The bytes of one MiB, the core's MB.
    static constexpr float32_t bytes_per_mib = 1048576.0f;

    string stl_path_{};                    ///< the STL file
    float3 stl_size_m_{};                  ///< the geometry's size in metres (the core's float)
    uint3 stl_size_lbm_{ 0u, 0u, 0u };     ///< the geometry's size in cells, without clearances

    float32_t lbm_reference_size_{};       ///< the reference length in cells
    float32_t si_reference_size_m_{};      ///< the reference length in metres (the core's float)
    float32_t meters_per_cell_{};          ///< the cell size in metres (the core's float)

    si::MemoryAmount budget_{ au::ZERO };  ///< the VRAM budget (zero in the given cell size mode)
    float32_t bottom_m_{};                 ///< the clearance below the geometry, in metres
    float32_t top_m_{};                    ///< the clearance above the geometry, in metres
    float32_t sides_m_{};                  ///< the clearance on each of the four sides, in metres
    LatticeMemory lattice_{};              ///< the memory per cell
    float3x3 rotation_{ 1.0f };            ///< the geometry's rotation

    /// @brief Gives a length as the float metres of the scaler's float operations.
    /// @param length the length to convert
    /// @return the length in metres, as a float
    static float32_t metres(si::Length length) {
        const double length_m{ length.in(au::meters) };
        const float32_t length_float{ static_cast<float32_t>(length_m) };
        return length_float;
    }

    /// @brief Gives float metres as a length.
    /// @param length_m a length in metres
    /// @return the length
    static si::Length length_of(float32_t length_m) {
        const double length_double{ static_cast<double>(length_m) };
        const si::Length length{ au::meters(length_double) };
        return length;
    }

    /// @brief Gives a length in float metres in whole cells, rounded to the nearest as the core's to_uint() rounds.
    /// @param length_m a length in metres, 0 or more
    /// @return the length in whole cells
    uint32_t cells_of(float32_t length_m) const {
        const float32_t cells{ length_m / meters_per_cell_ };
        const uint32_t whole_cells{ to_uint(cells) };
        return whole_cells;
    }

    /// @brief Reads stl_size_m_ from the bounding box of the rotated geometry.
    /// @throws SetupError when the STL file does not exist or cannot be read
    void read_stl_size();

    /// @brief Finds the cell size of the VRAM budget mode: the geometry with its clearances fits budget_.
    /// @throws SetupError when the STL file does not exist or cannot be read, or the budget gives no grid
    void load_stl_and_calculate_scaling();

    /// @brief Sets the scale of the given cell size mode and checks the grid against the memory limit.
    /// @param cell_m the cell size in metres
    /// @param limit the device memory that the grid may use
    /// @throws SetupError if the grid does not fit the limit
    void calculate_from_voxel_size(float32_t cell_m, si::MemoryAmount limit);

    /// @brief Gives the smallest cell size whose grid (geometry and clearances) fits a memory limit, for the error
    /// message.
    /// @param limit the device memory that the grid may use
    /// @return the smallest cell size in metres
    float32_t calculate_min_voxel_size(si::MemoryAmount limit) const;

    /// @brief Gives the reference length of a size: its extent along Y.
    /// @param size a geometry's size
    /// @return its extent along Y
    static float32_t get_reference_dimension(const float3& size) { return size.y; }
};
