#pragma once
#include "utilities.hpp"
#include "setup/core/types.hpp"
#include "setup/config/simulation_config.hpp"
#include "setup/domain/lattice.hpp"

// The simulation domain computed from a SimulationConfig: grid size, geometry placement, reference sizes, and the file
// to voxelize. SimulationSetup::Results is this struct.
struct DomainPlan {
    float3 stl_size_si{};           // geometry size in m (without geometry: the domain size)
    uint32_t Nx{}, Ny{}, Nz{};      // domain size in cells
    uint3 base_grid{};              // geometry size in cells without clearances (SDF resolution)
    float3 center_lbm{};            // geometry center in cells
    float3x3 rotation_matrix{};
    float32_t lbm_reference_size{}; // reference length in cells
    float32_t si_reference_size{};  // reference length in m
    string stl_path;                // geometry file, resolved in resources/ (empty without geometry)
    string voxelization_path;       // file for voxelize(): the STL, or its cached SDF
    bool voxelize_sdf = false;
    float32_t voxel_size{};         // the geometry's longest side in cells, as voxelize_stl/voxelize_sdf expect
};

// Computes the DomainPlan for the config's mode (GEOMETRY_BASED, ASPECT_RATIO or DOMAIN_ONLY).
class DomainPlanner {
public:
    // lattice: the example's device memory per cell. Throws SetupError (geometry file missing, grid over the memory limit).
    static DomainPlan plan(const SimulationConfig& config, LatticeMemory lattice);

    // voxelization file and kernel; in GEOMETRY_BASED mode the STL is converted to a cached SDF at the base grid resolution
    static void choose_voxelization(DomainPlan& plan, const SimulationConfig& config);

private:
    static DomainPlan plan_domain_only(const SimulationConfig& config, LatticeMemory lattice);
    static DomainPlan plan_aspect_ratio(const SimulationConfig& config, LatticeMemory lattice);
    static DomainPlan plan_geometry_based(const SimulationConfig& config, LatticeMemory lattice);

    static float3x3 rotation(const SimulationConfig& config, bool with_angle_of_attack = true);
    static float32_t reference_dimension(const SimulationConfig& config, const float3& size);
    static uint32_t reference_dimension(const SimulationConfig& config, const uint3& size);
    static uint3 grid_for_vram(const SimulationConfig& config, const float3& aspect, LatticeMemory lattice);
};
