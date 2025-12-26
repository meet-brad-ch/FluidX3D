#pragma once

#include <string>

/**
 * @file sdf_cache.hpp
 * @brief SDF caching system for FluidX3D
 *
 * Provides automatic caching of SDF files based on STL content and generation parameters.
 * Cache keys are computed using xxHash64 of all parameters affecting SDF output.
 */

/**
 * @brief Configuration for SDF cache manager
 */
struct SDFCacheConfig {
    std::string cache_directory = "resources/sdf_cache/";
    bool enable_cache = true;
    bool force_regenerate = false;
    bool verbose = false;
};

/**
 * @brief SDF cache manager
 *
 * Manages automatic generation and caching of SDF files.
 * Uses hash-based lookup to detect when STL or parameters change.
 */
class SDFCacheManager {
public:
    /**
     * @brief Construct cache manager with config
     */
    explicit SDFCacheManager(const SDFCacheConfig& config = {});

    /**
     * @brief Get cached SDF or generate new one at specified resolution
     *
     * This is a dimension-driven approach: the caller (GeometrySetup) calculates
     * the target LBM base grid size, and SDF Cache generates to match that exact size.
     *
     * 1. Compute cache key from STL + target dimensions
     * 2. Check if cached SDF exists with matching hash
     * 3. If found, return cached path
     * 4. If not found, generate SDF at target resolution, then return new path
     *
     * @param stl_path Path to STL file
     * @param target_nx Target SDF dimension X (LBM base grid, excluding clearances)
     * @param target_ny Target SDF dimension Y (LBM base grid, excluding clearances)
     * @param target_nz Target SDF dimension Z (LBM base grid, excluding clearances)
     * @param padding Padding cells to add around SDF (default: 1)
     * @return Path to SDF file (cached or newly generated)
     */
    std::string get_or_generate(const std::string& stl_path, uint32_t target_nx, uint32_t target_ny, uint32_t target_nz, int32_t padding = 1);

    /**
     * @brief Clear cached SDFs for a specific STL basename
     *
     * @param stl_basename Basename of STL file (e.g., "hill" from "hill.stl")
     * @return Number of files deleted
     */
    int clear_cache(const std::string& stl_basename);

    /**
     * @brief Clear all cached SDFs
     *
     * @return Number of files deleted
     */
    int clear_all_cache();

    /**
     * @brief Get cache statistics
     *
     * @param total_files Output: total number of cached SDF files
     * @param total_size_mb Output: total size of cache in MB
     */
    void get_cache_stats(int& total_files, float& total_size_mb);

    /**
     * @brief Set verbose output
     */
    void set_verbose(bool verbose);

private:
    SDFCacheConfig config_;

    /**
     * @brief Find cached SDF file matching the expected hash and dimensions
     *
     * @param stl_basename Basename of STL (e.g., "hill")
     * @param expected_hash Expected 64-bit hash (will be formatted as 8 hex chars)
     * @param nx Target dimension X
     * @param ny Target dimension Y
     * @param nz Target dimension Z
     * @return Path to cached SDF, or empty string if not found
     */
    std::string find_cached_sdf(const std::string& stl_basename, uint64_t expected_hash, uint32_t nx, uint32_t ny, uint32_t nz);

    /**
     * @brief Generate SDF at specified resolution
     *
     * @param stl_path Path to STL file
     * @param target_nx Target dimension X (base grid)
     * @param target_ny Target dimension Y (base grid)
     * @param target_nz Target dimension Z (base grid)
     * @param padding Padding cells to add
     * @return Path to generated SDF file
     */
    std::string generate_sdf(const std::string& stl_path, uint32_t target_nx, uint32_t target_ny, uint32_t target_nz, int32_t padding);
};
