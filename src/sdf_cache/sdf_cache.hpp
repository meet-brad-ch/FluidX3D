#pragma once

#include <cstdint>
#include <string>

/**
 * @file sdf_cache.hpp
 * @brief SDF caching for FluidX3D
 *
 * Signed distance fields (SDFs) of STL models are generated with SDFGen, which takes seconds to minutes for a large
 * model, and cached on disk. A cached SDF is found by its file name, which carries the model, the grid and a key
 * (SDFCacheKey) of everything that shapes the field, so a changed model, grid, padding, mesh repair setting or
 * generator version gives a new file instead of a stale hit.
 */

/**
 * @brief Configuration of the SDF cache
 */
struct SDFCacheConfig {
    std::string cache_directory = "resources/sdf_cache/"; ///< Directory of the SDF files, relative to the working directory
    bool fix_mesh = false; ///< Repair a non-watertight mesh (fill its holes) before the SDF is generated; the sign of the field is unreliable otherwise
    bool verbose = false;  ///< Report cache hits and misses, the mesh analysis and the generated file on stdout
};

/**
 * @brief SDF cache manager
 *
 * Returns the SDF of an STL model on a requested grid, generating and caching it on the first request. The grid is
 * dimension-driven: the caller (DomainPlanner in the Setup API) computes the model's base grid in cells, and the
 * SDF is generated at exactly that size, plus padding cells on every side. The cache directory is created if it
 * does not exist.
 *
 * Cached files are named `STL_sdf_NXxNYxNZ_KEY.sdf` after the STL file's name, the grid (which includes the padding)
 * and the key of SDFCacheKey::format(), e.g. `hill_sdf_502x502x90_d1332ade.sdf`. They are the core's binary SDF format (see read_sdf(): a header of the grid and its bounds,
 * then one float per cell, negative inside the model).
 */
class SDFCacheManager {
public:
    /**
     * @brief Construct the cache manager
     *
     * Creates the cache directory if it does not exist.
     *
     * @param config The cache directory, the mesh repair flag and the verbosity
     */
    explicit SDFCacheManager(const SDFCacheConfig& config = {});

    /**
     * @brief Get the cached SDF of a model, or generate it
     *
     * 1. Compute the cache key from the STL, the grid, the padding and the settings
     * 2. Look for a cached file with that key and grid
     * 3. If found, return its path
     * 4. If not, generate the SDF at the target resolution, write it to the cache and return its path
     *
     * The cell size is the model's extent along X divided by nx; ny and nz should follow from the same cell size,
     * as DomainPlanner computes them, so the grid covers the model. The padding is at least one cell.
     *
     * @param stl_path Path to the binary STL file
     * @param nx Target SDF dimension X, in cells across the model (the LBM base grid, without clearances)
     * @param ny Target SDF dimension Y
     * @param nz Target SDF dimension Z
     * @param padding Padding cells added on each side of the model (default 1, the minimum)
     * @return Path to the SDF file, cached or newly generated; empty if the file is not a binary STL or cannot be written
     */
    std::string get_or_generate(const std::string& stl_path, uint32_t nx, uint32_t ny, uint32_t nz, int32_t padding = 1);

private:
    SDFCacheConfig config_; ///< The cache directory and the generation settings

    /**
     * @brief Find the cached SDF of a key and grid
     *
     * @param stl_basename Basename of the STL file (e.g. "hill" for "hill.stl")
     * @param key Cache key (SDFCacheKey::compute())
     * @param nx Target dimension X, without padding
     * @param ny Target dimension Y, without padding
     * @param nz Target dimension Z, without padding
     * @param padding Padding cells on each side, part of the file's grid
     * @return Path to the cached SDF, or empty if there is none
     */
    std::string find_cached(const std::string& stl_basename, uint64_t key, uint32_t nx, uint32_t ny, uint32_t nz, int32_t padding) const;

    /**
     * @brief Generate the SDF of a model into the cache
     *
     * Reads the STL, welds its duplicate vertices, analyzes and optionally repairs the mesh, lays the grid around
     * the model's center and runs SDFGen (on the GPU, or on the CPU if the GPU fails).
     *
     * @param stl_path Path to the binary STL file
     * @param key Cache key, part of the file name
     * @param nx Target dimension X, without padding
     * @param ny Target dimension Y, without padding
     * @param nz Target dimension Z, without padding
     * @param padding Padding cells on each side
     * @return Path to the generated SDF, or empty if the STL cannot be read or the file cannot be written
     */
    std::string generate(const std::string& stl_path, uint64_t key, uint32_t nx, uint32_t ny, uint32_t nz, int32_t padding) const;
};
