#pragma once

#include <cstdint>
#include <cstddef>
#include <string>

/**
 * @file hash_utils.hpp
 * @brief Fast hashing utilities for STL file content and cache keys
 *
 * Provides xxHash64 implementation for generating cache keys.
 * Hash includes: STL vertex data + VRAM budget + padding + margin + clearances
 */

/**
 * @brief Compute xxHash64 of a memory buffer
 *
 * @param data Pointer to data buffer
 * @param length Length of data in bytes
 * @param seed Hash seed (default: 0)
 * @return 64-bit hash value
 */
uint64_t xxhash64(const void* data, size_t length, uint64_t seed = 0);

/**
 * @brief Compute xxHash64 of an STL file
 *
 * Only hashes vertex data (skips 80-byte header).
 *
 * @param filename Path to STL file
 * @param seed Hash seed (default: 0)
 * @return 64-bit hash value, or 0 if file cannot be opened
 */
uint64_t xxhash64_stl_file(const char* filename, uint64_t seed = 0);

/**
 * @brief Compute cache key for SDF generation parameters
 *
 * Hash includes all parameters that affect SDF output:
 * - STL vertex data
 * - Target dimensions (nx, ny, nz)
 * - Padding cells
 *
 * @param stl_path Path to STL file
 * @param target_nx Target SDF dimension X
 * @param target_ny Target SDF dimension Y
 * @param target_nz Target SDF dimension Z
 * @param padding Padding cells
 * @return 64-bit cache key
 */
uint64_t compute_sdf_cache_key(const std::string& stl_path, uint32_t target_nx, uint32_t target_ny, uint32_t target_nz, int32_t padding);

/**
 * @brief Format hash as 8-character hex string
 *
 * Takes the lower 32 bits of a 64-bit hash and formats it as
 * an 8-character hexadecimal string for use in filenames.
 *
 * @param hash 64-bit hash value
 * @return 8-character hex string (e.g., "a1b2c3d4")
 */
std::string format_hash(uint64_t hash);
