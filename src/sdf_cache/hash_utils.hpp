#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

/**
 * @file hash_utils.hpp
 * @brief Hashing for the keys of cached SDF files
 *
 * The cache names an SDF file after everything that shapes its content, so that a changed STL, grid, padding, mesh
 * repair setting or SDFGen version produces a new file instead of a stale hit. The key is an xxHash64 of all of them.
 */

/**
 * @brief xxHash64, Yann Collet's fast 64-bit hash
 *
 * A self-contained implementation of the xxHash64 algorithm, used for the cache keys. Hashing can be chained: the
 * hash of one buffer is the seed of the next.
 */
class XXHash64 {
public:
    /**
     * @brief Compute the xxHash64 of a memory buffer
     *
     * @param data Pointer to the data
     * @param length Length of the data in bytes
     * @param seed Hash seed (default 0); pass a previous hash to chain buffers
     * @return 64-bit hash value
     */
    static uint64_t hash(const void* data, size_t length, uint64_t seed = 0);

    /**
     * @brief Compute the xxHash64 of a binary STL file's triangles
     *
     * Hashes everything after the 80-byte header: the triangle count and the triangles, so that a changed
     * model gives a different hash while a changed header comment does not.
     *
     * @param path Path to the STL file
     * @param seed Hash seed (default 0)
     * @return 64-bit hash value, or 0 if the file cannot be opened or is too small to be a binary STL
     */
    static uint64_t of_stl_file(const std::string& path, uint64_t seed = 0);

private:
    static constexpr uint64_t prime1 = 0x9E3779B185EBCA87ULL; ///< xxHash64 prime 1
    static constexpr uint64_t prime2 = 0xC2B2AE3D27D4EB4FULL; ///< xxHash64 prime 2
    static constexpr uint64_t prime3 = 0x165667B19E3779F9ULL; ///< xxHash64 prime 3
    static constexpr uint64_t prime4 = 0x85EBCA77C2B2AE63ULL; ///< xxHash64 prime 4
    static constexpr uint64_t prime5 = 0x27D4EB2F165667C5ULL; ///< xxHash64 prime 5

    /**
     * @brief Rotate left
     *
     * @param x Value to rotate
     * @param r Number of bits, 1 to 63
     * @return x rotated left by r bits
     */
    static uint64_t rotl(uint64_t x, int r) { return (x << r) | (x >> (64 - r)); }

    /**
     * @brief One xxHash64 round: mix an 8-byte input into an accumulator
     *
     * @param acc Accumulator
     * @param input 8 bytes of input, as a number
     * @return The updated accumulator
     */
    static uint64_t round(uint64_t acc, uint64_t input) { return rotl(acc + input * prime2, 31) * prime1; }

    /**
     * @brief Read 8 bytes as a little-endian number (unaligned)
     *
     * @param p Pointer to the bytes
     * @return The number
     */
    static uint64_t read64(const uint8_t* p);

    /**
     * @brief Read 4 bytes as a little-endian number (unaligned)
     *
     * @param p Pointer to the bytes
     * @return The number
     */
    static uint32_t read32(const uint8_t* p);
};

/**
 * @brief The key of a cached SDF
 *
 * The key hashes every parameter that affects the SDF's content:
 * - the STL's triangles (XXHash64::of_stl_file())
 * - the target grid (nx, ny, nz)
 * - the padding cells
 * - the mesh repair flag (repairing the mesh changes the field)
 * - the SDFGen version (SDFGEN_VERSION_TAG, set by CMake, is the seed): SDFs made by another generator version get
 *   another key
 */
class SDFCacheKey {
public:
    /**
     * @brief Compute the cache key of an SDF
     *
     * @param stl_path Path to the STL file
     * @param nx Target SDF dimension X, in cells across the model (without padding)
     * @param ny Target SDF dimension Y
     * @param nz Target SDF dimension Z
     * @param padding Padding cells added on each side of the model
     * @param fix_mesh Whether the mesh is repaired before the SDF is generated
     * @return 64-bit cache key
     */
    static uint64_t compute(const std::string& stl_path, uint32_t nx, uint32_t ny, uint32_t nz, int32_t padding, bool fix_mesh);

    /**
     * @brief Format a key for a file name
     *
     * Takes the lower 32 bits of the key and formats them as 8 hexadecimal digits.
     *
     * @param key 64-bit cache key
     * @return 8-character hex string (e.g. "a1b2c3d4")
     */
    static std::string format(uint64_t key);
};
