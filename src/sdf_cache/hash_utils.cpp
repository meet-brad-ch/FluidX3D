#include "hash_utils.hpp"
#include <cstring>
#include <fstream>
#include <sstream>
#include <iomanip>

// xxHash64 constants
namespace {
    constexpr uint64_t PRIME64_1 = 0x9E3779B185EBCA87ULL;
    constexpr uint64_t PRIME64_2 = 0xC2B2AE3D27D4EB4FULL;
    constexpr uint64_t PRIME64_3 = 0x165667B19E3779F9ULL;
    constexpr uint64_t PRIME64_4 = 0x85EBCA77C2B2AE63ULL;
    constexpr uint64_t PRIME64_5 = 0x27D4EB2F165667C5ULL;

    inline uint64_t rotl64(uint64_t x, int r) {
        return (x << r) | (x >> (64 - r));
    }

    inline uint64_t read64(const void* ptr) {
        uint64_t val;
        std::memcpy(&val, ptr, sizeof(val));
        return val;
    }

    inline uint32_t read32(const void* ptr) {
        uint32_t val;
        std::memcpy(&val, ptr, sizeof(val));
        return val;
    }
}

// xxHash64 implementation
uint64_t xxhash64(const void* data, size_t length, uint64_t seed) {
    const uint8_t* p = static_cast<const uint8_t*>(data);
    const uint8_t* const end = p + length;
    uint64_t hash;

    if (length >= 32) {
        const uint8_t* const limit = end - 32;
        uint64_t v1 = seed + PRIME64_1 + PRIME64_2;
        uint64_t v2 = seed + PRIME64_2;
        uint64_t v3 = seed + 0;
        uint64_t v4 = seed - PRIME64_1;

        do {
            v1 += read64(p) * PRIME64_2; v1 = rotl64(v1, 31); v1 *= PRIME64_1; p += 8;
            v2 += read64(p) * PRIME64_2; v2 = rotl64(v2, 31); v2 *= PRIME64_1; p += 8;
            v3 += read64(p) * PRIME64_2; v3 = rotl64(v3, 31); v3 *= PRIME64_1; p += 8;
            v4 += read64(p) * PRIME64_2; v4 = rotl64(v4, 31); v4 *= PRIME64_1; p += 8;
        } while (p <= limit);

        hash = rotl64(v1, 1) + rotl64(v2, 7) + rotl64(v3, 12) + rotl64(v4, 18);

        v1 *= PRIME64_2; v1 = rotl64(v1, 31); v1 *= PRIME64_1; hash ^= v1; hash = hash * PRIME64_1 + PRIME64_4;
        v2 *= PRIME64_2; v2 = rotl64(v2, 31); v2 *= PRIME64_1; hash ^= v2; hash = hash * PRIME64_1 + PRIME64_4;
        v3 *= PRIME64_2; v3 = rotl64(v3, 31); v3 *= PRIME64_1; hash ^= v3; hash = hash * PRIME64_1 + PRIME64_4;
        v4 *= PRIME64_2; v4 = rotl64(v4, 31); v4 *= PRIME64_1; hash ^= v4; hash = hash * PRIME64_1 + PRIME64_4;
    } else {
        hash = seed + PRIME64_5;
    }

    hash += static_cast<uint64_t>(length);

    while (p + 8 <= end) {
        uint64_t k1 = read64(p);
        k1 *= PRIME64_2; k1 = rotl64(k1, 31); k1 *= PRIME64_1;
        hash ^= k1;
        hash = rotl64(hash, 27) * PRIME64_1 + PRIME64_4;
        p += 8;
    }

    if (p + 4 <= end) {
        hash ^= static_cast<uint64_t>(read32(p)) * PRIME64_1;
        hash = rotl64(hash, 23) * PRIME64_2 + PRIME64_3;
        p += 4;
    }

    while (p < end) {
        hash ^= static_cast<uint64_t>(*p) * PRIME64_5;
        hash = rotl64(hash, 11) * PRIME64_1;
        p++;
    }

    hash ^= hash >> 33;
    hash *= PRIME64_2;
    hash ^= hash >> 29;
    hash *= PRIME64_3;
    hash ^= hash >> 32;

    return hash;
}

// Hash STL file (skip 80-byte header, hash vertex data)
uint64_t xxhash64_stl_file(const char* filename, uint64_t seed) {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        return 0;
    }

    // Get file size
    file.seekg(0, std::ios::end);
    size_t file_size = file.tellg();
    file.seekg(0, std::ios::beg);

    // Binary STL format: 80-byte header + 4-byte triangle count + triangle data
    if (file_size < 84) {
        return 0;  // File too small to be valid binary STL
    }

    // Skip 80-byte header
    file.seekg(80, std::ios::beg);

    // Hash the rest of the file (triangle count + all vertex data)
    const size_t chunk_size = 8192;
    uint8_t buffer[chunk_size];
    uint64_t hash = seed;

    while (file.good()) {
        file.read(reinterpret_cast<char*>(buffer), chunk_size);
        size_t bytes_read = file.gcount();
        if (bytes_read > 0) {
            hash = xxhash64(buffer, bytes_read, hash);
        }
    }

    return hash;
}

// Compute SDF cache key from all parameters affecting SDF output
uint64_t compute_sdf_cache_key(const std::string& stl_path, uint32_t target_nx, uint32_t target_ny, uint32_t target_nz, int32_t padding) {
    // Start with STL file hash
    uint64_t hash = xxhash64_stl_file(stl_path.c_str(), 0);

    // Mix in target dimensions (these determine the SDF resolution)
    hash = xxhash64(&target_nx, sizeof(target_nx), hash);
    hash = xxhash64(&target_ny, sizeof(target_ny), hash);
    hash = xxhash64(&target_nz, sizeof(target_nz), hash);

    // Mix in padding
    hash = xxhash64(&padding, sizeof(padding), hash);

    return hash;
}

// Format hash as 8-character hex string for filenames
std::string format_hash(uint64_t hash) {
    std::ostringstream oss;
    oss << std::hex << std::setw(8) << std::setfill('0') << (hash & 0xFFFFFFFF);
    return oss.str();
}
