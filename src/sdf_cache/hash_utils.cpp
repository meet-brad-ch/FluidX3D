/**
 * @file hash_utils.cpp
 * @brief xxHash64 and the SDF cache key
 */

#include "hash_utils.hpp"

#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>

uint64_t XXHash64::read64(const uint8_t* p) {
    uint64_t value;
    std::memcpy(&value, p, sizeof(value));
    return value;
}

uint32_t XXHash64::read32(const uint8_t* p) {
    uint32_t value;
    std::memcpy(&value, p, sizeof(value));
    return value;
}

uint64_t XXHash64::hash(const void* data, size_t length, uint64_t seed) {
    const uint8_t* p = static_cast<const uint8_t*>(data);
    const uint8_t* const end = p + length;
    uint64_t h;

    if(length >= 32) { // four accumulators over 32-byte stripes
        const uint8_t* const limit = end - 32;
        uint64_t v1 = seed + prime1 + prime2;
        uint64_t v2 = seed + prime2;
        uint64_t v3 = seed;
        uint64_t v4 = seed - prime1;
        do {
            v1 = round(v1, read64(p)); p += 8;
            v2 = round(v2, read64(p)); p += 8;
            v3 = round(v3, read64(p)); p += 8;
            v4 = round(v4, read64(p)); p += 8;
        } while(p <= limit);

        h = rotl(v1, 1) + rotl(v2, 7) + rotl(v3, 12) + rotl(v4, 18);
        h = (h ^ round(0, v1)) * prime1 + prime4;
        h = (h ^ round(0, v2)) * prime1 + prime4;
        h = (h ^ round(0, v3)) * prime1 + prime4;
        h = (h ^ round(0, v4)) * prime1 + prime4;
    } else {
        h = seed + prime5;
    }

    h += static_cast<uint64_t>(length);

    while(p + 8 <= end) { // the remaining 8-byte words
        h ^= round(0, read64(p));
        h = rotl(h, 27) * prime1 + prime4;
        p += 8;
    }
    if(p + 4 <= end) { // a remaining 4-byte word
        h ^= static_cast<uint64_t>(read32(p)) * prime1;
        h = rotl(h, 23) * prime2 + prime3;
        p += 4;
    }
    while(p < end) { // the remaining bytes
        h ^= static_cast<uint64_t>(*p) * prime5;
        h = rotl(h, 11) * prime1;
        p++;
    }

    // the avalanche
    h ^= h >> 33;
    h *= prime2;
    h ^= h >> 29;
    h *= prime3;
    h ^= h >> 32;
    return h;
}

uint64_t XXHash64::of_stl_file(const std::string& path, uint64_t seed) {
    std::ifstream file(path, std::ios::binary);
    if(!file.is_open()) return 0;

    file.seekg(0, std::ios::end);
    const std::streamoff file_size = file.tellg();
    if(file_size < 84) return 0; // a binary STL has an 80-byte header and a 4-byte triangle count at least

    file.seekg(80, std::ios::beg); // past the header: the triangle count and the triangles
    constexpr size_t chunk_size = 8192;
    uint8_t buffer[chunk_size];
    uint64_t h = seed;
    while(file.good()) {
        file.read(reinterpret_cast<char*>(buffer), chunk_size);
        const size_t bytes_read = static_cast<size_t>(file.gcount());
        if(bytes_read > 0) h = hash(buffer, bytes_read, h);
    }
    return h;
}

#ifndef SDFGEN_VERSION_TAG
#define SDFGEN_VERSION_TAG "unknown" ///< the SDFGen version (CMake sets it from the fetched tag)
#endif

uint64_t SDFCacheKey::compute(const std::string& stl_path, uint32_t nx, uint32_t ny, uint32_t nz, int32_t padding, bool fix_mesh) {
    const uint64_t version_seed = XXHash64::hash(SDFGEN_VERSION_TAG, sizeof(SDFGEN_VERSION_TAG) - 1, 0);
    uint64_t key = XXHash64::of_stl_file(stl_path, version_seed);
    key = XXHash64::hash(&nx, sizeof(nx), key);
    key = XXHash64::hash(&ny, sizeof(ny), key);
    key = XXHash64::hash(&nz, sizeof(nz), key);
    key = XXHash64::hash(&padding, sizeof(padding), key);
    const uint8_t fix_mesh_byte = fix_mesh ? 1 : 0;
    return XXHash64::hash(&fix_mesh_byte, sizeof(fix_mesh_byte), key);
}

std::string SDFCacheKey::format(uint64_t key) {
    std::ostringstream text;
    text << std::hex << std::setw(8) << std::setfill('0') << (key & 0xFFFFFFFFu);
    return text.str();
}
