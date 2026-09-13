#include <gtest/gtest.h>

#include "sdf_cache.hpp"
#include "hash_utils.hpp"
#include "box_stl.hpp"

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;

// The reference values of the xxHash64 specification (seed 0); the last one is long enough for the 32-byte stripes.
TEST(XXHash64, MatchesTheReferenceVectors) {
    EXPECT_EQ(XXHash64::hash("", 0, 0), 0xEF46DB3751D8E999ull);
    EXPECT_EQ(XXHash64::hash("a", 1, 0), 0xD24EC4F1A98C6E5Bull);
    EXPECT_EQ(XXHash64::hash("abc", 3, 0), 0x44BC2CF5AD770999ull);
    const char* text = "Nobody inspects the spammish repetition";
    EXPECT_EQ(XXHash64::hash(text, std::strlen(text), 0), 0xFBCEA83C8A378BF1ull);
    EXPECT_NE(XXHash64::hash("abc", 3, 1), XXHash64::hash("abc", 3, 0)); // the seed matters
}

// The STL hash covers the triangles, not the 80-byte header a program writes its name into.
TEST(XXHash64, StlHashIgnoresTheHeader) {
    const BoxStl box("fluidx3d_hash_box.stl", 2.0f, 1.0f, 0.5f);
    const fs::path other = fs::temp_directory_path() / "fluidx3d_hash_box_other_header.stl";
    {
        std::ifstream in(box.path(), std::ios::binary);
        std::vector<char> bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        for(size_t i = 0u; i < 80u; i++) bytes[i] = 'x';
        std::ofstream(other, std::ios::binary).write(bytes.data(), (std::streamsize)bytes.size());
    }
    EXPECT_EQ(XXHash64::of_stl_file(box.path()), XXHash64::of_stl_file(other.string()));
    EXPECT_NE(XXHash64::of_stl_file(box.path()), 0u);
    EXPECT_EQ(XXHash64::of_stl_file("fluidx3d_no_such_file.stl"), 0u);
    fs::remove(other);
}

// Every parameter that shapes the SDF is part of its key.
TEST(SDFCacheKey, ChangesWithEveryParameter) {
    const BoxStl box("fluidx3d_key_box.stl", 2.0f, 1.0f, 0.5f);
    const BoxStl other("fluidx3d_key_other_box.stl", 2.0f, 1.0f, 0.6f);
    const uint64_t key = SDFCacheKey::compute(box.path(), 8u, 4u, 2u, 1, false);
    EXPECT_EQ(SDFCacheKey::compute(box.path(), 8u, 4u, 2u, 1, false), key);
    EXPECT_NE(SDFCacheKey::compute(box.path(), 9u, 4u, 2u, 1, false), key);
    EXPECT_NE(SDFCacheKey::compute(box.path(), 8u, 5u, 2u, 1, false), key);
    EXPECT_NE(SDFCacheKey::compute(box.path(), 8u, 4u, 3u, 1, false), key);
    EXPECT_NE(SDFCacheKey::compute(box.path(), 8u, 4u, 2u, 2, false), key);
    EXPECT_NE(SDFCacheKey::compute(box.path(), 8u, 4u, 2u, 1, true), key);
    EXPECT_NE(SDFCacheKey::compute(other.path(), 8u, 4u, 2u, 1, false), key);
}

TEST(SDFCacheKey, FormatIsTheLower32BitsAs8HexDigits) {
    EXPECT_EQ(SDFCacheKey::format(0x123456789ABCDEF0ull), "9abcdef0");
    EXPECT_EQ(SDFCacheKey::format(0u), "00000000");
    EXPECT_EQ(SDFCacheKey::format(0xFFFFFFFFull), "ffffffff");
}

// A cache directory of its own in the temp directory, emptied before and after each test.
class SDFCacheTest : public ::testing::Test {
protected:
    fs::path cache_dir = fs::temp_directory_path() / "fluidx3d_sdf_cache_test";

    void SetUp() override { fs::remove_all(cache_dir); }
    void TearDown() override { fs::remove_all(cache_dir); }

    SDFCacheConfig config(bool fix_mesh = false) const {
        SDFCacheConfig c;
        c.cache_directory = cache_dir.string() + "/";
        c.fix_mesh = fix_mesh;
        return c;
    }

    struct SdfFile { // the core's SDF format: the grid, its bounds, one float per node, written z-fastest
        std::int32_t n[3];
        float bounds[6];
        std::vector<float> values;
    };
    static SdfFile read(const std::string& path) {
        SdfFile sdf{};
        std::ifstream file(path, std::ios::binary);
        file.read(reinterpret_cast<char*>(sdf.n), sizeof(sdf.n));
        file.read(reinterpret_cast<char*>(sdf.bounds), sizeof(sdf.bounds));
        sdf.values.resize((size_t)sdf.n[0] * sdf.n[1] * sdf.n[2]);
        file.read(reinterpret_cast<char*>(sdf.values.data()), (std::streamsize)(sdf.values.size() * sizeof(float)));
        return sdf;
    }
};

// A 2 x 1 x 0.5 box on 8 x 4 x 2 cells: cells of 0.25, one padding cell on each side, the box centered. SDFGen samples
// the distance at the grid's nodes, origin + i*cell, so the box's faces lie on nodes (distance 0) and the field is
// negative at exactly the 7 x 3 x 1 nodes strictly inside; the node at the box's center is 0.25 from its nearest face.
TEST_F(SDFCacheTest, GeneratesTheSdfOfABoxAndCachesIt) {
    const BoxStl box("fluidx3d_cache_box.stl", 2.0f, 1.0f, 0.5f);
    SDFCacheManager cache(config());
    const std::string path = cache.get_or_generate(box.path(), 8u, 4u, 2u, 1);
    ASSERT_FALSE(path.empty());
    ASSERT_TRUE(fs::is_regular_file(path));

    const std::string name = fs::path(path).filename().string();
    const std::string prefix = "fluidx3d_cache_box_sdf_10x6x4_";
    EXPECT_EQ(name.substr(0, prefix.size()), prefix);
    EXPECT_EQ(name.size(), prefix.size() + 8u + 4u) << name; // 8 hex digits and ".sdf"

    const SdfFile sdf = read(path);
    EXPECT_EQ(sdf.n[0], 10);
    EXPECT_EQ(sdf.n[1], 6);
    EXPECT_EQ(sdf.n[2], 4);
    const float expected_bounds[6] = { -0.25f, -0.25f, -0.25f, 2.25f, 1.25f, 0.75f };
    for(int i = 0; i < 6; i++) EXPECT_NEAR(sdf.bounds[i], expected_bounds[i], 1e-5f) << "bound " << i;
    size_t inside_nodes = 0u;
    for(const float value : sdf.values) inside_nodes += value < 0.0f ? 1u : 0u;
    EXPECT_EQ(inside_nodes, 21u);
    EXPECT_GT(sdf.values.front(), 0.0f); // the corner node, in the padding
    // the node (5, 3, 2) at (1, 0.5, 0.25), 0.25 from the nearest face; the file is written z-fastest: (i*nj + j)*nk + k.
    // SDFGen computes exact distances only in the band next to the surface and sweeps the rest, so a node one cell in
    // is a few percent off (-0.239 here): the sign is what the voxelizer uses
    const size_t center = (5u * 6u + 3u) * 4u + 2u;
    EXPECT_NEAR(sdf.values[center], -0.25f, 0.02f);

    // the second request is a hit: the same file, not written again
    const auto written = fs::last_write_time(path);
    EXPECT_EQ(cache.get_or_generate(box.path(), 8u, 4u, 2u, 1), path);
    EXPECT_EQ(fs::last_write_time(path), written);

    // the mesh repair setting is part of the key: another file
    const std::string repaired = SDFCacheManager(config(true)).get_or_generate(box.path(), 8u, 4u, 2u, 1);
    ASSERT_FALSE(repaired.empty());
    EXPECT_NE(repaired, path);
}

TEST_F(SDFCacheTest, RejectsAnAsciiStlAndAnEmptyGrid) {
    const fs::path ascii = fs::temp_directory_path() / "fluidx3d_cache_ascii.stl";
    std::ofstream(ascii) << "solid box\n  facet normal 0 0 1\n    outer loop\n      vertex 0 0 0\n      vertex 1 0 0\n      vertex 0 1 0\n    endloop\n  endfacet\nendsolid box\n";
    SDFCacheManager cache(config());
    EXPECT_EQ(cache.get_or_generate(ascii.string(), 8u, 4u, 2u, 1), "");
    fs::remove(ascii);

    const BoxStl box("fluidx3d_cache_box2.stl", 2.0f, 1.0f, 0.5f);
    EXPECT_EQ(cache.get_or_generate(box.path(), 0u, 4u, 2u, 1), "");
    EXPECT_TRUE(fs::is_empty(cache_dir));
}

} // namespace
