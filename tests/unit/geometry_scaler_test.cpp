#include <gtest/gtest.h> // before utilities.hpp, which includes <Windows.h> without NOMINMAX

#include "setup/domain/geometry_scaler.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

namespace {

constexpr LatticeMemory d3q19_fp16 { 55u };

// Binary STL (the only format read_stl() reads) of an axis-aligned box from the origin to (sx, sy, sz) m.
class BoxStl {
public:
    BoxStl(const std::string& name, float sx, float sy, float sz) : path_(std::filesystem::temp_directory_path() / name) {
        const float corners[8][3] = { {0, 0, 0}, {sx, 0, 0}, {sx, sy, 0}, {0, sy, 0}, {0, 0, sz}, {sx, 0, sz}, {sx, sy, sz}, {0, sy, sz} };
        const int triangles[12][3] = { {0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
                                       {2, 3, 7}, {2, 7, 6}, {1, 2, 6}, {1, 6, 5}, {0, 4, 7}, {0, 7, 3} };
        std::ofstream file(path_, std::ios::binary);
        const char header[80] = {};
        file.write(header, sizeof(header));
        const std::uint32_t count = 12u;
        file.write(reinterpret_cast<const char*>(&count), sizeof(count));
        for(const auto& triangle : triangles) {
            const float normal[3] = { 0.0f, 0.0f, 0.0f };
            file.write(reinterpret_cast<const char*>(normal), sizeof(normal));
            for(const int corner : triangle) file.write(reinterpret_cast<const char*>(corners[corner]), 3 * sizeof(float));
            const std::uint16_t attributes = 0u;
            file.write(reinterpret_cast<const char*>(&attributes), sizeof(attributes));
        }
    }
    ~BoxStl() { std::filesystem::remove(path_); }
    std::string path() const { return path_.string(); }

private:
    std::filesystem::path path_;
};

TEST(GeometryScaler, ReadsTheGeometrySizeInMetres) {
    const BoxStl box("fluidx3d_test_box_size.stl", 4.0f, 2.0f, 1.0f);
    const GeometryScaler scaler(box.path(), 0.1f, 20000u, d3q19_fp16);
    EXPECT_NEAR(scaler.get_stl_size_meters().x, 4.0f, 1e-5f);
    EXPECT_NEAR(scaler.get_stl_size_meters().y, 2.0f, 1e-5f);
    EXPECT_NEAR(scaler.get_stl_size_meters().z, 1.0f, 1e-5f);
    EXPECT_NEAR(scaler.get_reference_size_meters(), 2.0f, 1e-5f); // reference axis Y
}

TEST(GeometryScaler, CellSizeModeCountsCellsFromTheCellSize) {
    const BoxStl box("fluidx3d_test_box_cells.stl", 4.0f, 2.0f, 1.0f);
    const GeometryScaler scaler(box.path(), 0.1f, 20000u, d3q19_fp16);
    const uint3 cells = scaler.get_stl_size_cells();
    EXPECT_EQ(cells.x, 40u);
    EXPECT_EQ(cells.y, 20u);
    EXPECT_EQ(cells.z, 10u);
    EXPECT_FLOAT_EQ(scaler.get_reference_size_cells(), 20.0f);
    EXPECT_FLOAT_EQ(scaler.get_scale_factor(), 0.1f);
}

TEST(GeometryScaler, ClearancesAddCellsAroundTheGeometry) {
    const BoxStl box("fluidx3d_test_box_clearance.stl", 4.0f, 2.0f, 1.0f);
    const GeometryScaler scaler(box.path(), 0.1f, 20000u, d3q19_fp16);
    const GeometryScaler::Clearances clearances { 0.5f, 1.0f, 0.3f }; // bottom, top, sides in m
    const uint3 domain = scaler.calculate_domain_size(clearances);
    EXPECT_EQ(domain.x, 40u + 2u * 3u);
    EXPECT_EQ(domain.y, 20u + 2u * 3u);
    EXPECT_EQ(domain.z, 10u + 5u + 10u);
    const float3 center = scaler.calculate_center(domain, clearances); // centered in x/y, resting on the bottom clearance
    EXPECT_FLOAT_EQ(center.x, 23.0f);
    EXPECT_FLOAT_EQ(center.y, 13.0f);
    EXPECT_FLOAT_EQ(center.z, 5.0f + 5.0f);
}

TEST(GeometryScaler, VramModeFitsGeometryAndClearancesIntoTheBudget) {
    const BoxStl box("fluidx3d_test_box_vram.stl", 4.0f, 2.0f, 1.0f);
    const GeometryScaler::Clearances clearances { 0.5f, 1.0f, 0.3f };
    const std::uint32_t budget_mb = 100u;
    const GeometryScaler scaler(box.path(), budget_mb, clearances, d3q19_fp16);
    const uint3 domain = scaler.calculate_domain_size(clearances);
    const std::uint64_t used_mb = required_memory_mb({ domain.x, domain.y, domain.z }, d3q19_fp16);
    EXPECT_LE(used_mb, budget_mb + 2u); // whole cells: rounding the clearances can add a little
    EXPECT_GE(used_mb, budget_mb * 9u / 10u);
    EXPECT_NEAR(scaler.get_scale_factor() * (float)scaler.get_stl_size_cells().y, 2.0f, scaler.get_scale_factor());
}

} // namespace
