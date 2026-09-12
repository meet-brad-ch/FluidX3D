#include <gtest/gtest.h> // before utilities.hpp, which includes <Windows.h> without NOMINMAX

#include "setup/domain/geometry_scaler.hpp"
#include "box_stl.hpp"

#include <cmath>
#include <filesystem>
#include <string>

namespace {

constexpr LatticeMemory d3q19_fp16 { 55u };
constexpr GeometryScaler::Clearances no_clearances { 0.0f, 0.0f, 0.0f };
constexpr GeometryScaler::Clearances wide_clearances { 0.0f, 10.0f, 10.0f }; // bottom, top, sides in m

TEST(GeometryScaler, ReadsTheGeometrySizeInMetres) {
    const BoxStl box("fluidx3d_test_box_size.stl", 4.0f, 2.0f, 1.0f);
    const GeometryScaler scaler(box.path(), 0.1f, 20000u, no_clearances, d3q19_fp16);
    EXPECT_NEAR(scaler.get_stl_size_meters().x, 4.0f, 1e-5f);
    EXPECT_NEAR(scaler.get_stl_size_meters().y, 2.0f, 1e-5f);
    EXPECT_NEAR(scaler.get_stl_size_meters().z, 1.0f, 1e-5f);
    EXPECT_NEAR(scaler.get_reference_size_meters(), 2.0f, 1e-5f); // reference axis Y
}

TEST(GeometryScaler, CellSizeModeCountsCellsFromTheCellSize) {
    const BoxStl box("fluidx3d_test_box_cells.stl", 4.0f, 2.0f, 1.0f);
    const GeometryScaler scaler(box.path(), 0.1f, 20000u, no_clearances, d3q19_fp16);
    const uint3 cells = scaler.get_stl_size_cells();
    EXPECT_EQ(cells.x, 40u);
    EXPECT_EQ(cells.y, 20u);
    EXPECT_EQ(cells.z, 10u);
    EXPECT_FLOAT_EQ(scaler.get_reference_size_cells(), 20.0f);
    EXPECT_FLOAT_EQ(scaler.get_scale_factor(), 0.1f);
}

TEST(GeometryScaler, ClearancesAddCellsAroundTheGeometry) {
    const BoxStl box("fluidx3d_test_box_clearance.stl", 4.0f, 2.0f, 1.0f);
    const GeometryScaler scaler(box.path(), 0.1f, 20000u, no_clearances, d3q19_fp16);
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

// Regression for review A8: the cell-size mode checked only the geometry's own cells against the memory limit.
TEST(GeometryScaler, CellSizeModeCountsTheClearancesInTheMemoryLimit) {
    const BoxStl box("fluidx3d_test_box_limit.stl", 4.0f, 2.0f, 1.0f);
    // 0.1 m cells: 40 x 20 x 10 cells alone (0.4 MB); with the clearances 240 x 220 x 110 cells (304 MB)
    EXPECT_NO_THROW(GeometryScaler(box.path(), 0.1f, 100u, no_clearances, d3q19_fp16));
    EXPECT_THROW(GeometryScaler(box.path(), 0.1f, 100u, wide_clearances, d3q19_fp16), SetupError);
}

TEST(GeometryScaler, MemoryLimitErrorNamesTheNumbersAndACellSizeThatFits) {
    const BoxStl box("fluidx3d_test_box_message.stl", 4.0f, 2.0f, 1.0f);
    try {
        GeometryScaler(box.path(), 0.1f, 100u, wide_clearances, d3q19_fp16);
        FAIL() << "expected a SetupError";
    } catch(const SetupError& error) {
        const std::string message = error.what();
        EXPECT_NE(message.find("240 x 220 x 110"), std::string::npos) << message;
        EXPECT_NE(message.find("304 MB"), std::string::npos) << message;
        EXPECT_NE(message.find("100 MB"), std::string::npos) << message;
    }
    // the suggestion: (24 x 22 x 11 m³ / cells in 100 MB)^(1/3) = 0.145 m; a little larger fits
    const float min_cell_size = std::cbrt(24.0f * 22.0f * 11.0f / (100.0f * 1048576.0f / 55.0f));
    EXPECT_NO_THROW(GeometryScaler(box.path(), 1.05f * min_cell_size, 100u, wide_clearances, d3q19_fp16));
}

TEST(GeometryScaler, MissingStlThrows) {
    const std::string missing = (std::filesystem::temp_directory_path() / "fluidx3d_test_missing.stl").string();
    EXPECT_THROW(GeometryScaler(missing, 0.1f, 100u, no_clearances, d3q19_fp16), SetupError);
    EXPECT_THROW(GeometryScaler(missing, 100u, no_clearances, d3q19_fp16), SetupError);
}

} // namespace
