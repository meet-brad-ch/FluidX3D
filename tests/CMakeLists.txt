# Test tools and tests. The per-example baseline tests are registered by add_fluidx3d_example().

# Compares a baseline output with its expectation (used by every baseline_<example> test)
add_executable(fluidx3d_baseline_compare baseline/baseline_compare.cpp)
target_link_libraries(fluidx3d_baseline_compare PRIVATE fluidx3d::build_options fluidx3d::warnings)

if(FLUIDX3D_BUILD_ORIGINALS)
    add_subdirectory(originals)
endif()

# --- Unit tests (GoogleTest, CPU only: ctest -L unit) ---------------------------------

include(FetchContent)
FetchContent_Declare(googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.15.2
    GIT_SHALLOW    TRUE
    SYSTEM
)
block()
    set(gtest_force_shared_crt ON) # same MSVC runtime library as the tests (/MD)
    set(INSTALL_GTEST OFF)
    set(BUILD_GMOCK OFF)
    FetchContent_MakeAvailable(googletest)
endblock()
include(GoogleTest)

# fluidx3d_add_test(<name> SOURCES <test sources...> [LINK <targets...>])
# One executable; each TEST() becomes its own CTest test with the label "unit".
function(fluidx3d_add_test name)
    cmake_parse_arguments(PARSE_ARGV 1 ARG "" "" "SOURCES;LINK")
    add_executable(${name} ${ARG_SOURCES})
    target_link_libraries(${name} PRIVATE
        ${ARG_LINK}
        fluidx3d::build_options
        fluidx3d::warnings
        GTest::gtest_main
    )
    gtest_discover_tests(${name} PROPERTIES LABELS unit)
endfunction()

# The core's cell tests (shapes.cpp), which Shape calls; examples compile them with the rest of the core. Core code:
# no first-party warnings.
add_library(fluidx3d_test_core_shapes OBJECT ${PROJECT_SOURCE_DIR}/src/shapes.cpp)
target_link_libraries(fluidx3d_test_core_shapes PRIVATE fluidx3d::setup)

fluidx3d_add_test(fluidx3d_setup_tests
    SOURCES
        unit/quantity_test.cpp
        unit/unit_scale_test.cpp
        unit/temperature_scale_test.cpp
        unit/lattice_test.cpp
        unit/geometry_scaler_test.cpp
        unit/domain_plan_test.cpp
        unit/domain_test.cpp
        unit/model_placement_test.cpp
        unit/camera_view_test.cpp
        unit/shape_test.cpp
    LINK
        fluidx3d::setup
        fluidx3d_test_core_shapes
)
# test geometry files are written here; Model finds them relative to resources/
target_compile_definitions(fluidx3d_setup_tests PRIVATE FLUIDX3D_TEST_DIR="${CMAKE_CURRENT_BINARY_DIR}")
