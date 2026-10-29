# The tests: physics tests (GPU), the header check (built, not run) and the unit tests (CPU).

add_subdirectory(physics) # simulations against analytic solutions: ctest -L physics
add_subdirectory(headers) # every Setup API header compiles on its own (built, not run)

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
        unit/lattice_test.cpp
        unit/geometry_scaler_test.cpp
        unit/domain_plan_test.cpp
        unit/mesh_loader_test.cpp
        unit/domain_test.cpp
        unit/model_placement_test.cpp
        unit/camera_view_test.cpp
        unit/shape_test.cpp
        unit/face_layer_test.cpp
        unit/step_schedule_test.cpp
        unit/time_step_rule_test.cpp
        unit/scalar_transport_rules_test.cpp
        unit/boussinesq_buoyancy_test.cpp
        unit/run_estimate_test.cpp
        unit/units_test.cpp
        unit/lattice_converter_test.cpp
        unit/level_grid_test.cpp
        unit/vertical_profile_test.cpp
        unit/sounding_test.cpp
        unit/moist_column_test.cpp
        unit/wind_target_test.cpp
        unit/sponge_profile_test.cpp
        unit/start_noise_test.cpp
        unit/heated_patch_test.cpp
        unit/boundary_layer_diagnostics_test.cpp
        unit/run_checks_test.cpp
        unit/wind_nudging_test.cpp
        unit/cbl_description_test.cpp
        unit/lattice_parameters_test.cpp
    LINK
        fluidx3d::setup
        fluidx3d_test_core_shapes
)
# the tests' own expected values round as written, like the Setup API's (cmake/configs.cmake)
set_target_properties(fluidx3d_setup_tests PROPERTIES FLUIDX3D_PRECISE_FLOAT ON)
# test geometry files are written here; Model finds them relative to resources/
target_compile_definitions(fluidx3d_setup_tests PRIVATE FLUIDX3D_TEST_DIR="${CMAKE_CURRENT_BINARY_DIR}"
    FLUIDX3D_GOLDEN_DIR="${CMAKE_CURRENT_SOURCE_DIR}/golden") # the frozen captures of the CBL example (golden_capture.hpp)

# The SDF cache: its hash and key, and an SDF generated with SDFGen (GPU, or CPU without one) in a temp cache
fluidx3d_add_test(fluidx3d_sdf_cache_tests
    SOURCES unit/sdf_cache_test.cpp
    LINK fluidx3d::sdf_cache
)
