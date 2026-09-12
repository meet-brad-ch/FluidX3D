# add_fluidx3d_example(NAME <name>)
#   Builds examples/<name>/main.cpp with examples/<name>/defines.hpp into bin/<name>.
#   With FLUIDX3D_BUILD_TESTS also its headless baseline variant bin/<name>_baseline and the CTest test baseline_<name>.

# Internal: executable TARGET from the setup SOURCES (ARGN) and the core, configured by DEFINES_DIR/defines.hpp.
# The setup sources form the OBJECT library <TARGET>_setup, which gets first-party warnings; the core sources do not.
function(_fluidx3d_add_executable TARGET DEFINES_DIR)
    add_library(${TARGET}_setup OBJECT ${ARGN} ${FLUIDX3D_SETUP_SOURCES})
    target_include_directories(${TARGET}_setup PUBLIC ${DEFINES_DIR})
    target_link_libraries(${TARGET}_setup PUBLIC fluidx3d::core PRIVATE fluidx3d::warnings)

    add_executable(${TARGET} ${FLUIDX3D_CORE_SOURCES})
    target_link_libraries(${TARGET} PRIVATE ${TARGET}_setup)

    # Executables run from bin/ (the runtime needs it, see runtime_problem_solution.md)
    set_target_properties(${TARGET} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
        RUNTIME_OUTPUT_DIRECTORY_DEBUG "${PROJECT_SOURCE_DIR}/bin"
        RUNTIME_OUTPUT_DIRECTORY_RELEASE "${PROJECT_SOURCE_DIR}/bin"
        RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${PROJECT_SOURCE_DIR}/bin"
        RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${PROJECT_SOURCE_DIR}/bin"
    )
endfunction()

# Internal: headless baseline variant of example NAME, registered as CTest test baseline_<NAME>.
# It renders with GRAPHICS instead of INTERACTIVE_GRAPHICS (so the example's video branch is compiled too),
# prints the setup state at the first lbm.run() (tests/baseline/baseline_dump.cpp) and exits.
function(_fluidx3d_add_baseline NAME)
    set(BASELINE_DIR ${CMAKE_CURRENT_BINARY_DIR}/baseline)
    file(CONFIGURE OUTPUT ${BASELINE_DIR}/defines.hpp CONTENT
"#pragma once
// Baseline build of example '${NAME}': its own settings, rendered headless
#include \"${CMAKE_CURRENT_SOURCE_DIR}/defines.hpp\"
#undef INTERACTIVE_GRAPHICS
#undef INTERACTIVE_GRAPHICS_ASCII
")
    file(CONFIGURE OUTPUT ${BASELINE_DIR}/main.cpp CONTENT
"#include \"defines.hpp\"
#include \"${CMAKE_CURRENT_SOURCE_DIR}/main.cpp\"
")
    _fluidx3d_add_executable(${NAME}_baseline ${BASELINE_DIR}
        ${BASELINE_DIR}/main.cpp
        ${PROJECT_SOURCE_DIR}/tests/baseline/baseline_dump.cpp
    )
    target_compile_definitions(${NAME}_baseline_setup PUBLIC FLUIDX3D_BASELINE) # PUBLIC: lbm.cpp calls the dump

    add_test(NAME baseline_${NAME}
        COMMAND ${CMAKE_COMMAND}
            -DEXE=$<TARGET_FILE:${NAME}_baseline>
            -DWORKDIR=${PROJECT_SOURCE_DIR}/bin
            -DEXPECTED=${PROJECT_SOURCE_DIR}/tests/baselines/${NAME}.txt
            -DACTUAL=${BASELINE_DIR}/${NAME}.actual.txt
            -DCOMPARE=$<TARGET_FILE:fluidx3d_baseline_compare>
            -P ${PROJECT_SOURCE_DIR}/tests/baseline/run_baseline.cmake
    )
    set_tests_properties(baseline_${NAME} PROPERTIES
        LABELS "baseline;gpu"
        SKIP_REGULAR_EXPRESSION "BASELINE_SKIPPED"
        TIMEOUT 900
        RUN_SERIAL TRUE
    )
endfunction()

function(add_fluidx3d_example)
    cmake_parse_arguments(PARSE_ARGV 0 EXAMPLE "" "NAME" "THINGIVERSE_STL")
    if(NOT EXAMPLE_NAME)
        message(FATAL_ERROR "add_fluidx3d_example: NAME argument is required")
    endif()

    _fluidx3d_add_executable(${EXAMPLE_NAME} ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)
    if(FLUIDX3D_BUILD_TESTS)
        _fluidx3d_add_baseline(${EXAMPLE_NAME})
    endif()
endfunction()
