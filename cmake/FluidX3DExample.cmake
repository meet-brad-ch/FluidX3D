# add_fluidx3d_example(NAME <name>)
#   Builds examples/<name>/main.cpp with examples/<name>/defines.hpp into bin/<name>.
#   With FLUIDX3D_BUILD_TESTS also its headless baseline variant bin/<name>_baseline and the CTest test baseline_<name>.

# Internal: executable TARGET from the setup SOURCES (ARGN) and the core, configured by DEFINES_DIR/defines.hpp.
# The setup sources form the OBJECT library <TARGET>_setup, which gets first-party warnings (not with THIRD_PARTY);
# the core sources do not.
function(_fluidx3d_add_executable TARGET DEFINES_DIR)
    cmake_parse_arguments(PARSE_ARGV 2 ARG "THIRD_PARTY" "" "")
    add_library(${TARGET}_setup OBJECT ${ARG_UNPARSED_ARGUMENTS})
    target_include_directories(${TARGET}_setup PUBLIC ${DEFINES_DIR})
    target_link_libraries(${TARGET}_setup PUBLIC fluidx3d::core)
    if(NOT ARG_THIRD_PARTY)
        target_link_libraries(${TARGET}_setup PRIVATE fluidx3d::warnings)
    endif()

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

# _fluidx3d_add_baseline(<target> SOURCE_DIR <dir> BINARY_DIR <dir> TEST <test> EXPECTED <file> LABELS <labels...>
#                        [THIRD_PARTY])
#   Internal: headless baseline variant <target> of the example in SOURCE_DIR (its main.cpp and defines.hpp), generated
#   in BINARY_DIR and registered as CTest test <test>, which compares its output with EXPECTED.
#   It renders with GRAPHICS instead of INTERACTIVE_GRAPHICS (so the example's video branch is compiled too),
#   prints the setup state at the first lbm.run() (tests/baseline/baseline_dump.cpp) and exits.
#   THIRD_PARTY: the example is not ours (the original examples), so it gets no first-party warnings.
function(_fluidx3d_add_baseline TARGET)
    cmake_parse_arguments(PARSE_ARGV 1 ARG "THIRD_PARTY" "SOURCE_DIR;BINARY_DIR;TEST;EXPECTED" "LABELS")
    set(third_party)
    if(ARG_THIRD_PARTY)
        set(third_party THIRD_PARTY)
    endif()
    file(CONFIGURE OUTPUT ${ARG_BINARY_DIR}/defines.hpp CONTENT
"#pragma once
// Baseline build of the example in ${ARG_SOURCE_DIR}: its own settings, rendered headless
#include \"${ARG_SOURCE_DIR}/defines.hpp\"
#undef INTERACTIVE_GRAPHICS
#undef INTERACTIVE_GRAPHICS_ASCII
")
    file(CONFIGURE OUTPUT ${ARG_BINARY_DIR}/main.cpp CONTENT
"#include \"defines.hpp\"
#include \"${ARG_SOURCE_DIR}/main.cpp\"
")
    _fluidx3d_add_executable(${TARGET} ${ARG_BINARY_DIR} ${third_party}
        ${ARG_BINARY_DIR}/main.cpp
        ${PROJECT_SOURCE_DIR}/tests/baseline/baseline_dump.cpp
    )
    target_compile_definitions(${TARGET}_setup PUBLIC FLUIDX3D_BASELINE) # PUBLIC: lbm.cpp calls the dump

    get_filename_component(expected_name ${ARG_EXPECTED} NAME_WE)
    add_test(NAME ${ARG_TEST}
        COMMAND ${CMAKE_COMMAND}
            -DEXE=$<TARGET_FILE:${TARGET}>
            -DWORKDIR=${PROJECT_SOURCE_DIR}/bin
            -DEXPECTED=${ARG_EXPECTED}
            -DACTUAL=${ARG_BINARY_DIR}/${expected_name}.actual.txt
            -DCOMPARE=$<TARGET_FILE:fluidx3d_baseline_compare>
            -P ${PROJECT_SOURCE_DIR}/tests/baseline/run_baseline.cmake
    )
    set_tests_properties(${ARG_TEST} PROPERTIES
        LABELS "${ARG_LABELS}"
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
        _fluidx3d_add_baseline(${EXAMPLE_NAME}_baseline
            SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}
            BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/baseline
            TEST baseline_${EXAMPLE_NAME}
            EXPECTED ${PROJECT_SOURCE_DIR}/tests/baselines/${EXAMPLE_NAME}.txt
            LABELS baseline gpu
        )
    endif()
endfunction()
