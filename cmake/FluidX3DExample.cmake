# add_fluidx3d_example(NAME <name> [EXTENSIONS <macros...>] [SETTINGS "<macro> <value>"...])
#   Builds examples/<name>/main.cpp into bin/<name>, with the core configured by a defines.hpp generated from the
#   EXTENSIONS and SETTINGS (fluidx3d_configure_defines() below).
#   With FLUIDX3D_BUILD_TESTS also its headless baseline variant bin/<name>_baseline and the CTest test baseline_<name>.

# The macros a configuration may enable (cmake/defines.hpp.in describes them)
set(FLUIDX3D_VELOCITY_SETS D2Q9 D3Q15 D3Q19 D3Q27)
set(FLUIDX3D_COLLISION_OPERATORS SRT TRT)
set(FLUIDX3D_KNOWN_MACROS
    ${FLUIDX3D_VELOCITY_SETS} ${FLUIDX3D_COLLISION_OPERATORS}
    FP16S FP16C BENCHMARK
    VOLUME_FORCE FORCE_FIELD EQUILIBRIUM_BOUNDARIES MOVING_BOUNDARIES SURFACE TEMPERATURE SUBGRID PARTICLES
    INTERACTIVE_GRAPHICS INTERACTIVE_GRAPHICS_ASCII GRAPHICS UPDATE_FIELDS
)

# fluidx3d_configure_defines(<dir> [EXTENSIONS <macros...>] [SETTINGS "<macro> <value>"...])
#   Writes <dir>/defines.hpp, the core's configuration, from cmake/defines.hpp.in: the velocity set (default D3Q19),
#   the collision operator (default SRT), the precision (FP16S or FP16C) and the extensions (VOLUME_FORCE, SURFACE,
#   INTERACTIVE_GRAPHICS, ...) as macros, plus settings with a value, such as "GRAPHICS_FRAME_WIDTH 3840", which
#   replace the template's defaults.
function(fluidx3d_configure_defines dir)
    cmake_parse_arguments(PARSE_ARGV 1 ARG "" "" "EXTENSIONS;SETTINGS")
    set(macros ${ARG_EXTENSIONS})
    foreach(macro IN LISTS macros)
        if(NOT macro IN_LIST FLUIDX3D_KNOWN_MACROS)
            message(FATAL_ERROR "fluidx3d_configure_defines(${dir}): unknown macro ${macro}; the options are ${FLUIDX3D_KNOWN_MACROS}")
        endif()
    endforeach()
    _fluidx3d_add_default_macro(macros "${FLUIDX3D_COLLISION_OPERATORS}" SRT)
    _fluidx3d_add_default_macro(macros "${FLUIDX3D_VELOCITY_SETS}" D3Q19)
    list(TRANSFORM macros PREPEND "#define ")
    list(TRANSFORM ARG_SETTINGS PREPEND "#define ")
    list(JOIN macros "\n" FLUIDX3D_CONFIGURATION)
    if(ARG_SETTINGS)
        list(JOIN ARG_SETTINGS "\n" setting_lines)
        string(APPEND FLUIDX3D_CONFIGURATION "\n${setting_lines}")
    endif()
    configure_file(${CMAKE_CURRENT_FUNCTION_LIST_DIR}/defines.hpp.in ${dir}/defines.hpp @ONLY)
endfunction()

# Internal: prepends DEFAULT to the caller's list variable LIST_VAR unless it already holds one of the CHOICES.
function(_fluidx3d_add_default_macro list_var choices default)
    foreach(choice IN LISTS choices)
        if(choice IN_LIST ${list_var})
            return()
        endif()
    endforeach()
    set(${list_var} ${default} ${${list_var}} PARENT_SCOPE)
endfunction()

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
#                        (DEFINES_FILE <file> | [EXTENSIONS <macros...>] [SETTINGS ...]) [THIRD_PARTY])
#   Internal: headless baseline variant <target> of the main.cpp in SOURCE_DIR, generated in BINARY_DIR and registered
#   as CTest test <test>, which compares its output with EXPECTED. It renders with GRAPHICS instead of
#   INTERACTIVE_GRAPHICS (so the example's video branch is compiled too), prints the setup state at the first lbm.run()
#   (tests/baseline/baseline_dump.cpp) and exits.
#   The configuration is generated from EXTENSIONS and SETTINGS (an example's), or taken from DEFINES_FILE (the
#   original examples' own defines.hpp).
#   THIRD_PARTY: the example is not ours (the original examples), so it gets no first-party warnings.
function(_fluidx3d_add_baseline TARGET)
    cmake_parse_arguments(PARSE_ARGV 1 ARG "THIRD_PARTY" "SOURCE_DIR;BINARY_DIR;TEST;EXPECTED;DEFINES_FILE" "LABELS;EXTENSIONS;SETTINGS")
    set(third_party)
    if(ARG_THIRD_PARTY)
        set(third_party THIRD_PARTY)
    endif()
    if(ARG_DEFINES_FILE)
        file(CONFIGURE OUTPUT ${ARG_BINARY_DIR}/defines.hpp CONTENT
"#pragma once
// Baseline build of the example in ${ARG_SOURCE_DIR}: its own settings, rendered headless
#include \"${ARG_DEFINES_FILE}\"
#undef INTERACTIVE_GRAPHICS
#undef INTERACTIVE_GRAPHICS_ASCII
")
    else()
        set(macros ${ARG_EXTENSIONS})
        if(INTERACTIVE_GRAPHICS IN_LIST macros OR INTERACTIVE_GRAPHICS_ASCII IN_LIST macros)
            list(REMOVE_ITEM macros INTERACTIVE_GRAPHICS INTERACTIVE_GRAPHICS_ASCII)
            list(APPEND macros GRAPHICS)
            list(REMOVE_DUPLICATES macros)
        endif()
        fluidx3d_configure_defines(${ARG_BINARY_DIR} EXTENSIONS ${macros} SETTINGS ${ARG_SETTINGS})
    endif()
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
    cmake_parse_arguments(PARSE_ARGV 0 EXAMPLE "" "NAME" "EXTENSIONS;SETTINGS")
    if(NOT EXAMPLE_NAME)
        message(FATAL_ERROR "add_fluidx3d_example: NAME argument is required")
    endif()

    set(defines_dir ${CMAKE_CURRENT_BINARY_DIR}/defines)
    fluidx3d_configure_defines(${defines_dir} EXTENSIONS ${EXAMPLE_EXTENSIONS} SETTINGS ${EXAMPLE_SETTINGS})
    _fluidx3d_add_executable(${EXAMPLE_NAME} ${defines_dir} ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)
    if(FLUIDX3D_BUILD_TESTS)
        _fluidx3d_add_baseline(${EXAMPLE_NAME}_baseline
            SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}
            BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/baseline
            TEST baseline_${EXAMPLE_NAME}
            EXPECTED ${PROJECT_SOURCE_DIR}/tests/baselines/${EXAMPLE_NAME}.txt
            LABELS baseline gpu
            EXTENSIONS ${EXAMPLE_EXTENSIONS}
            SETTINGS ${EXAMPLE_SETTINGS}
        )
    endif()
endfunction()
