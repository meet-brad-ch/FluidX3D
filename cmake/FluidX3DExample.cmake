# add_fluidx3d_example(NAME <name> [EXTENSIONS <macros...>] [SETTINGS "<macro> <value>"...])
#   Builds examples/<name>/main.cpp into bin/<name>, with the core configured by a defines.hpp generated from the
#   EXTENSIONS and SETTINGS (fluidx3d_configure_defines() below).

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
# The setup sources form the OBJECT library <TARGET>_setup, which gets first-party warnings; the core sources do not.
function(_fluidx3d_add_executable TARGET DEFINES_DIR)
    add_library(${TARGET}_setup OBJECT ${ARGN})
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

function(add_fluidx3d_example)
    cmake_parse_arguments(PARSE_ARGV 0 EXAMPLE "" "NAME" "EXTENSIONS;SETTINGS")
    if(NOT EXAMPLE_NAME)
        message(FATAL_ERROR "add_fluidx3d_example: NAME argument is required")
    endif()

    set(defines_dir ${CMAKE_CURRENT_BINARY_DIR}/defines)
    fluidx3d_configure_defines(${defines_dir} EXTENSIONS ${EXAMPLE_EXTENSIONS} SETTINGS ${EXAMPLE_SETTINGS})
    _fluidx3d_add_executable(${EXAMPLE_NAME} ${defines_dir} ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)
endfunction()
