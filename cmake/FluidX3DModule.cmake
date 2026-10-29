# add_fluidx3d_module(NAME <name> SOURCES <files...> BINDINGS <files...> [EXTENSIONS <macros...>]
#                     [SETTINGS "<macro> <value>"...] [STABLE_ABI])
#   Builds the Python extension module <name> (nanobind) into bin/, with the core configured as add_fluidx3d_example()
#   configures an example (a defines.hpp from the EXTENSIONS and SETTINGS). SOURCES hold the application code; they
#   form the OBJECT library <name>_setup with the first-party warnings, as an example's main.cpp does. BINDINGS hold
#   the nanobind code; it is compiled without the first-party warnings, because nanobind's headers do not pass them.
#   The core's src/main.cpp (the console program's main() and main_setup() call) is not part of a module: the module's
#   functions run the simulation in the calling thread.
#   STABLE_ABI builds the module for Python's stable ABI (Python 3.12 or later): one build works with every later
#   Python version. Without it, the module works only with the Python version it was built for.
#   Needs -DFLUIDX3D_PYTHON_MODULES=ON, a Python 3.8 or later interpreter and nanobind (pip install nanobind).

if(FLUIDX3D_PYTHON_MODULES)
    # Development.SABIModule (Python 3.12 or later) is what nanobind's STABLE_ABI builds against
    find_package(Python 3.8 COMPONENTS Interpreter Development.Module OPTIONAL_COMPONENTS Development.SABIModule REQUIRED)
    execute_process(
        COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
        OUTPUT_VARIABLE _fluidx3d_nanobind_dir
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE _fluidx3d_nanobind_result
    )
    if(NOT _fluidx3d_nanobind_result EQUAL 0)
        message(FATAL_ERROR "FLUIDX3D_PYTHON_MODULES: nanobind is not installed for ${Python_EXECUTABLE} (pip install nanobind)")
    endif()
    list(APPEND CMAKE_PREFIX_PATH "${_fluidx3d_nanobind_dir}")
    find_package(nanobind CONFIG REQUIRED)
    message(STATUS "Python modules: ${Python_EXECUTABLE} (${Python_VERSION}), nanobind ${nanobind_VERSION}")
endif()

function(add_fluidx3d_module)
    cmake_parse_arguments(PARSE_ARGV 0 MODULE "STABLE_ABI" "NAME" "SOURCES;BINDINGS;EXTENSIONS;SETTINGS")
    if(NOT FLUIDX3D_PYTHON_MODULES)
        message(FATAL_ERROR "add_fluidx3d_module(${MODULE_NAME}): configure with -DFLUIDX3D_PYTHON_MODULES=ON")
    endif()
    if(NOT MODULE_NAME OR NOT MODULE_SOURCES OR NOT MODULE_BINDINGS)
        message(FATAL_ERROR "add_fluidx3d_module: NAME, SOURCES and BINDINGS are required")
    endif()

    set(defines_dir ${CMAKE_CURRENT_BINARY_DIR}/defines_${MODULE_NAME}) # per target, as add_fluidx3d_example()
    fluidx3d_configure_defines(${defines_dir} EXTENSIONS ${MODULE_EXTENSIONS} SETTINGS ${MODULE_SETTINGS})

    add_library(${MODULE_NAME}_setup OBJECT ${MODULE_SOURCES})
    target_include_directories(${MODULE_NAME}_setup PUBLIC ${defines_dir})
    target_link_libraries(${MODULE_NAME}_setup PUBLIC fluidx3d::core PRIVATE fluidx3d::warnings)

    set(core_sources ${FLUIDX3D_CORE_SOURCES})
    list(REMOVE_ITEM core_sources ${FLUIDX3D_SRC_DIR}/main.cpp)
    set(abi_option "")
    if(MODULE_STABLE_ABI)
        set(abi_option STABLE_ABI)
    endif()
    nanobind_add_module(${MODULE_NAME} NOMINSIZE ${abi_option} ${MODULE_BINDINGS} ${core_sources}) # NOMINSIZE: the core is built for speed, not size
    target_link_libraries(${MODULE_NAME} PRIVATE ${MODULE_NAME}_setup)
    set_target_properties(${MODULE_NAME} PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
        LIBRARY_OUTPUT_DIRECTORY_DEBUG "${PROJECT_SOURCE_DIR}/bin"
        LIBRARY_OUTPUT_DIRECTORY_RELEASE "${PROJECT_SOURCE_DIR}/bin"
        LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO "${PROJECT_SOURCE_DIR}/bin"
        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
        RUNTIME_OUTPUT_DIRECTORY_RELEASE "${PROJECT_SOURCE_DIR}/bin"
    )
endfunction()
