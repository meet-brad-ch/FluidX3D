cmake_minimum_required(VERSION 3.25)

project(FluidX3D VERSION 3.5 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(configs)               # options, build type, fluidx3d::build_options, fluidx3d::warnings
include(FluidX3DDependencies)  # pinned third-party code: LodePNG, OpenCL headers, SDFGen
include(FluidX3DPlatform)      # fluidx3d::platform: OpenCL ICD loader, system libraries, X11, threads

add_subdirectory(src/sdf_cache)
target_compile_definitions(sdf_cache PRIVATE SDFGEN_VERSION_TAG="${SDFGEN_TAG}") # the SDFGen version is part of the SDF cache key

include(FluidX3DCore)          # fluidx3d::core and the core source lists
include(FluidX3DExample)       # add_fluidx3d_example()

if(FLUIDX3D_BUILD_TESTS)
    enable_testing() # before the examples: each one registers its baseline test
    add_subdirectory(tests)
endif()
add_subdirectory(examples)

# Detect platform
if(WIN32)
    set(PLATFORM_NAME "Windows")
    set(BUNDLED_LIBS "OpenCL")
elseif(APPLE)
    set(PLATFORM_NAME "macOS")
    set(BUNDLED_LIBS "OpenCL, X11, Xrandr")
elseif(UNIX)
    set(PLATFORM_NAME "Linux")
    set(BUNDLED_LIBS "OpenCL, X11, Xrandr")
else()
    set(PLATFORM_NAME "Unknown")
    set(BUNDLED_LIBS "OpenCL")
endif()

# =============================================================================
# STL file download information
# =============================================================================
find_program(PYTHON_EXECUTABLE NAMES python3 python)
if(PYTHON_EXECUTABLE)
    if(WIN32)
        message(STATUS "To download STL files:")
        message(STATUS "  cd resources && python download_all_thingiverse_stl.py")
    else()
        message(STATUS "To download STL files:")
        message(STATUS "  cd resources && python3 download_all_thingiverse_stl.py")
    endif()
else()
    message(STATUS "Python not found - cannot download STL files automatically")
endif()

# Print configuration summary
message(STATUS "===========================================")
message(STATUS "FluidX3D CMake Configuration")
message(STATUS "===========================================")
message(STATUS "Version:         ${PROJECT_VERSION}")
message(STATUS "Platform:        ${PLATFORM_NAME}")
message(STATUS "Compiler:        ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "Architecture:    Unity build (no library)")
message(STATUS "Using bundled:   ${BUNDLED_LIBS}")
message(STATUS "Tests:           ${FLUIDX3D_BUILD_TESTS}")
message(STATUS "Warnings=errors: ${FLUIDX3D_WERROR}")
message(STATUS "===========================================")
