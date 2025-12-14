cmake_minimum_required(VERSION 3.20)

project(FluidX3D VERSION 3.5 LANGUAGES CXX)

# Define absolute path to project resources at compile time
add_compile_definitions(FLUIDX3D_RESOURCE_DIR="${PROJECT_SOURCE_DIR}/resources")

# Fetch external dependencies
include(FetchContent)

# Fetch LodePNG (PNG encoding/decoding library)
# Using version 20200306 (commit e34ac04 from 2020-03-06)
FetchContent_Declare(
    lodepng
    GIT_REPOSITORY https://github.com/lvandeve/lodepng.git
    GIT_TAG        e34ac04553e51a6982ae234d98ce6b76dd57a6a1
    GIT_SHALLOW    FALSE  # Need full history to fetch older commit
)
FetchContent_MakeAvailable(lodepng)

# Fetch OpenCL Headers (Khronos official C headers)
# Using latest release v2024.10.24
FetchContent_Declare(
    opencl_headers
    GIT_REPOSITORY https://github.com/KhronosGroup/OpenCL-Headers.git
    GIT_TAG        v2024.10.24
)
FetchContent_MakeAvailable(opencl_headers)

# Fetch OpenCL C++ bindings (Khronos official C++ wrapper)
# Using latest release v2024.10.24
set(BUILD_DOCS OFF CACHE BOOL "Build documentation" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "Build examples" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "Build tests" FORCE)
FetchContent_Declare(
    opencl_clhpp
    GIT_REPOSITORY https://github.com/KhronosGroup/OpenCL-CLHPP.git
    GIT_TAG        v2024.10.24
)
FetchContent_MakeAvailable(opencl_clhpp)

# Find dependencies - create IMPORTED targets
if(UNIX)
    find_package(Threads REQUIRED)
endif()

# We use fetched OpenCL headers + bundled ICD loader libs
# We use bundled X11/Xrandr on Unix (no find_package needed)

# Set paths for examples to use
set(FLUIDX3D_SRC_DIR ${PROJECT_SOURCE_DIR}/src)
set(FLUIDX3D_OPENCL_HEADERS_DIR ${opencl_headers_SOURCE_DIR})
set(FLUIDX3D_OPENCL_CLHPP_DIR ${opencl_clhpp_SOURCE_DIR}/include)
set(FLUIDX3D_OPENCL_LIB_DIR ${PROJECT_SOURCE_DIR}/third_party/OpenCL/lib)
set(FLUIDX3D_X11_DIR ${PROJECT_SOURCE_DIR}/third_party/X11)
set(FLUIDX3D_LODEPNG_DIR ${lodepng_SOURCE_DIR})

# Include generic example builder module
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(FluidX3DExample)

# No core library - each example compiles core sources directly
# This avoids ODR violations from different defines.hpp configs
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
message(STATUS "===========================================")
