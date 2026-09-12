cmake_minimum_required(VERSION 3.20)

project(SDFCache VERSION 1.0 LANGUAGES CXX)

# Create static library for SDF caching
add_library(sdf_cache STATIC
    sdf_cache.cpp
    hash_utils.cpp
)
add_library(fluidx3d::sdf_cache ALIAS sdf_cache)

# Include directories
target_include_directories(sdf_cache PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# C++17 required for std::filesystem
target_compile_features(sdf_cache PUBLIC cxx_std_17)
set_target_properties(sdf_cache PROPERTIES CXX_EXTENSIONS OFF)

# Stricter warnings for sdf_cache code (warnings as errors)
if(MSVC)
    target_compile_options(sdf_cache PRIVATE /W4 /WX)
else()
    target_compile_options(sdf_cache PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

# Link against SDFGen unified library (includes CPU and GPU implementations)
# sdfgen_common provides the unified interface and links to both sdfgen_cpu and sdfgen_gpu
target_link_libraries(sdf_cache PUBLIC sdfgen_common)

# Link filesystem library if needed (GCC < 9)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
    target_link_libraries(sdf_cache PUBLIC stdc++fs)
endif()
