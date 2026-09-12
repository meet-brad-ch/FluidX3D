# Pinned third-party dependencies (FetchContent). SYSTEM: their headers never produce warnings in our code.
include(FetchContent)

# LodePNG (PNG encoding/decoding), commit e34ac04 from 2020-03-06
FetchContent_Declare(lodepng
    GIT_REPOSITORY https://github.com/lvandeve/lodepng.git
    GIT_TAG        e34ac04553e51a6982ae234d98ce6b76dd57a6a1
    GIT_SHALLOW    FALSE  # a plain commit hash needs the full history
    SYSTEM
)

# Khronos OpenCL C headers and C++ bindings (targets OpenCL::Headers, OpenCL::HeadersCpp)
FetchContent_Declare(opencl_headers
    GIT_REPOSITORY https://github.com/KhronosGroup/OpenCL-Headers.git
    GIT_TAG        v2024.10.24
    SYSTEM
)
FetchContent_Declare(opencl_clhpp
    GIT_REPOSITORY https://github.com/KhronosGroup/OpenCL-CLHPP.git
    GIT_TAG        v2024.10.24
    SYSTEM
)

# SDFGen (SDF generation), pinned to a release; the tag is also part of the SDF cache key (see CMakeLists.txt)
set(SDFGEN_TAG v1.0.0)
FetchContent_Declare(sdfgen
    GIT_REPOSITORY https://github.com/meet-brad-ch/SDFGenFast.git
    GIT_TAG        ${SDFGEN_TAG}
    SYSTEM
)

# Skip the dependencies' docs, examples and tests; these variables stay local to the block
block()
    set(BUILD_DOCS OFF)
    set(BUILD_EXAMPLES OFF)
    set(BUILD_TESTING OFF)
    FetchContent_MakeAvailable(opencl_headers opencl_clhpp sdfgen)
endblock()
FetchContent_MakeAvailable(lodepng)

# LodePNG has no CMake project: build it once as a static library
add_library(fluidx3d_lodepng STATIC ${lodepng_SOURCE_DIR}/lodepng.cpp)
add_library(fluidx3d::lodepng ALIAS fluidx3d_lodepng)
target_include_directories(fluidx3d_lodepng SYSTEM PUBLIC ${lodepng_SOURCE_DIR})
