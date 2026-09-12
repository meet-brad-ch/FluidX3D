# The Setup API layers that do not need the LBM: physical quantities and unit scaling (core/),
# lattice sizing (domain/). Unlike the core they do not depend on an example's defines.hpp,
# so they are compiled once and unit-tested on the CPU.
add_library(fluidx3d_setup STATIC
    core/unit_scale.cpp
    domain/lattice.cpp
    domain/geometry_scaler.cpp
    domain/domain_plan.cpp
    sdf/sdf_generator.cpp
)
add_library(fluidx3d::setup ALIAS fluidx3d_setup)
# lib/ is the only include root: the API is always included as "setup/...", so none of its headers
# can shadow a core header of the same name (such as src/setup.hpp)
target_include_directories(fluidx3d_setup PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
# The core's utilities.hpp (vector types, STL loading) does not depend on defines.hpp either; SYSTEM like the rest of the core.
# It includes lodepng.h. The library must not include lbm.hpp.
target_include_directories(fluidx3d_setup SYSTEM PUBLIC ${PROJECT_SOURCE_DIR}/src)
# PUBLIC: get_resource_path() is an inline function of utilities.hpp that depends on this macro, so every translation
# unit that links the library must see the same value (fluidx3d::core defines it the same)
target_compile_definitions(fluidx3d_setup PUBLIC FLUIDX3D_RESOURCE_DIR="${PROJECT_SOURCE_DIR}/resources")
target_link_libraries(fluidx3d_setup
    PUBLIC  fluidx3d::build_options
            fluidx3d::lodepng
    PRIVATE fluidx3d::sdf_cache
            fluidx3d::warnings
)
