# The Setup API layers that do not need the LBM: physical quantities and unit scaling (core/),
# lattice sizing (domain/). Unlike the core they do not depend on an example's defines.hpp,
# so they are compiled once and unit-tested on the CPU.
add_library(fluidx3d_setup STATIC
    core/unit_scale.cpp
    domain/lattice.cpp
)
add_library(fluidx3d::setup ALIAS fluidx3d_setup)
# lib/ is the only include root: the API is always included as "setup/...", so none of its headers
# can shadow a core header of the same name (such as src/setup.hpp)
target_include_directories(fluidx3d_setup PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(fluidx3d_setup
    PUBLIC  fluidx3d::build_options
    PRIVATE fluidx3d::warnings
)
