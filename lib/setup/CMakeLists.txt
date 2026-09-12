# The Setup API layers that do not need the LBM: physical quantities and unit scaling (core/),
# lattice sizing (domain/). Unlike the core they do not depend on an example's defines.hpp,
# so they are compiled once and unit-tested on the CPU.
add_library(fluidx3d_setup STATIC
    core/unit_scale.cpp
    domain/lattice.cpp
)
add_library(fluidx3d::setup ALIAS fluidx3d_setup)
target_include_directories(fluidx3d_setup PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/..  # "setup/core/quantity.hpp"
    ${CMAKE_CURRENT_SOURCE_DIR}     # the API's own includes ("core/quantity.hpp")
)
target_link_libraries(fluidx3d_setup
    PUBLIC  fluidx3d::build_options
    PRIVATE fluidx3d::warnings
)
