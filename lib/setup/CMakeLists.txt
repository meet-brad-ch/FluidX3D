# The Setup API layers that do not need the LBM: physical quantities and their conversion to the lattice (units/), the
# value types (core/), lattice sizing,
# shapes and the domain plan (domain/), cameras, the step schedule. Unlike the core they do not depend on an
# example's defines.hpp, so they are compiled once and unit-tested on the CPU. The SDF cache (fluidx3d::sdf_cache)
# is theirs to use.
add_library(fluidx3d_setup STATIC
    units/lattice_converter.cpp
    units/moist_air_physics.cpp
    core/time_step_rule.cpp
    core/diffusivity.cpp
    core/scalar_transport_rules.cpp
    core/boussinesq_buoyancy.cpp
    domain/lattice.cpp
    domain/geometry_scaler.cpp
    domain/domain_plan.cpp
    domain/run_estimate.cpp
    domain/level_grid.cpp
    atmosphere/sounding.cpp
    atmosphere/moist_column.cpp
    atmosphere/wind_target.cpp
    atmosphere/sponge_profile.cpp
    atmosphere/start_noise.cpp
    atmosphere/heated_patch.cpp
    atmosphere/boundary_layer_diagnostics.cpp
    atmosphere/run_checks.cpp
    atmosphere/wind_nudging.cpp
    atmosphere/atmosphere_grid.cpp
    atmosphere/cbl_description.cpp
    atmosphere/lattice_parameters.cpp
    domain/domain.cpp
    domain/model_placement.cpp
    domain/shape.cpp
    graphics/camera_view.cpp
    simulation/step_schedule.cpp
)
add_library(fluidx3d::setup ALIAS fluidx3d_setup)
# no fast math: the set-up values round exactly as written, on every compiler (cmake/configs.cmake)
set_target_properties(fluidx3d_setup PROPERTIES FLUIDX3D_PRECISE_FLOAT ON)
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
            Au::au
    PRIVATE fluidx3d::sdf_cache
            fluidx3d::warnings
)
