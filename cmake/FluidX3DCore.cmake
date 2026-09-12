# The FluidX3D core. Its sources depend on each executable's defines.hpp, so every executable compiles them
# (unity build, no core library: one shared library would break the one-definition rule across configurations).
# fluidx3d::core carries what they need: headers, definitions and libraries.
set(FLUIDX3D_SRC_DIR ${PROJECT_SOURCE_DIR}/src)
set(FLUIDX3D_CORE_SOURCES
    ${FLUIDX3D_SRC_DIR}/graphics.cpp
    ${FLUIDX3D_SRC_DIR}/info.cpp
    ${FLUIDX3D_SRC_DIR}/kernel.cpp
    ${FLUIDX3D_SRC_DIR}/lbm.cpp
    ${FLUIDX3D_SRC_DIR}/main.cpp
    ${FLUIDX3D_SRC_DIR}/shapes.cpp
)
# Setup API (lib/setup), compiled per executable as well (it includes lbm.hpp)
set(FLUIDX3D_SETUP_DIR ${PROJECT_SOURCE_DIR}/lib/setup)
set(FLUIDX3D_SETUP_SOURCES
    ${FLUIDX3D_SETUP_DIR}/simulation/geometry_scaler.cpp
    ${FLUIDX3D_SETUP_DIR}/sdf/sdf_generator.cpp
)

add_library(fluidx3d_core INTERFACE)
add_library(fluidx3d::core ALIAS fluidx3d_core)
# The upstream core is treated as third-party code (SYSTEM: its warnings are not ours to fix).
target_include_directories(fluidx3d_core SYSTEM INTERFACE ${FLUIDX3D_SRC_DIR}) # lbm.hpp, ...
# The Setup API is first-party: fluidx3d::warnings applies to its headers too.
target_include_directories(fluidx3d_core INTERFACE
    ${PROJECT_SOURCE_DIR}/lib   # "setup/setup.hpp"
    ${FLUIDX3D_SETUP_DIR}       # the API's own includes ("core/types.hpp", ...)
)
target_compile_definitions(fluidx3d_core INTERFACE FLUIDX3D_RESOURCE_DIR="${PROJECT_SOURCE_DIR}/resources")
target_link_libraries(fluidx3d_core INTERFACE
    fluidx3d::build_options
    fluidx3d::setup     # the Setup API layers that do not need the LBM
    fluidx3d::platform
    fluidx3d::lodepng
    fluidx3d::sdf_cache
)
