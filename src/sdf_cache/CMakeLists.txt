# fluidx3d::sdf_cache: signed distance fields of STL models, generated with SDFGen and cached on disk
add_library(sdf_cache STATIC
    sdf_cache.cpp
    hash_utils.cpp
)
add_library(fluidx3d::sdf_cache ALIAS sdf_cache)
target_include_directories(sdf_cache PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(sdf_cache PRIVATE SDFGEN_VERSION_TAG="${SDFGEN_TAG}") # the SDFGen version is part of the cache key
target_link_libraries(sdf_cache
    PUBLIC  sdfgen_common # SDFGen's unified interface (CPU and GPU)
            fluidx3d::build_options
    PRIVATE fluidx3d::warnings
)
