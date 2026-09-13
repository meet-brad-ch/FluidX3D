# Self-contained headers: each Setup API header compiles on its own, included twice (include guard). A build check:
# the object libraries are part of the default build and are not run.
# Two configurations: every extension (all of the headers' extension branches) and none (the headers that need an
# extension say so with an #error).

file(GLOB_RECURSE fluidx3d_setup_headers CONFIGURE_DEPENDS RELATIVE ${PROJECT_SOURCE_DIR}/lib ${PROJECT_SOURCE_DIR}/lib/setup/*.hpp)

# fluidx3d_add_header_check(<variant> [EXTENSIONS <defines...>])
function(fluidx3d_add_header_check variant)
    cmake_parse_arguments(PARSE_ARGV 1 ARG "" "" "EXTENSIONS")
    set(dir ${CMAKE_CURRENT_BINARY_DIR}/${variant})
    fluidx3d_configure_defines(${dir} EXTENSIONS ${ARG_EXTENSIONS})
    set(sources)
    foreach(header IN LISTS fluidx3d_setup_headers)
        # a header that needs an extension this variant lacks stops with its #error "... needs <EXTENSION> ..."
        file(STRINGS ${PROJECT_SOURCE_DIR}/lib/${header} needs REGEX "^#error \".* needs [A-Z_]+")
        if(needs MATCHES "needs ([A-Z_]+)" AND NOT CMAKE_MATCH_1 IN_LIST ARG_EXTENSIONS)
            continue()
        endif()
        string(MAKE_C_IDENTIFIER ${header} source_name)
        file(CONFIGURE OUTPUT ${dir}/${source_name}.cpp CONTENT "#include \"${header}\"\n#include \"${header}\"\n")
        list(APPEND sources ${dir}/${source_name}.cpp)
    endforeach()
    add_library(fluidx3d_header_check_${variant} OBJECT ${sources})
    target_include_directories(fluidx3d_header_check_${variant} PRIVATE ${dir})
    target_link_libraries(fluidx3d_header_check_${variant} PRIVATE fluidx3d::core fluidx3d::warnings)
endfunction()

fluidx3d_add_header_check(all_extensions EXTENSIONS
    VOLUME_FORCE FORCE_FIELD EQUILIBRIUM_BOUNDARIES MOVING_BOUNDARIES SURFACE TEMPERATURE SUBGRID PARTICLES GRAPHICS)
fluidx3d_add_header_check(no_extensions)
