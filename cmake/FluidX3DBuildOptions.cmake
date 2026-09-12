# Build policy as INTERFACE targets:
#   fluidx3d::build_options  C++ standard, optimization and platform flags for every FluidX3D executable
#   fluidx3d::warnings       warnings for first-party code (examples, Setup API, tests), linked PRIVATE;
#                            third-party and core headers are SYSTEM includes and stay quiet
# (The MSVC runtime library and debug-information flags come from CMake itself.)

set(_fluidx3d_gcc_like "$<CXX_COMPILER_ID:GNU,Clang,AppleClang>")
set(_fluidx3d_msvc "$<CXX_COMPILER_ID:MSVC>")

add_library(fluidx3d_build_options INTERFACE)
add_library(fluidx3d::build_options ALIAS fluidx3d_build_options)
target_compile_features(fluidx3d_build_options INTERFACE cxx_std_20)
target_compile_options(fluidx3d_build_options INTERFACE
    $<${_fluidx3d_gcc_like}:-pthread -Wno-comment>
    $<$<AND:${_fluidx3d_gcc_like},$<CONFIG:Debug>>:-g -O0>
    $<$<AND:${_fluidx3d_gcc_like},$<CONFIG:Release>>:-O3 -march=native -ffast-math>
    $<$<AND:${_fluidx3d_gcc_like},$<CONFIG:RelWithDebInfo>>:-g -O2>
    $<${_fluidx3d_msvc}:/MP /wd26451 /wd6386 /wd6001>   # multi-processor compilation; disable specific warnings
    $<$<AND:${_fluidx3d_msvc},$<CONFIG:Debug>>:/Od /RTC1>
    $<$<AND:${_fluidx3d_msvc},$<CONFIG:Release>>:/O2 /Oi /Ot /GL /fp:fast>
    $<$<AND:${_fluidx3d_msvc},$<CONFIG:RelWithDebInfo>>:/O2 /Ot /fp:fast>
)
target_link_options(fluidx3d_build_options INTERFACE
    $<$<AND:${_fluidx3d_msvc},$<CONFIG:Release>>:/LTCG>   # link-time code generation (pairs with /GL)
)

add_library(fluidx3d_warnings INTERFACE)
add_library(fluidx3d::warnings ALIAS fluidx3d_warnings)
target_compile_options(fluidx3d_warnings INTERFACE
    $<${_fluidx3d_msvc}:/W4 /external:anglebrackets /external:W0>
    $<${_fluidx3d_gcc_like}:-Wall -Wextra -Wpedantic>
)
if(FLUIDX3D_WERROR)
    target_compile_options(fluidx3d_warnings INTERFACE $<IF:${_fluidx3d_msvc},/WX,-Werror>)
endif()
