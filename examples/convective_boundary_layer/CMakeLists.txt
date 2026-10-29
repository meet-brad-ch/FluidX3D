# The convective boundary layer with T and Q carried by the conservative link-flux finite volumes (SCALAR_FV), in any
# number of domains.
add_fluidx3d_example(NAME convective_boundary_layer_fv
    EXTENSIONS VOLUME_FORCE TEMPERATURE SUBGRID THERMAL_DIFFUSIVITY_FIX LEVEL_FORCING MOISTURE SCALAR_FV
)
# The same with FP16S storage of the flow's DDFs: 95 bytes per cell against 133. With SCALAR_FV the scalars stay FP32,
# only fi is compressed.
add_fluidx3d_example(NAME convective_boundary_layer_fv_fp16s
    EXTENSIONS FP16S VOLUME_FORCE TEMPERATURE SUBGRID THERMAL_DIFFUSIVITY_FIX LEVEL_FORCING MOISTURE SCALAR_FV
)
# the example's own code without fast math: its set-up values and diagnostics round as written, the same as the
# Setup API's (cmake/configs.cmake); the core and the kernel keep theirs
set_target_properties(convective_boundary_layer_fv_setup convective_boundary_layer_fv_fp16s_setup PROPERTIES FLUIDX3D_PRECISE_FLOAT ON)
