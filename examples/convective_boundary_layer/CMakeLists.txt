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
