add_fluidx3d_example(NAME convective_boundary_layer
    EXTENSIONS VOLUME_FORCE TEMPERATURE SUBGRID THERMAL_DIFFUSIVITY_FIX LEVEL_FORCING MOISTURE # the fix: alpha is the diffusivity; MOISTURE: the water lattice (without set_level_moist the buoyancy stays dry and Q is a passive tracer)
)
# The same example with FP16S storage of the DDFs. FP16S also converts the thermal DDFs gi (see the
# thermal_diffusivity_fp16s test); this target measures what that costs on the CBL.
add_fluidx3d_example(NAME convective_boundary_layer_fp16s
    EXTENSIONS FP16S VOLUME_FORCE TEMPERATURE SUBGRID THERMAL_DIFFUSIVITY_FIX LEVEL_FORCING
)
# The same example with T and Q carried by the conservative link-flux finite volumes (SCALAR_FV) instead of the D3Q7
# lattices, in any number of domains (a 2-layer halo for the finite volumes).
add_fluidx3d_example(NAME convective_boundary_layer_fv
    EXTENSIONS VOLUME_FORCE TEMPERATURE SUBGRID THERMAL_DIFFUSIVITY_FIX LEVEL_FORCING MOISTURE SCALAR_FV
)
# The finite-volume example with FP16S storage of the flow's DDFs (95 B per cell against 133): with SCALAR_FV the
# scalars stay FP32 and only fi is compressed. The question it answers: does the CBL stay within the seed spread?
add_fluidx3d_example(NAME convective_boundary_layer_fv_fp16s
    EXTENSIONS FP16S VOLUME_FORCE TEMPERATURE SUBGRID THERMAL_DIFFUSIVITY_FIX LEVEL_FORCING MOISTURE SCALAR_FV
)
