# Example: bell_222
# Unity build - compiles all core sources with this example's defines.hpp
#
# NOTE: Thingiverse provides "BELL222__FIXED.stl" (combined model).
# This example expects separated files: Bell-222-body.stl, Bell-222-main.stl, Bell-222-back.stl
# Manual processing required: Download BELL222__FIXED.stl, then split into body and rotors using 3D modeling software.

add_fluidx3d_example(
    NAME bell_222
)
