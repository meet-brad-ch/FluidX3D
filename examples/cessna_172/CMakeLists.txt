# Example: cessna_172
# Unity build - compiles all core sources with this example's defines.hpp
#
# NOTE: Thingiverse provides "Airplane.stl" (combined model).
# This example expects separated files: Cessna-172-Skyhawk-body.stl and Cessna-172-Skyhawk-rotor.stl
# Manual processing required: Download Airplane.stl, then split into body and rotor using 3D modeling software.

add_fluidx3d_example(
    NAME cessna_172
)
