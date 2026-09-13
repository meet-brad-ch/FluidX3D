# The original examples (before the Setup API), built as baseline targets: ctest -L original
# Their sources come from git (FLUIDX3D_ORIGINALS_REF); their expected output is in baselines/.
# Compare an original with its port: cmake -P tests/originals/compare_port.cmake (see there).

find_package(Git REQUIRED)
set(FLUIDX3D_ORIGINALS_REF f3a0e26 CACHE STRING "Git revision of the original examples")

# the examples ported to the Setup API
set(FLUIDX3D_ORIGINALS
    ahmed_body bell_222 boeing_747 breaking_waves bursting_bubble cessna_172 city city_rt colliding_droplets concorde
    cow cow_sdf cube_gravity cylinder_duct dam_break delta_wing edf hydraulic_jump karman_vortex_street
    lid_driven_cavity liquid_metal mercedes_f1 nasa_crm particle_test periodic_faucet poiseuille_flow radial_fan
    raindrop rayleigh_benard river space_shuttle starship stokes_drag taylor_couette taylor_green_2d taylor_green_3d
    thermal_convection tie_fighter xwing
)

foreach(name IN LISTS FLUIDX3D_ORIGINALS)
    set(source_dir ${CMAKE_CURRENT_BINARY_DIR}/${name})
    file(MAKE_DIRECTORY ${source_dir})
    foreach(file main.cpp defines.hpp)
        execute_process(
            COMMAND ${GIT_EXECUTABLE} show ${FLUIDX3D_ORIGINALS_REF}:examples/${name}/${file}
            WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
            OUTPUT_FILE ${source_dir}/${file}.git
            RESULT_VARIABLE git_result
        )
        if(NOT git_result EQUAL 0)
            message(FATAL_ERROR "git show ${FLUIDX3D_ORIGINALS_REF}:examples/${name}/${file} failed")
        endif()
        configure_file(${source_dir}/${file}.git ${source_dir}/${file} COPYONLY) # rewritten only when it changed
    endforeach()

    _fluidx3d_add_baseline(${name}_original
        SOURCE_DIR ${source_dir}
        BINARY_DIR ${source_dir}/baseline
        TEST original_${name}
        EXPECTED ${CMAKE_CURRENT_SOURCE_DIR}/baselines/${name}.txt
        LABELS original gpu
        DEFINES_FILE ${source_dir}/defines.hpp
        THIRD_PARTY
    )
endforeach()
