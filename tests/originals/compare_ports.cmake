# Prints each original example's baseline beside its port's, "!=" where they differ.
#   cmake -DCOMPARE=<build>/tests/fluidx3d_baseline_compare[.exe] [-DNAMES=edf;cow] -P tests/originals/compare_ports.cmake
# Reads the stored baselines only (tests/originals/baselines/<name>.txt, tests/baselines/<name>.txt): run the
# original_<name> tests with FLUIDX3D_BLESS=1 first to record the originals.

get_filename_component(tests_dir ${CMAKE_CURRENT_LIST_DIR} DIRECTORY)
if(NOT NAMES)
    file(GLOB originals ${CMAKE_CURRENT_LIST_DIR}/baselines/*.txt)
    set(NAMES)
    foreach(file IN LISTS originals)
        get_filename_component(name ${file} NAME_WE)
        list(APPEND NAMES ${name})
    endforeach()
endif()

foreach(name IN LISTS NAMES)
    set(original ${CMAKE_CURRENT_LIST_DIR}/baselines/${name}.txt)
    set(port ${tests_dir}/baselines/${name}.txt)
    if(NOT EXISTS ${original} OR NOT EXISTS ${port})
        message("=== ${name}: no baseline (original or port)")
        continue()
    endif()
    execute_process(COMMAND ${COMPARE} --report ${original} ${port} OUTPUT_VARIABLE table)
    message("=== ${name}  (original | port)\n${table}")
endforeach()
