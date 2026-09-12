# Test tools and tests. The per-example baseline tests are registered by add_fluidx3d_example().

# Compares a baseline output with its expectation (used by every baseline_<example> test)
add_executable(fluidx3d_baseline_compare baseline/baseline_compare.cpp)
target_link_libraries(fluidx3d_baseline_compare PRIVATE fluidx3d::build_options fluidx3d::warnings)
