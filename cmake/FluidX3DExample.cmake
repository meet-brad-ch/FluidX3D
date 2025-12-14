# FluidX3D Example Builder
# Generic CMake function to create FluidX3D examples with optional STL downloads

function(add_fluidx3d_example)
    # Parse arguments
    cmake_parse_arguments(
        EXAMPLE                          # Prefix for output variables
        ""                               # Options (boolean flags)
        "NAME"                          # Single-value arguments
        "THINGIVERSE_STL"               # Multi-value arguments (pairs: thing_id stl_filename)
        ${ARGN}
    )

    # Validate required arguments
    if(NOT EXAMPLE_NAME)
        message(FATAL_ERROR "add_fluidx3d_example: NAME argument is required")
    endif()

    # ==========================================================================
    # Create executable with all core sources (Unity Build)
    # ==========================================================================
    add_executable(${EXAMPLE_NAME}
        main.cpp
        ${FLUIDX3D_SRC_DIR}/graphics.cpp
        ${FLUIDX3D_SRC_DIR}/info.cpp
        ${FLUIDX3D_SRC_DIR}/kernel.cpp
        ${FLUIDX3D_SRC_DIR}/lbm.cpp
        ${FLUIDX3D_SRC_DIR}/main.cpp
        ${FLUIDX3D_SRC_DIR}/shapes.cpp
        ${FLUIDX3D_LODEPNG_DIR}/lodepng.cpp
    )

    # ==========================================================================
    # Include directories (example's defines.hpp overrides core's)
    # ==========================================================================
    target_include_directories(${EXAMPLE_NAME} BEFORE PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}        # Example's defines.hpp
        ${FLUIDX3D_OPENCL_CLHPP_DIR}       # Fetched OpenCL C++ bindings (opencl.hpp)
        ${FLUIDX3D_OPENCL_HEADERS_DIR}     # Fetched OpenCL C headers (cl.h, cl_platform.h, etc.)
        ${FLUIDX3D_SRC_DIR}                # FluidX3D source headers (lbm.hpp, graphics.hpp, etc.)
        ${FLUIDX3D_LODEPNG_DIR}            # Fetched LodePNG
        ${PROJECT_SOURCE_DIR}/src          # For relative includes
    )

    include(configs)

    # ==========================================================================
    # Link directories for bundled libraries (platform-specific)
    # ==========================================================================
    target_link_directories(${EXAMPLE_NAME} PRIVATE
        ${FLUIDX3D_OPENCL_LIB_DIR}
    )

    # X11 libraries only on Unix-like systems
    if(UNIX)
        target_link_directories(${EXAMPLE_NAME} PRIVATE
            ${FLUIDX3D_X11_DIR}/lib
        )
    endif()

    # ==========================================================================
    # Set output directories (place executable next to stl/ folder)
    # ==========================================================================
    # For multi-config generators (Visual Studio), put exe in build/examples/<name>/
    # CRITICAL FIX: Output to project root's bin/ directory
    # The executable MUST be in ${PROJECT_SOURCE_DIR}/bin/ for runtime to work correctly.
    # When placed in build directories, initialization fails with "Memory size must be larger than 0"
    # See runtime_problem_solution.md for detailed explanation
    set_target_properties(${EXAMPLE_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/bin"
        RUNTIME_OUTPUT_DIRECTORY_DEBUG "${PROJECT_SOURCE_DIR}/bin"
        RUNTIME_OUTPUT_DIRECTORY_RELEASE "${PROJECT_SOURCE_DIR}/bin"
        RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${PROJECT_SOURCE_DIR}/bin"
        RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${PROJECT_SOURCE_DIR}/bin"
    )

    # ==========================================================================
    # Link libraries (platform-specific)
    # ==========================================================================
    # Common libraries
    target_link_libraries(${EXAMPLE_NAME} PRIVATE OpenCL)

    # Platform-specific libraries
    if(WIN32)
        # Windows: System libraries required by OpenCL
        target_link_libraries(${EXAMPLE_NAME} PRIVATE
            kernel32 user32 gdi32 winspool comdlg32 advapi32
            shell32 ole32 oleaut32 uuid odbc32 odbccp32
        )
    elseif(UNIX)
        # Linux/Mac: X11 and threading
        target_link_libraries(${EXAMPLE_NAME} PRIVATE
            Threads::Threads
            X11
            Xrandr
        )
    endif()

endfunction()
