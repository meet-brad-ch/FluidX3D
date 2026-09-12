# Platform libraries as usage requirements of fluidx3d::platform:
# the OpenCL headers and bundled ICD loader, system libraries on Windows, X11 and threads on Unix.
add_library(fluidx3d_platform INTERFACE)
add_library(fluidx3d::platform ALIAS fluidx3d_platform)

target_link_directories(fluidx3d_platform INTERFACE ${PROJECT_SOURCE_DIR}/third_party/OpenCL/lib)
target_link_libraries(fluidx3d_platform INTERFACE OpenCL::HeadersCpp OpenCL)

if(WIN32)
    target_link_libraries(fluidx3d_platform INTERFACE
        kernel32 user32 gdi32 winspool comdlg32 advapi32
        shell32 ole32 oleaut32 uuid odbc32 odbccp32
    )
elseif(UNIX)
    find_package(Threads REQUIRED)
    if(APPLE)
        target_link_directories(fluidx3d_platform INTERFACE /opt/X11/lib) # X11 from XQuartz
    else()
        target_link_directories(fluidx3d_platform INTERFACE ${PROJECT_SOURCE_DIR}/third_party/X11/lib) # bundled X11
    endif()
    target_link_libraries(fluidx3d_platform INTERFACE Threads::Threads X11 Xrandr)
endif()
