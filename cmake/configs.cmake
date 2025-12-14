   # ==========================================================================
   # Compiler options (cross-platform with Debug/Release configurations)
   # ==========================================================================
   target_compile_features(${EXAMPLE_NAME} PRIVATE cxx_std_17)

   # GCC/Clang flags - Debug configuration
   target_compile_options(${EXAMPLE_NAME} PRIVATE
           $<$<AND:$<CXX_COMPILER_ID:GNU>,$<CONFIG:Debug>>:-std=c++17 -pthread -g -O0 -Wno-comment>
           $<$<AND:$<CXX_COMPILER_ID:Clang>,$<CONFIG:Debug>>:-std=c++17 -pthread -g -O0 -Wno-comment>
           $<$<AND:$<CXX_COMPILER_ID:AppleClang>,$<CONFIG:Debug>>:-std=c++17 -pthread -g -O0 -Wno-comment>
   )

   # GCC/Clang flags - Release configuration
   target_compile_options(${EXAMPLE_NAME} PRIVATE
           $<$<AND:$<CXX_COMPILER_ID:GNU>,$<CONFIG:Release>>:-std=c++17 -pthread -O3 -march=native -ffast-math -Wno-comment>
           $<$<AND:$<CXX_COMPILER_ID:Clang>,$<CONFIG:Release>>:-std=c++17 -pthread -O3 -march=native -ffast-math -Wno-comment>
           $<$<AND:$<CXX_COMPILER_ID:AppleClang>,$<CONFIG:Release>>:-std=c++17 -pthread -O3 -march=native -ffast-math -Wno-comment>
   )

   # GCC/Clang flags - RelWithDebInfo configuration
   target_compile_options(${EXAMPLE_NAME} PRIVATE
           $<$<AND:$<CXX_COMPILER_ID:GNU>,$<CONFIG:RelWithDebInfo>>:-std=c++17 -pthread -g -O2 -Wno-comment>
           $<$<AND:$<CXX_COMPILER_ID:Clang>,$<CONFIG:RelWithDebInfo>>:-std=c++17 -pthread -g -O2 -Wno-comment>
           $<$<AND:$<CXX_COMPILER_ID:AppleClang>,$<CONFIG:RelWithDebInfo>>:-std=c++17 -pthread -g -O2 -Wno-comment>
   )

   # MSVC flags - Debug configuration
   target_compile_options(${EXAMPLE_NAME} PRIVATE
           $<$<AND:$<CXX_COMPILER_ID:MSVC>,$<CONFIG:Debug>>:
           /Od                      # No optimization
           /Zi                      # Debug symbols
           /MP                      # Multi-processor compilation
           /RTC1                    # Runtime checks
           /MDd                     # Debug runtime library
           /wd26451 /wd6386 /wd6001 # Disable specific warnings
           >
   )

   # MSVC flags - Release configuration
   target_compile_options(${EXAMPLE_NAME} PRIVATE
           $<$<AND:$<CXX_COMPILER_ID:MSVC>,$<CONFIG:Release>>:
           /O2                      # Maximum optimization
           /Oi                      # Enable intrinsics
           /MP                      # Multi-processor compilation
           /Ot                      # Favor fast code
           /GL                      # Whole program optimization
           /fp:fast                 # Fast floating point
           /MD                      # Release runtime library
           /wd26451 /wd6386 /wd6001 # Disable specific warnings
           >
   )

   # MSVC flags - RelWithDebInfo configuration
   target_compile_options(${EXAMPLE_NAME} PRIVATE
           $<$<AND:$<CXX_COMPILER_ID:MSVC>,$<CONFIG:RelWithDebInfo>>:
           /O2                      # Maximum optimization
           /Zi                      # Debug symbols
           /MP                      # Multi-processor compilation
           /Ot                      # Favor fast code
           /fp:fast                 # Fast floating point
           /MD                      # Release runtime library
           /wd26451 /wd6386 /wd6001 # Disable specific warnings
           >
   )

   # MSVC linker flags - Release only
   target_link_options(${EXAMPLE_NAME} PRIVATE
           $<$<AND:$<CXX_COMPILER_ID:MSVC>,$<CONFIG:Release>>:/LTCG> # Link-time code generation
   )