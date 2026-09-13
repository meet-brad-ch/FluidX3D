# CMake Build System

This document explains the CMake-based build architecture for FluidX3D.

## Project Structure

```
FluidX3D/
├── CMakeLists.txt              # Root CMake configuration
├── cmake/
│   ├── configs.cmake           # Options, build type, fluidx3d::build_options, fluidx3d::warnings
│   ├── FluidX3DDependencies.cmake  # Pinned third-party code (FetchContent)
│   ├── FluidX3DPlatform.cmake  # fluidx3d::platform: OpenCL, system libraries, X11
│   ├── FluidX3DCore.cmake      # fluidx3d::core and the core source lists
│   └── FluidX3DExample.cmake   # add_fluidx3d_example()
├── resources/                  # Resource files (STL models, textures)
│   ├── skybox8k.png           # Skybox texture for raytracing
│   ├── *.stl                  # 3D models
│   └── download_all_thingiverse_stl.py    # STL downloader script
├── src/                        # Upstream core sources (NO library - Unity Build)
│   ├── graphics.cpp/hpp
│   ├── lbm.cpp/hpp
│   ├── info.cpp/hpp
│   ├── kernel.cpp/hpp
│   ├── shapes.cpp/hpp
│   ├── main.cpp
│   ├── sdf_cache/             # SDF generation and caching (static library)
│   └── utilities.hpp          # Includes get_resource_path()
├── lib/setup/                  # Setup API (#include "setup/setup.hpp")
│   ├── CMakeLists.txt         # fluidx3d::setup: the layers that do not need the LBM
│   └── ...
├── tests/
│   ├── CMakeLists.txt         # GoogleTest, fluidx3d_add_test(), unit tests
│   ├── unit/                  # CPU unit tests (ctest -L unit)
│   ├── headers/               # Every Setup API header compiles on its own (build check)
│   └── physics/               # Simulations against analytic solutions (ctest -L physics)
├── third_party/               # Bundled third-party binaries
│   ├── OpenCL/lib/           # OpenCL ICD loaders
│   └── X11/                  # X11/Xrandr libraries
└── examples/                  # Example simulations (41 total)
    ├── CMakeLists.txt
    ├── taylor_green_3d/
    │   ├── main.cpp            # Implements main_setup()
    │   └── CMakeLists.txt      # add_fluidx3d_example(NAME ... EXTENSIONS ...): its configuration
    └── ...
```

---

## Unity Build Architecture

### Why Unity Build?

This build system uses a **unity build** approach where each example compiles all core sources directly with its own configuration.

**Problem with Shared Library Approach:**
- Core library compiled once with one `defines.hpp` configuration
- Examples compiled with different `defines.hpp` configurations
- **Result**: ODR (One Definition Rule) violations, class layout mismatches, linker errors

**Solution: Unity Build:**
- Each example compiles ALL core sources (`graphics.cpp`, `lbm.cpp`, `info.cpp`, etc.)
- Each uses its own `defines.hpp` configuration
- No shared library = No ODR violations
- Matches original FluidX3D architecture philosophy

**Exception: the Setup API library.** The parts of the Setup API that do not include `lbm.hpp` (physical quantities, unit scaling, lattice and geometry sizing, the domain plan, which uses the SDF cache) do not depend on `defines.hpp`; what they need from an example's configuration, the bytes per lattice cell, is passed in. They are compiled once into the static library `fluidx3d::setup` (`lib/setup/CMakeLists.txt`) and unit-tested on the CPU. The parts that use the LBM are compiled per example, like the core.

---

## Targets and Settings

All build settings are attached to targets (`target_*` commands); there are no global flags.

| Target | Kind | Carries |
|--------|------|---------|
| `fluidx3d::build_options` | INTERFACE | C++20, optimization and per-compiler flags per build type |
| `fluidx3d::warnings` | INTERFACE | `/W4` or `-Wall -Wextra -Wpedantic`; errors with `-DFLUIDX3D_WERROR=ON` |
| `fluidx3d::platform` | INTERFACE | OpenCL, system libraries, X11/Xrandr, threads |
| `fluidx3d::core` | INTERFACE | Core headers (as SYSTEM headers), resource path, LodePNG, SDF cache |
| `fluidx3d::setup` | STATIC | Setup API layers without the LBM |

The upstream core in `src/` is included as SYSTEM headers: its warnings are not reported. The Setup API in `lib/setup/` is first-party code: `fluidx3d::warnings` applies to it.

---

## Generic Example Builder

All examples use the **`add_fluidx3d_example()`** function from `cmake/FluidX3DExample.cmake`.

### Example CMakeLists.txt

Every example has a CMakeLists.txt of 1-3 lines:

```cmake
# examples/taylor_green_3d/CMakeLists.txt
add_fluidx3d_example(NAME taylor_green_3d
    EXTENSIONS INTERACTIVE_GRAPHICS
)
```

```cmake
# examples/cow/CMakeLists.txt
add_fluidx3d_example(NAME cow
    EXTENSIONS FP16S EQUILIBRIUM_BOUNDARIES SUBGRID INTERACTIVE_GRAPHICS
)
```

### What the Function Does

For an example `<name>`, `add_fluidx3d_example()` creates:

1. **`<name>_setup`** (OBJECT library): the example's `main.cpp`, which also compiles the Setup API headers that use the LBM, with its generated `defines.hpp` (see [Configuration Per Example](#configuration-per-example)) on the include path and `fluidx3d::warnings` applied.
2. **`<name>`** (executable in `bin/`): the core sources (`FLUIDX3D_CORE_SOURCES` in `cmake/FluidX3DCore.cmake`), linked with `<name>_setup` and `fluidx3d::core`.

### Benefits

| Aspect | Original | CMake Unity Build |
|--------|----------|-------------------|
| **Lines per example** | 47-129 lines | 1-3 lines (the extensions) |
| **Code duplication** | Repeated 37× | Centralized function |
| **Example selection** | Edit source code | `cmake --build build --target <name>` |
| **IDE support** | Limited | Full (VS, CLion, VS Code) |

---

## Configuration Per Example

Each example names its configuration in its `CMakeLists.txt`; `add_fluidx3d_example()` generates the `defines.hpp` the core reads from `cmake/defines.hpp.in` (`fluidx3d_configure_defines()` in `cmake/FluidX3DExample.cmake`). A macro the template does not know stops the configuration with a message.

```cmake
add_fluidx3d_example(NAME <name>
    EXTENSIONS <macros...>                 # the options below, in any order
    SETTINGS "<macro> <value>" ...         # optional: graphics settings with a value
)
```

### Available Options

- **Velocity Set**: `D2Q9`, `D3Q15`, `D3Q19` (default), `D3Q27`
- **Collision Operator**: `SRT` (default), `TRT`
- **Compression**: `FP16S`, `FP16C`
- **Extensions**:
  - `VOLUME_FORCE`
  - `FORCE_FIELD`
  - `EQUILIBRIUM_BOUNDARIES`
  - `MOVING_BOUNDARIES`
  - `SURFACE`
  - `TEMPERATURE`
  - `SUBGRID`
  - `PARTICLES`
- **Graphics Mode** (none: console only):
  - `INTERACTIVE_GRAPHICS`
  - `INTERACTIVE_GRAPHICS_ASCII`
  - `GRAPHICS`
- **Settings** (`SETTINGS`): `GRAPHICS_FRAME_WIDTH`, `GRAPHICS_FRAME_HEIGHT`, `GRAPHICS_BACKGROUND_COLOR`, `GRAPHICS_U_MAX`, `GRAPHICS_TRANSPARENCY`, ... (the defaults and their meaning: `cmake/defines.hpp.in`)

### Example Configurations

```cmake
add_fluidx3d_example(NAME taylor_green_3d
    EXTENSIONS INTERACTIVE_GRAPHICS
)
add_fluidx3d_example(NAME karman_vortex_street
    EXTENSIONS D2Q9 FP16S EQUILIBRIUM_BOUNDARIES INTERACTIVE_GRAPHICS
)
add_fluidx3d_example(NAME city
    EXTENSIONS FP16S EQUILIBRIUM_BOUNDARIES SUBGRID INTERACTIVE_GRAPHICS
    SETTINGS "GRAPHICS_FRAME_WIDTH 3840" "GRAPHICS_FRAME_HEIGHT 2160" "GRAPHICS_U_MAX 0.15f"
)
add_fluidx3d_example(NAME benchmark
    EXTENSIONS BENCHMARK        # disables all extensions
)
```

**Note**: Each example compiles all core sources with its own configuration (Unity Build). This allows different examples to have different features enabled simultaneously without conflicts.

---

## Adding New Examples

### Simple Example

1. **Create example directory**:
   ```bash
   mkdir examples/my_example
   ```

2. **Name the extensions** in its `CMakeLists.txt` (step 4)

3. **Create `main.cpp`** with the Setup API (see [SETUP_API.md](SETUP_API.md)):
   ```cpp
   #include "setup/setup.hpp"

   void main_setup() {
       const Speed flow = 10.0_mps;
       Simulation sim(Domain::around(Model("my_model.stl").length(2.0_m))
           .size(2.0_m, 6.0_m, 2.0_m).on_floor().vram(2000_mb),
           Fluid::AIR, flow);
       sim.boundaries().set_solid_floor().set_open_boundaries().initialize_velocity_y(flow).apply();
       sim.graphics().show_surface().show_vortices().apply();
       sim.video().add(CameraView::orbit(-40_deg, 20_deg)).set_length(10.0_s); // written when built for video
       sim.run_for(10.0_s);
   }
   ```

4. **Create `CMakeLists.txt`**:
   ```cmake
   add_fluidx3d_example(NAME my_example
       EXTENSIONS FP16S INTERACTIVE_GRAPHICS
   )
   ```

5. **Add to `examples/CMakeLists.txt`**:
   ```cmake
   add_subdirectory(my_example)
   ```

6. **Build**:
   ```bash
   cmake --build build --target my_example
   ```

### Example with STL Files

Place STL files in `resources/` directory and name them in the `Model`; the simulation finds, scales, places and voxelizes them:

**main.cpp:**
```cpp
Simulation sim(Domain::around(Model("my_model.stl").length(2.0_m)).size(...), Fluid::AIR, 10.0_mps);
```

A missing file stops the program with a message; `Model::instructions({...})` adds how to get it. Underneath, the core's `get_resource_path()` function searches:
1. `<project>/resources/` (development)
2. `./resources/` (relative to executable for distributions)

**STL Acquisition:**
- Download manually to `resources/`
- Or use `python resources/download_all_thingiverse_stl.py` for batch download

---

## Tests

Tests are built when `FLUIDX3D_BUILD_TESTS` is on (the default) and run with CTest.

| Label | What it checks | Needs |
|-------|----------------|-------|
| `unit` | Setup API library (quantities, unit scaling, lattice and geometry sizing, shapes, cameras), one CTest test per GoogleTest `TEST` | CPU only |
| `physics` | Simulations in physical units against analytic solutions: Poiseuille profile (L2 error < 1 %), Stokes drag (< 5 %), Taylor-Green decay (viscosity within 1 %), hydrostatic pressure (< 1 %); about 15 s in total | OpenCL device |

```bash
ctest --test-dir build -L unit
ctest --test-dir build -L physics
```

The build also checks that every Setup API header compiles on its own (`tests/headers/`): each header gets a source file that includes only it, compiled with every extension and with none. A header that needs an extension stops with an `#error` that names it (for example `surface_builder.hpp` without `SURFACE`) and is left out of the configuration without it.

Unit tests are added with `fluidx3d_add_test(<name> SOURCES <files...> LINK <targets...>)` in `tests/CMakeLists.txt`. A physics test is a `main_setup()` in `tests/physics/<name>.cpp` that ends with `PhysicsCheck::report()`, added with `fluidx3d_add_physics_test(<name> EXTENSIONS <defines...>)` in `tests/physics/CMakeLists.txt`; its `defines.hpp` is generated from `cmake/defines.hpp.in` with those extensions, as an example's.

---

## Dependency Management

### FetchContent (Automatic)

These dependencies are automatically fetched at configure time (pinned versions, as SYSTEM so their warnings are not reported):

- **LodePNG** - PNG encoding/decoding
- **OpenCL Headers** (v2024.10.24) - Khronos C headers
- **OpenCL C++ Bindings** (v2024.10.24) - Khronos C++ wrapper
- **SDFGen** (v1.0.0) - GPU signed distance field generation, used by the SDF cache
- **GoogleTest** (v1.15.2) - unit tests (only with `FLUIDX3D_BUILD_TESTS`)

### Bundled Libraries

Platform-specific binaries in `third_party/`:

- **OpenCL ICD loaders** (`third_party/OpenCL/lib/`) - Dispatch libraries
- **X11/Xrandr** (`third_party/X11/`) - Linux/macOS graphics libraries

---

## Resource Path System

The `get_resource_path()` function (in `src/utilities.hpp`) provides flexible resource location:

```cpp
inline string get_resource_path(const string& relative_path) {
    // Location 1: CMake compile-time path (development)
    #ifdef FLUIDX3D_RESOURCE_DIR
        const string cmake_path = string(FLUIDX3D_RESOURCE_DIR) + "/" + relative_path;
        if(std::filesystem::exists(cmake_path)) {
            return cmake_path;
        }
    #endif

    // Location 2: Relative to executable (distributions)
    const string exe_path = get_exe_path() + "resources/" + relative_path;
    if(std::filesystem::exists(exe_path)) {
        return exe_path;
    }

    // Not found
    print_error("Resource not found: " + relative_path);
    return "";
}
```

**Usage** (the Setup API calls it for a `Model`'s files; by hand for other resources):
```cpp
const string skybox = get_resource_path("skybox8k.png");
```

---

## Technical Details

### Include Path Priority

Each example's generated `defines.hpp` (`build/<...>/examples/<name>/defines/`) comes first on its include path, before the core's `src/`.

### Performance Considerations

**Runtime:**
- ✅ No overhead - each example optimally compiled
- ✅ No shared library overhead
- ✅ Full optimization per example (`-O3 -march=native -ffast-math` or `/O2 /GL /fp:fast` in Release)

---

## Advantages Over Original

| Feature | Original | CMake Build |
|---------|----------|-------------|
| **Build system** | Makefiles | Modern CMake |
| **Example selection** | Edit `setup.cpp` | `--target <name>` |
| **CMakeLists.txt size** | N/A | 1-3 lines per example |
| **Parallel examples** | ❌ | ✅ |
| **IDE integration** | Limited | Full |
| **Dependency management** | Bundled | FetchContent |
| **Tests** | None | Unit and physics tests (CTest), a header check |

---

## Summary

The CMake build system provides:

- ✅ **Unity Build**: No ODR violations, each example independent
- ✅ **Simplicity**: a CMakeLists.txt of 1-3 lines per example, naming its extensions
- ✅ **Flexibility**: Different examples with different features
- ✅ **Modern**: IDE support, FetchContent, target-based builds
- ✅ **Maintainable**: Centralized build logic, no duplication
- ✅ **Tested**: CTest unit and physics tests

**See [BUILD.md](BUILD.md) for build instructions and [EXAMPLES.md](EXAMPLES.md) for the complete list of examples.**
