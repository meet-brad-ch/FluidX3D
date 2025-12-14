# CMake Build System

This document explains the CMake-based build architecture for FluidX3D.

## Project Structure

```
FluidX3D/
├── CMakeLists.txt              # Root CMake configuration
├── cmake/
│   └── FluidX3DExample.cmake   # Generic example builder function
├── resources/                  # Resource files (STL models, textures)
│   ├── skybox8k.png           # Skybox texture for raytracing
│   ├── *.stl                  # 3D models
│   └── download_all_thingiverse_stl.py    # STL downloader script
├── src/                        # Core source files (NO library - Unity Build)
│   ├── graphics.cpp/hpp
│   ├── lbm.cpp/hpp
│   ├── info.cpp/hpp
│   ├── kernel.cpp/hpp
│   ├── shapes.cpp/hpp
│   ├── main.cpp
│   └── utilities.hpp          # Includes get_resource_path()
├── third_party/               # Bundled third-party binaries
│   ├── OpenCL/lib/           # OpenCL ICD loaders
│   └── X11/                  # X11/Xrandr libraries
└── examples/                  # Example simulations (37 total)
    ├── CMakeLists.txt
    ├── taylor_green_3d/
    │   ├── defines.hpp         # Example-specific configuration
    │   ├── main.cpp            # Implements main_setup()
    │   └── CMakeLists.txt      # 1 line: add_fluidx3d_example()
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

---

## Generic Example Builder

All examples use the **`add_fluidx3d_example()`** function from `cmake/FluidX3DExample.cmake`.

### Example CMakeLists.txt

Every example has a simple 1-line CMakeLists.txt:

```cmake
# examples/taylor_green_3d/CMakeLists.txt
add_fluidx3d_example(NAME taylor_green_3d)
```

```cmake
# examples/cow/CMakeLists.txt
add_fluidx3d_example(NAME cow)
```

### What the Function Does

The `add_fluidx3d_example()` function automatically:

1. **Compiles all core sources** (Unity Build):
   - `main.cpp` + `graphics.cpp` + `lbm.cpp` + `info.cpp` + `kernel.cpp` + `shapes.cpp` + fetched `lodepng.cpp`

2. **Sets up include directories** (with `BEFORE PRIVATE` priority):
   - Example's `defines.hpp` (FIRST - shadows core's defines.hpp)
   - Fetched OpenCL C++ bindings
   - Fetched OpenCL C headers
   - Core source headers
   - Fetched LodePNG
   - Project source directory

3. **Configures compiler**: `-O3 -pthread -Wno-comment`

4. **Links libraries**: `Threads::Threads`, `OpenCL`, `X11`, `Xrandr`

### Benefits

| Aspect | Original | CMake Unity Build |
|--------|----------|-------------------|
| **Lines per example** | 47-129 lines | 1 line |
| **Code duplication** | Repeated 37× | Centralized function |
| **Example selection** | Edit source code | `cmake --build build --target <name>` |
| **IDE support** | Limited | Full (VS, CLion, VS Code) |

---

## Configuration Per Example

Each example has its own `examples/<name>/defines.hpp` file that configures:

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
- **Graphics Mode**:
  - `INTERACTIVE_GRAPHICS` (default)
  - `INTERACTIVE_GRAPHICS_ASCII`
  - `GRAPHICS`

### Example Configurations

**taylor_green_3d:**
```cpp
#define D3Q19
#define SRT
#define INTERACTIVE_GRAPHICS
```

**karman_vortex_street:**
```cpp
#define D2Q9                    // 2D simulation
#define SRT
#define FP16S                   // Compression
#define EQUILIBRIUM_BOUNDARIES
#define INTERACTIVE_GRAPHICS
```

**benchmark:**
```cpp
#define D3Q19
#define SRT
#define BENCHMARK               // Disables all extensions
```

**Note**: Each example compiles all core sources with its own configuration (Unity Build). This allows different examples to have different features enabled simultaneously without conflicts.

---

## Adding New Examples

### Simple Example

1. **Create example directory**:
   ```bash
   mkdir examples/my_example
   cp examples/taylor_green_3d/defines.hpp examples/my_example/
   ```

2. **Edit `defines.hpp`**: Enable required extensions

3. **Create `main.cpp`**:
   ```cpp
   #include "defines.hpp"
   #include "lbm.hpp"
   #include "graphics.hpp"

   void main_setup() {
       LBM lbm(128u, 128u, 128u, 0.02f);
       // Your setup here
       lbm.run();
   }
   ```

4. **Create `CMakeLists.txt`**:
   ```cmake
   add_fluidx3d_example(NAME my_example)
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

Place STL files in `resources/` directory and reference them using `get_resource_path()`:

**main.cpp:**
```cpp
Mesh* mesh = read_stl(get_resource_path("my_model.stl"), ...);
```

The `get_resource_path()` function searches:
1. `<project>/resources/` (development)
2. `./resources/` (relative to executable for distributions)

**STL Acquisition:**
- Download manually to `resources/`
- Or use `python resources/download_all_thingiverse_stl.py` for batch download

---

## Dependency Management

### FetchContent (Automatic)

These dependencies are automatically fetched at configure time:

- **LodePNG** (v20200306) - PNG encoding/decoding
- **OpenCL Headers** (v2024.10.24) - Khronos C headers
- **OpenCL C++ Bindings** (v2024.10.24) - Khronos C++ wrapper

Result: ~8200 lines removed from repository.

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

**Usage:**
```cpp
const string skybox = get_resource_path("skybox8k.png");
Mesh* cow = read_stl(get_resource_path("Cow_t.stl"), ...);
```

---

## Technical Details

### Include Path Priority

The `BEFORE PRIVATE` directive ensures correct include order:

1. **Example directory** - Example's `defines.hpp` (FIRST)
2. **Fetched OpenCL C++ bindings**
3. **Fetched OpenCL C headers**
4. **Core headers** (`src/`)
5. **Fetched LodePNG**
6. **Project source directory**

This guarantees each example's `defines.hpp` overrides the core's default configuration.

### Performance Considerations

**Runtime:**
- ✅ No overhead - each example optimally compiled
- ✅ No shared library overhead
- ✅ Full `-O3` optimization per example

---

## Advantages Over Original

| Feature | Original | CMake Build |
|---------|----------|-------------|
| **Build system** | Makefiles | Modern CMake |
| **Example selection** | Edit `setup.cpp` | `--target <name>` |
| **CMakeLists.txt size** | N/A | 1 line per example |
| **Parallel examples** | ❌ | ✅ |
| **IDE integration** | Limited | Full |
| **Dependency management** | Bundled | FetchContent |
| **Repository size** | Larger | -8200 lines |

---

## Summary

The CMake build system provides:

- ✅ **Unity Build**: No ODR violations, each example independent
- ✅ **Simplicity**: 1-line CMakeLists.txt per example
- ✅ **Flexibility**: Different examples with different features
- ✅ **Modern**: IDE support, FetchContent, target-based builds
- ✅ **Maintainable**: Centralized build logic, no duplication

**See [BUILD.md](BUILD.md) for build instructions and [EXAMPLES.md](EXAMPLES.md) for the complete list of examples.**
