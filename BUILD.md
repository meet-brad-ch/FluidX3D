# Build Instructions

How to build FluidX3D with CMake on Windows, Linux, and macOS.

## Quick Start

```bash
# Configure
cmake -B build

# Build specific example
cmake --build build --target benchmark

# Run
./bin/benchmark
```

---

## Prerequisites

### All Platforms
- **CMake 3.20+**
- **C++17-compatible compiler**
- **OpenCL Runtime** - Install GPU drivers or [Intel CPU Runtime](https://www.intel.com/content/www/us/en/developer/articles/technical/intel-cpu-runtime-for-opencl-applications-with-sycl-support.html)

### Windows
- **Visual Studio 2019+** with Desktop C++ development
- **Python 3** (optional, for STL downloads)

### Linux
- **g++ 8.0+** (C++17 support)
- X11 libraries (usually pre-installed)

### macOS
- **Xcode Command Line Tools**
- X11 libraries

---

## Building on Windows

### Command Line (Recommended)

**1. Open Developer Command Prompt**

Open **"x64 Native Tools Command Prompt for VS 2019"** (or VS 2022) from Start Menu.

**2. Navigate to Project**

```cmd
cd /d C:\path\to\FluidX3D
```

**3. Configure**

```cmd
cmake -B build -G "Visual Studio 16 2019" -A x64
```

*(Use `"Visual Studio 17 2022"` for VS 2022)*

**4. Build**

```cmd
REM Build single example
cmake --build build --config Release --target benchmark

REM Build all examples
cmake --build build --config Release
```

**5. Run**

```cmd
bin\benchmark.exe
```

### Visual Studio IDE

1. **Configure:**
   ```cmd
   cmake -B build -G "Visual Studio 16 2019" -A x64
   ```

2. **Open solution:**
   ```cmd
   start build\FluidX3D.sln
   ```

3. **Build:**
   - Select `Release` configuration
   - Right-click example → **Build**

4. **Run:**
   - Executables in `bin\<name>.exe`

---

## Building on Linux

```bash
# Configure
cmake -B build

# Build specific example
cmake --build build --target benchmark

# Run
./bin/benchmark
```

---

## Building on macOS

```bash
# Configure
cmake -B build

# Build specific example
cmake --build build --target benchmark

# Run
./bin/benchmark
```

---

## Build Targets

### List All Examples

```bash
cmake --build build --target help | grep -E "^\.\.\."
```

### Build Specific Example

```bash
# Linux/macOS
cmake --build build --target taylor_green_3d

# Windows
cmake --build build --config Release --target taylor_green_3d
```

### Build All Examples

```bash
# Linux/macOS
cmake --build build -j$(nproc)

# Windows
cmake --build build --config Release --parallel
```

### Example Names

See [EXAMPLES.md](EXAMPLES.md) for the complete list of 39 examples.

Common examples:
- [`benchmark`](examples/benchmark/) - Performance benchmark
- [`taylor_green_3d`](examples/taylor_green_3d/) - 3D Taylor-Green vortex
- [`karman_vortex_street`](examples/karman_vortex_street/) - 2D Kármán vortex street
- [`cow`](examples/cow/) - Aerodynamics of a cow (requires STL file)
- [`nasa_crm`](examples/nasa_crm/) - NASA Common Research Model

---

## Executable Locations

All executables are placed in the `bin/` directory:

```
# Linux/macOS
./bin/<example_name>

# Windows
bin\<example_name>.exe
```

**Examples:**
```bash
# Linux/macOS
./bin/benchmark
./bin/cow
./bin/taylor_green_3d

# Windows
bin\benchmark.exe
bin\cow.exe
bin\taylor_green_3d.exe
```

---

## Compiler Optimization

The build system automatically applies optimal flags:

| Compiler | Flags |
|----------|-------|
| **GCC/Clang** | `-O3 -pthread -Wno-comment` |
| **MSVC** | `/O2 /MP /Ot /GL /fp:fast /LTCG` |

---

## Troubleshooting

### Windows: Compiler not found

**Problem:** `where cl.exe` returns "INFO: Could not find files"

**Solution:** Use **"x64 Native Tools Command Prompt for VS 2019"** instead of regular `cmd.exe`

### Windows: Resource files not found

**Problem:** Example says `Resource not found: skybox8k.png` or STL file missing

**Solution:**
1. Ensure files are in `resources/` directory
2. For STL files: `python resources/download_all_thingiverse_stl.py`
3. Verify: `dir resources`

### Windows: OpenCL.lib not found

**Problem:** Linker cannot find OpenCL.lib

**Solution:**
1. Install GPU drivers (includes OpenCL runtime)
2. Verify `third_party/OpenCL/lib/OpenCL.lib` exists
3. Clean rebuild: `rmdir /s /q build && cmake -B build -G "Visual Studio 16 2019" -A x64`

### Linux: X11 not found

**Problem:** Cannot find X11 development libraries

**Solution:**
```bash
# Ubuntu/Debian
sudo apt install libx11-dev libxrandr-dev

# Fedora/RHEL
sudo dnf install libX11-devel libXrandr-devel
```

### Linux: OpenCL runtime not installed

**Problem:** Runtime error about OpenCL

**Solution:** Install OpenCL runtime:
```bash
# Intel CPU Runtime (for CPUs without GPU)
# Download from: https://www.intel.com/content/www/us/en/developer/articles/technical/intel-cpu-runtime-for-opencl-applications-with-sycl-support.html

# Or install PoCL (Portable Computing Language)
sudo apt install pocl-opencl-icd  # Ubuntu/Debian
```

### macOS: OpenCL deprecated warnings

**Problem:** Warnings about OpenCL being deprecated

**Solution:** These are warnings only. macOS still supports OpenCL. Build will succeed.

### CMake version too old

**Problem:** CMake version < 3.20

**Solution:**
```bash
# Ubuntu: Get newer CMake from Kitware
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc | sudo apt-key add -
sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ focal main'
sudo apt update && sudo apt install cmake
```

### Build fails with "error C2039" (Windows)

**Problem:** Compiler errors about missing C++17 features

**Solution:**
- Ensure **MSVC v142 (Visual Studio 2019)** or newer
- C++17 should be automatic with CMake 3.20+

### Configuration changes not applied

**Problem:** Changed `defines.hpp` but example behavior unchanged

**Solution:** Clean rebuild:
```bash
# Linux/macOS
rm -rf build && cmake -B build && cmake --build build --target <example>

# Windows
rmdir /s /q build && cmake -B build -G "Visual Studio 16 2019" -A x64 && cmake --build build --config Release --target <example>
```

---

## Parallel Builds

CMake automatically uses all CPU cores:

```bash
# Linux/macOS
cmake --build build -j$(nproc)

# Windows
cmake --build build --config Release --parallel
```

---

## Additional Resources

- **[CMAKE.md](CMAKE.md)** - Architecture and technical details
- **[EXAMPLES.md](EXAMPLES.md)** - Complete list of examples
- **[DOCUMENTATION.md](DOCUMENTATION.md)** - Original FluidX3D documentation
