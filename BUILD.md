# Build Instructions

How to build FluidX3D on Windows, Linux, and macOS.

## Cloning the Repository

This project uses git submodules. Clone with:

```bash
git clone --recurse-submodules https://github.com/meet-brad-ch/FluidX3D
```

If you already cloned without submodules:

```bash
git submodule update --init
```

---

## Quick Start

**Important:** Run scripts directly from the shell. Do NOT prefix with `cmd.exe` or use `cmd //c`.

```bash
# Windows (Git Bash, PowerShell, or Developer Command Prompt)
cd tools
./configure.bat Release
./build.bat benchmark Release

# Linux/macOS
cd tools
./configure.sh Release
./build.sh benchmark

# Run
../bin/benchmark
```

---

## Prerequisites

### All Platforms
- **CMake 3.20+**
- **C++20-compatible compiler** (the Setup API uses `consteval` literals and designated initializers)
- **OpenCL Runtime** - Install GPU drivers or [Intel CPU Runtime](https://www.intel.com/content/www/us/en/developer/articles/technical/intel-cpu-runtime-for-opencl-applications-with-sycl-support.html)

### Windows
- **Visual Studio 2019 16.10 or later (2022 recommended)** with Desktop C++ development
- **Python 3** (optional, for STL downloads)

### Linux
- **g++ 10+** (C++20 support)
- X11 libraries (usually pre-installed)

### macOS
- **Xcode Command Line Tools**
- X11 libraries

---

## Building on Windows

**1. Open any terminal** (Git Bash, PowerShell, or Command Prompt)

**2. Navigate to tools directory**

```bash
cd tools
```

**3. Configure and Build**

```bash
./configure.bat Release
./build.bat benchmark Release
```

**4. Run**

```bash
../bin/benchmark.exe
```

---

## Building on Linux

```bash
cd tools
./configure.sh Release
./build.sh benchmark
../bin/benchmark
```

---

## Building on macOS

```bash
cd tools
./configure.sh Release
./build.sh benchmark
../bin/benchmark
```

---

## Build Targets

### Build Specific Example

```bash
cd tools

# Windows
./build.bat taylor_green_3d Release

# Linux/macOS
./build.sh taylor_green_3d
```

### Build All Examples

```bash
cd tools

# Windows
./build.bat all Release

# Linux/macOS
./build.sh all
```

### Verifying API Changes

When modifying shared code (e.g., `Simulation`, `MovingPartsManager`, `BoundaryBuilder`),
build all examples to verify nothing is broken:

```bash
cd tools
./build.bat all Release   # Windows
./build.sh all            # Linux/macOS
```

This builds all examples that depend on the modified API and catches compilation errors early.
Then run the tests (from the build directory, e.g. `build-Release`):

```bash
ctest -L unit                            # Setup API unit tests (CPU only)
ctest -L baseline                        # every example's setup state against tests/baselines/
ctest -L physics                         # simulations against analytic solutions (Poiseuille, Stokes, Taylor-Green, hydrostatic)
FLUIDX3D_BLESS=1 ctest -L baseline       # after an intended change: record the new baselines
```

`FLUIDX3D_BASELINE_STEPS=N ../bin/<example>_baseline` runs an example's own loop for N time steps and reports the largest velocity, to check that a changed setup stays stable. See [CMAKE.md](CMAKE.md#tests).

### Example Names

See [EXAMPLES.md](EXAMPLES.md) for the complete list of examples.

Common examples:
- [`benchmark`](examples/benchmark/) - Performance benchmark
- [`taylor_green_3d`](examples/taylor_green_3d/) - 3D Taylor-Green vortex
- [`karman_vortex_street`](examples/karman_vortex_street/) - 2D Kármán vortex street
- [`cow`](examples/cow/) - Aerodynamics of a cow (requires STL file)
- [`nasa_crm`](examples/nasa_crm/) - NASA Common Research Model

---

## Executable Locations

All executables are placed in the `bin/` directory:

```bash
# Linux/macOS
../bin/<example_name>

# Windows
..\bin\<example_name>.exe
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

### Windows: Resource files not found

**Problem:** Example says `Resource not found: skybox8k.png` or STL file missing

**Solution:**
1. Ensure files are in `resources/` directory
2. For STL files: `python resources/download_all_thingiverse_stl.py`
3. Verify: `dir resources`

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

### Configuration changes not applied

**Problem:** Changed an example's `EXTENSIONS` (its `CMakeLists.txt`) but example behavior unchanged

**Solution:** Clean rebuild:
```bash
cd tools

# Linux/macOS
rm -rf ../build-Release
./configure.sh Release
./build.sh <example>

# Windows
rmdir /s /q ..\build-Release
./configure.bat Release
./build.bat <example> Release
```

---

## Additional Resources

- **[CMAKE.md](CMAKE.md)** - Architecture and technical details
- **[EXAMPLES.md](EXAMPLES.md)** - Complete list of examples
- **[DOCUMENTATION.md](DOCUMENTATION.md)** - Original FluidX3D documentation
