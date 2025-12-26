# FluidX3D

The fastest and most memory efficient lattice Boltzmann CFD software, running on all GPUs and CPUs via [OpenCL](https://github.com/ProjectPhysX/OpenCL-Wrapper "OpenCL-Wrapper"). Free for non-commercial use.

---
> **📌 Original Repository:** [ProjectPhysX/FluidX3D](https://github.com/ProjectPhysX/FluidX3D) - See the original repo for benchmarks, performance data, screenshots, and detailed physics documentation.

## 🔨 About This Fork

This fork provides a **modern CMake build system** to make FluidX3D easier to build and use with IDEs like Visual Studio, VS Code, and CLion. The original build system required manual editing of source files to select examples - this fork enables building each example as a separate target with minimal configuration.

**Key improvements:**
- ✅ **Modern CMake**: Target-based build system with automatic dependency fetching
- ✅ **IDE Integration**: Full support for Visual Studio, VS Code, CLion, Qt Creator
- ✅ **Separate Examples**: Each of the 40 examples builds independently with 1-line CMakeLists.txt
- ✅ **Unity Build Architecture**: Examples can have different feature configurations simultaneously
- ✅ **Resource Management**: Centralized `resources/` directory with `get_resource_path()` function
- ✅ **Easy Build Selection**: `cmake --build build --target <example>` instead of editing source code
- ✅ **SDF File Support**: Load signed distance fields from binary `.sdf` files for mesh voxelization
- ✅ **Voxelization Fix**: Fixed vertical slice artifacts in mesh voxelization via ray jitter

**New Documentation:**
- **[CMAKE.md](CMAKE.md)** - Architecture, unity build design, and how to add examples
- **[BUILD.md](BUILD.md)** - Build instructions for Windows, Linux, and macOS
- **[EXAMPLES.md](EXAMPLES.md)** - Complete list of all 40 examples with STL download info


## How to get started?

Read the [FluidX3D Documentation](DOCUMENTATION.md)!
