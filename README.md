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
- ✅ **SDF Caching**: Automatic hash-based caching of generated SDF files via `sdf_cache` module
- ✅ **SimulationSetup API**: Fluent configuration API for domain sizing, STL loading, and voxelization
- ✅ **Voxelization Fix**: Fixed vertical slice artifacts in mesh voxelization via ray jitter

**New Documentation:**
- **[CMAKE.md](CMAKE.md)** - Architecture, unity build design, and how to add examples
- **[BUILD.md](BUILD.md)** - Build instructions for Windows, Linux, and macOS
- **[EXAMPLES.md](EXAMPLES.md)** - Complete list of all 40 examples with STL download info


## How to get started?

Read the [FluidX3D Documentation](DOCUMENTATION.md)!

---

## Setup API

This fork provides a high-level **Setup API** (`#include "setup/setup.hpp"`) for configuring simulations in SI units (meters, m/s, kg/m³) with fluent method chaining. The examples show every feature in use; [porting_guide.md](porting_guide.md) maps the low-level calls to the API.

| Component | Purpose |
|-----------|---------|
| `Domain`, `Model` | The domain in physical units: `Domain::box(1.0_m, 5.0_m, 0.75_m)` or `Domain::around(Model("hill.stl")).clearances(...)`, with `.vram(2000_mb)` or `.cell_size(8_m)` |
| `SimulationConfig` | Aspect-ratio domains (being replaced by `Domain`): aspect ratio, geometry scale, offsets, rotation |
| `SimulationSetup` | Domain sizing, unit conversion, LBM creation (plain, thermal, free surface, particles), voxelization (STL converted to a cached SDF) |
| `BoundaryBuilder` | Solid and open faces, initial velocity, wind profile, lid-driven cavity |
| `SurfaceBuilder`, `WaveBoundary` | Free surface regions, solid blocks, inlets and outlets; oscillating wave maker (SURFACE) |
| `ThermalBuilder` | Hot and cold walls in Kelvin, hydrostatic and perturbed start (TEMPERATURE) |
| `MovingPartsManager`, `MovingPart` | Rotating and tumbling parts, re-voxelized while the simulation runs |
| `ParticleManager` | Particle seeding in m (PARTICLES) |
| `ForceAnalyzer` | Force in N and drag coefficient on the tracked object (FORCE_FIELD) |
| `GraphicsConfig`, `VideoRecorder`, `CameraConfig` | Visualization modes; video frames from several cameras |

```cpp
#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() { // required extensions in defines.hpp: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS
    const float flow_velocity_mps = 1.0f;

    SimulationSetup sim(SimulationConfig("Cow_t.stl")
        .set_domain_aspect_ratio(1.0f, 2.0f, 1.0f) // domain proportions
        .set_vram_mb(1000u)                        // resolution from the VRAM budget
        .set_geometry_scale(0.65f)                 // cow length: 65% of the domain length (Y)
        .set_rotation_deg(180.0f, 0.0f, 180.0f)
        .set_pmin_offset_ratio(0.0f, 0.1f, 0.006f)); // gap to the inlet and to the floor, in cow lengths

    sim.setup();
    sim.configure_units_with_length(2.4f, flow_velocity_mps, Fluid::AIR, 0.075f); // cow length 2.4 m
    sim.print_reynolds_number(Fluid::AIR);

    LBM lbm = sim.create_lbm(Fluid::AIR);
    sim.voxelize(lbm);

    BoundaryBuilder(lbm)
        .set_solid_floor()
        .set_open_boundaries()
        .initialize_velocity_y(flow_velocity_mps)
        .apply();

    GraphicsConfig(lbm)
        .show_surface()
        .show_vortices()
        .apply();

    lbm.run();
}
```
