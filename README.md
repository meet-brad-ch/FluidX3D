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
| `SimulationConfig` | The domain planner's input behind `Domain`; used directly only by examples not yet ported (see [porting_guide.md](porting_guide.md#mapping-from-simulationconfig)) |
| `SimulationSetup` | Domain sizing, unit conversion, LBM creation (plain, thermal, free surface, particles), voxelization (STL converted to a cached SDF) |
| `BoundaryBuilder` | Solid and open faces, initial velocity, wind profile, lid-driven cavity |
| `SurfaceBuilder`, `WaveBoundary` | Free surface in metres and m/s: water level and boxes, solid blocks, inflows and outflows; oscillating wave maker (SURFACE) |
| `ThermalBuilder`, `TemperatureScale` | Hot and cold walls in Kelvin, hydrostatic and perturbed start; the lattice temperatures and buoyancy of a temperature range (TEMPERATURE) |
| `MovingPartsManager`, `MovingPart` | Rotating and tumbling parts, re-voxelized while the simulation runs |
| `ParticleManager` | Particle seeding in m (PARTICLES) |
| `ForceAnalyzer` | Force in N and drag coefficient on the tracked object (FORCE_FIELD) |
| `GraphicsConfig`, `VideoRecorder`, `CameraConfig` | Visualization modes; video frames from several cameras |

```cpp
#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() { // required extensions in defines.hpp: FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS
    const Speed flow_velocity = 1.0_mps;
    const Length cow_length = 2.4_m;
    const Length domain_length = cow_length / 0.65f; // the cow is 65 % of the domain length

    SimulationSetup sim(Domain::around(Model("Cow_t.stl").rotation(180_deg, 0_deg, 180_deg).length(cow_length))
        .size(0.5f * domain_length, domain_length, 0.5f * domain_length)
        .gap_to_inlet(0.1f * cow_length)   // the cow's nose
        .gap_to_floor(0.006f * cow_length) // about one cell
        .vram(1000_mb));                   // resolution from the VRAM budget

    sim.setup();
    sim.configure_units(flow_velocity, Fluid::AIR, 0.075f);
    sim.print_reynolds_number(Fluid::AIR);

    LBM lbm = sim.create_lbm(Fluid::AIR);
    sim.voxelize(lbm);

    BoundaryBuilder(lbm)
        .set_solid_floor()
        .set_open_boundaries()
        .initialize_velocity_y(flow_velocity)
        .apply();

    GraphicsConfig(lbm)
        .show_surface()
        .show_vortices()
        .apply();

    lbm.run();
}
```
