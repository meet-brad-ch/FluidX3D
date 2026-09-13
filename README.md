# FluidX3D

The fastest and most memory efficient lattice Boltzmann CFD software, running on all GPUs and CPUs via [OpenCL](https://github.com/ProjectPhysX/OpenCL-Wrapper "OpenCL-Wrapper"). Free for non-commercial use.

---
> **📌 Original Repository:** [ProjectPhysX/FluidX3D](https://github.com/ProjectPhysX/FluidX3D) - See the original repo for benchmarks, performance data, screenshots, and detailed physics documentation.

## 🔨 About This Fork

This fork provides a **modern CMake build system** to make FluidX3D easier to build and use with IDEs like Visual Studio, VS Code, and CLion. The original build system required manual editing of source files to select examples - this fork enables building each example as a separate target with minimal configuration.

**Key improvements:**
- ✅ **Modern CMake**: Target-based build system with automatic dependency fetching
- ✅ **IDE Integration**: Full support for Visual Studio, VS Code, CLion, Qt Creator
- ✅ **Separate Examples**: Each of the 41 examples builds independently; its CMakeLists.txt names its extensions
- ✅ **Unity Build Architecture**: Examples can have different feature configurations simultaneously
- ✅ **Resource Management**: Centralized `resources/` directory with `get_resource_path()` function
- ✅ **Easy Build Selection**: `cmake --build build --target <example>` instead of editing source code
- ✅ **SDF File Support**: Load signed distance fields from binary `.sdf` files for mesh voxelization
- ✅ **SDF Caching**: Automatic hash-based caching of generated SDF files via `sdf_cache` module
- ✅ **Setup API**: A `Simulation` in physical units: domain sizing, STL loading, voxelization, boundaries, graphics and a run loop in simulated time
- ✅ **Voxelization Fix**: Fixed vertical slice artifacts in mesh voxelization via ray jitter

**New Documentation:**
- **[CMAKE.md](CMAKE.md)** - Architecture, unity build design, and how to add examples
- **[BUILD.md](BUILD.md)** - Build instructions for Windows, Linux, and macOS
- **[EXAMPLES.md](EXAMPLES.md)** - Complete list of all 41 examples with STL download info
- **[SETUP_API.md](SETUP_API.md)** - How to write a simulation with the Setup API; **[lib/setup/README.md](lib/setup/README.md)** - its layout, conventions and tests (`ctest -L "unit|baseline|physics"`)


## How to get started?

Read the [FluidX3D Documentation](DOCUMENTATION.md)!

---

## Setup API

This fork provides a high-level **Setup API** (`#include "setup/setup.hpp"`) for configuring simulations in SI units (meters, m/s, kg/m³) with fluent method chaining. Every example except `benchmark` (a speed test of the core) uses it; `poiseuille_flow` and `stokes_drag` compare the flow with analytic solutions in SI units. [SETUP_API.md](SETUP_API.md) shows how to write a simulation with it, [lib/setup/README.md](lib/setup/README.md) describes its layout and conventions.

| Component | Purpose |
|-----------|---------|
| `Domain`, `Model` | The domain in physical units: `Domain::box(1.0_m, 5.0_m, 0.75_m)` or `Domain::around(Model("hill.stl")).clearances(...)`, with `.vram(2000_mb)` or `.cell_size(8_m)`; a model is an STL or an SDF file |
| `Simulation` | The simulation: the domain, the fluid and the reference speed set the units; gravity, surface tension, temperatures and particles are set before its first use; it owns the core's LBM (created on first use, the model voxelized, an STL converted to a cached SDF), hands out the builders and components below, and runs for a simulated time with tasks every interval |
| `Shape` | Regions in metres for objects, water and gas: sphere, cylinder, box, triangle, torus, combined with `!`, `&` and `\|` |
| `BoundaryBuilder` | Solid, open and periodic faces, solid and moving shapes, initial velocity and pressure (uniform or a field in metres), force field, wind profile, lid-driven cavity |
| `SurfaceBuilder`, `WaveBoundary` | Free surface in metres and m/s: water level and shapes (columns, drops) with a velocity, gas bubbles, solid objects, inflows, outflows and drains; oscillating wave maker (SURFACE) |
| `ThermalBuilder`, `TemperatureScale` | Hot and cold walls in Kelvin, hydrostatic and perturbed start; the lattice temperatures and buoyancy of a temperature range (TEMPERATURE) |
| `MovingPartsManager`, `MovingPart` | Rotating and tumbling parts, re-voxelized while the simulation runs (`sim.parts()`) |
| `ParticleManager` | Particle seeding in m (PARTICLES, `sim.particles()`) |
| `ForceAnalyzer` | Force in N and drag coefficient on the measured solids (FORCE_FIELD, `sim.forces()`) |
| `GraphicsConfig`, `VideoRecorder`, `CameraView` | Visualization modes (`sim.graphics()`); a video from several cameras, placed in metres and degrees, fixed or moving (`sim.video()`, written by `run_for()` when built for video) |

```cpp
#include "setup/setup.hpp"

void main_setup() { // extensions (the example's CMakeLists.txt): FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS
    const Speed flow_velocity = 1.0_mps;
    const Length cow_length = 2.4_m;
    const Length domain_length = cow_length / 0.65f; // the cow is 65 % of the domain length

    Simulation sim(Domain::around(Model("Cow_t.stl").rotation(180_deg, 0_deg, 180_deg).length(cow_length))
        .size(0.5f * domain_length, domain_length, 0.5f * domain_length)
        .gap_to_inlet(0.1f * cow_length) // the cow's nose
        .on_floor()                      // its hooves one cell above z = 0, on the solid floor
        .vram(1000_mb),                  // resolution from the VRAM budget
        Fluid::AIR, flow_velocity, LatticeMach(0.13f)); // the units; optional: how compressible the flow is simulated

    sim.boundaries()
        .set_solid_floor()
        .set_open_boundaries()
        .initialize_velocity_y(flow_velocity)
        .apply();

    sim.graphics()
        .show_surface()
        .show_vortices()
        .apply();

    sim.video()
        .add(CameraView::orbit(-40_deg, 20_deg).field_of_view(78_deg).view_height(domain_length / 1.25f))
        .set_length(10.0_s);
    sim.run_for(10.0_s); // 10 s of simulated time; writes the video when built with GRAPHICS but not INTERACTIVE_GRAPHICS
}
```
CMake: `add_fluidx3d_example(NAME cow EXTENSIONS FP16S EQUILIBRIUM_BOUNDARIES SUBGRID INTERACTIVE_GRAPHICS)`.
