# FluidX3D-API

FluidX3D-API is a modified version of [FluidX3D](https://github.com/ProjectPhysX/FluidX3D) by Dr. Moritz Lehmann. It is not the original software. FluidX3D is lattice Boltzmann CFD software that runs on all GPUs and CPUs through [OpenCL](https://github.com/ProjectPhysX/OpenCL-Wrapper "OpenCL-Wrapper").

FluidX3D-API adds a C++ API for simulations in SI units, a CMake build, SDF voxelization and a model for atmospheric convection. The lattice Boltzmann core is the core of FluidX3D.

> **Original software:** [ProjectPhysX/FluidX3D](https://github.com/ProjectPhysX/FluidX3D). Go there for benchmarks, performance data, screenshots and the physics documentation.
>
> **License:** the [FluidX3D license](LICENSE.md) applies without change. Use for public research, education or personal use only; no commercial or military use. If a scientific publication comes from this software, cite the [FluidX3D references](https://github.com/ProjectPhysX/FluidX3D#references).

## What FluidX3D-API adds

The CMake build makes FluidX3D easier to build and use with IDEs like Visual Studio, VS Code and CLion. The original build system requires you to edit source files to select an example. Here, each example is a separate build target.

**Key improvements:**

- ✅ **Modern CMake**: Target-based build system with automatic dependency fetching
- ✅ **IDE Integration**: Full support for Visual Studio, VS Code, CLion, Qt Creator
- ✅ **Separate Examples**: Each of the 42 examples builds independently; its CMakeLists.txt names its extensions
- ✅ **Unity Build Architecture**: Examples can have different feature configurations simultaneously
- ✅ **Resource Management**: Centralized `resources/` directory with `get_resource_path()` function
- ✅ **Easy Build Selection**: `cmake --build build --target <example>` instead of editing source code
- ✅ **SDF File Support**: Load signed distance fields from binary `.sdf` files for mesh voxelization
- ✅ **SDF Caching**: Automatic hash-based caching of generated SDF files via `sdf_cache` module
- ✅ **Setup API**: A `Simulation` in physical units: domain sizing, STL loading, voxelization, boundaries, graphics and a run loop in simulated time
- ✅ **Voxelization Fix**: Fixed vertical slice artifacts in mesh voxelization via ray jitter
- ✅ **Atmospheric Convection**: A validated thermal model for the convective boundary layer (the `alpha / 2` diffusivity fix, surface heat flux, eddy heat diffusivity, wind nudging, a sponge layer, moisture with condensation) and a conservative finite-volume transport of heat and water by the LBM's own link mass fluxes, see [research/convection/README.md](research/convection/README.md)

**New Documentation:**

- **[CMAKE.md](CMAKE.md)** - Architecture, unity build design, how to add examples, applications outside the repository and Python modules
- **[BUILD.md](BUILD.md)** - Build instructions for Windows, Linux, and macOS
- **[EXAMPLES.md](EXAMPLES.md)** - Complete list of all 42 examples with STL download info
- **[research/convection/README.md](research/convection/README.md)** - Atmospheric convection in FluidX3D: the features and how to use them, their limits, the validation, the references; **[FORK_CHANGES.md](FORK_CHANGES.md)** - every change of FluidX3D-API with its measurement
- **[SETUP_API.md](SETUP_API.md)** - How to write a simulation with the Setup API; **[lib/setup/README.md](lib/setup/README.md)** - its layout, conventions and tests (`ctest -L "unit|physics"`)

## How to get started?

Read the [FluidX3D Documentation](DOCUMENTATION.md)!

---

## Setup API

FluidX3D-API provides a high-level **Setup API** (`#include "setup/setup.hpp"`) for configuring simulations in SI units with fluent method chaining. Its values are quantities of the Au units library (`2.4 * m`, `10.0 * m / s`), checked by the compiler. Every example except `benchmark` (a speed test of the core) uses it; `poiseuille_flow` and `stokes_drag` compare the flow with analytic solutions in SI units. [SETUP_API.md](SETUP_API.md) shows how to write a simulation with it, [lib/setup/README.md](lib/setup/README.md) describes its layout and conventions.

| Component | Purpose |
|-----------|---------|
| `Domain`, `Model` | The domain in physical units: `Domain::box(1.0 * m, 5.0 * m, 0.75 * m)` or `Domain::around(Model("hill.stl")).clearances(...)`, with `.vram(2000.0 * MiB)` or `.cell_size(8.0 * m)`; a model is an STL or an SDF file |
| `Simulation` | The simulation: the domain, the fluid and the reference speed set the units; gravity, surface tension, temperatures and particles are set before its first use; it owns the core's LBM (created on first use, the model voxelized, an STL converted to a cached SDF), hands out the builders and components below, and runs for a simulated time with tasks every interval |
| `Shape` | Regions in metres for objects, water and gas: sphere, cylinder, box, triangle, torus, combined with `!`, `&` and `\|` |
| `BoundaryBuilder` | Solid, open and periodic faces, solid and moving shapes, initial velocity and pressure (uniform or a field in metres), force field, wind profile, lid-driven cavity |
| `SurfaceBuilder`, `WaveBoundary` | Free surface in metres and m/s: water level and shapes (columns, drops) with a velocity, gas bubbles, solid objects, inflows, outflows and drains; oscillating wave maker (SURFACE) |
| `ThermalBuilder` | Hot and cold walls in kelvin, hydrostatic and perturbed start; the lattice temperatures and the buoyancy come from the simulation's temperature range (TEMPERATURE) |
| `MovingPartsManager`, `MovingPart` | Rotating and tumbling parts, re-voxelized while the simulation runs (`sim.parts()`) |
| `ParticleManager` | Particle seeding in m (PARTICLES, `sim.particles()`) |
| `ForceAnalyzer` | Force in N and drag coefficient on the measured solids (FORCE_FIELD, `sim.forces()`) |
| `si::LatticeConverter` | The one conversion between the quantities and the core's lattice numbers (`sim.converter()`) |
| `GraphicsConfig`, `VideoRecorder`, `CameraView` | Visualization modes (`sim.graphics()`); a video from several cameras, placed in metres and degrees, fixed or moving (`sim.video()`, written by `run_for()` when built for video) |

```cpp
#include "setup/setup.hpp"

void main_setup() { // extensions (the example's CMakeLists.txt): FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS
    using namespace si::symbols;
    const si::Speed flow_velocity = 1.0 * m / s;
    const si::Length cow_length = 2.4 * m;
    const si::Length domain_length = cow_length / 0.65; // the cow is 65 % of the domain length

    Simulation sim(Domain::around(Model("Cow_t.stl").rotation(180.0 * deg, 0.0 * deg, 180.0 * deg).length(cow_length))
        .size(0.5 * domain_length, domain_length, 0.5 * domain_length)
        .gap_to_inlet(0.1 * cow_length) // the cow's nose
        .on_floor()                     // its hooves one cell above z = 0, on the solid floor
        .vram(1000.0 * MiB),            // resolution from the VRAM budget
        Fluid::AIR, flow_velocity, si::LatticeMach(0.13)); // the units; optional: how compressible the flow is simulated

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
        .add(CameraView::orbit(-40.0 * deg, 20.0 * deg).field_of_view(78.0 * deg).view_height(domain_length / 1.25))
        .set_length(10.0 * s);
    sim.run_for(10.0 * s); // 10 s of simulated time; writes the video when built with GRAPHICS but not INTERACTIVE_GRAPHICS
}
```

CMake: `add_fluidx3d_example(NAME cow EXTENSIONS FP16S EQUILIBRIUM_BOUNDARIES SUBGRID INTERACTIVE_GRAPHICS)`.
