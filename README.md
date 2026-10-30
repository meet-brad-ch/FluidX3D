# FluidX3D-API

FluidX3D-API is a modified version of [FluidX3D](https://github.com/ProjectPhysX/FluidX3D) by Dr. Moritz Lehmann. It
is not the original software. FluidX3D-API adds a C++ API for simulations in SI units, a CMake build, SDF voxelization
and a model for atmospheric convection. The lattice Boltzmann core is the core of FluidX3D.

> **Original software:** [ProjectPhysX/FluidX3D](https://github.com/ProjectPhysX/FluidX3D). This README tells only
> what FluidX3D-API adds. For all other information, go to the original repository: what FluidX3D is, the GPU and
> CPU drivers, the hardware, the benchmarks, the physics, the original documentation and the references.
>
> **License:** the [FluidX3D license](LICENSE.md) applies without change. Use for public research, education or
> personal use only. No commercial or military use. If a scientific publication comes from this software, cite the
> [FluidX3D references](https://github.com/ProjectPhysX/FluidX3D#references).

## What FluidX3D-API adds

- **A CMake build.** Each of the 42 examples is a separate build target, with its extensions in its own
  `CMakeLists.txt`. You select an example with `cmake --build build --target <example>`, not by an edit of the source
  code. Examples with different extensions build in one build tree. CMake fetches the dependencies. The build works
  with Visual Studio, VS Code, CLion and Qt Creator.
- **The Setup API.** A `Simulation` in physical units: the domain size, STL loading, voxelization, boundaries, graphics
  and a run loop in simulated time. See [Setup API](#setup-api) below.
- **SDF files.** Voxelization from binary `.sdf` files, and a cache that keeps the SDF of each STL file (`sdf_cache`).
- **Resources.** One `resources/` folder and the function `get_resource_path()`.
- **Atmospheric convection.** A thermal model for the convective boundary layer: the fix of the `alpha / 2`
  diffusivity, a surface heat flux, eddy heat diffusivity, wind nudging, a sponge layer and moisture with condensation.
  Heat and water can move by conservative finite volumes, driven by the link mass fluxes of the LBM.
- **A voxelization fix.** Ray jitter removes the vertical slice artifacts of mesh voxelization.
- **Python modules.** FluidX3D applications as Python extension modules (nanobind), also from outside the repository.

## Documentation of FluidX3D-API

| Document | What it tells |
|---|---|
| [BUILD.md](BUILD.md) | How to build on Windows, Linux and macOS |
| [CMAKE.md](CMAKE.md) | The build architecture, the unity build, how to add examples, applications outside the repository, Python modules |
| [EXAMPLES.md](EXAMPLES.md) | All 42 examples, their extensions and the STL files they need |
| [SETUP_API.md](SETUP_API.md) | How to write a simulation with the Setup API |
| [lib/setup/README.md](lib/setup/README.md) | The layout, the conventions and the tests of the Setup API (`ctest -L "unit\|physics"`) |
| [research/convection/README.md](research/convection/README.md) | Atmospheric convection: the features, how to use them, their limits, the validation, the references |
| [FORK_CHANGES.md](FORK_CHANGES.md) | Each change of FluidX3D-API, with its measurement |
| [upstream_issues.md](upstream_issues.md) | Problems found in FluidX3D, to report to the original project |
| [CHANGELOG.md](CHANGELOG.md) | The changes in each release |

## Get started

1. Install the OpenCL drivers as the [original documentation](https://github.com/ProjectPhysX/FluidX3D/blob/master/DOCUMENTATION.md) tells.
2. Clone this repository with its submodule: `git clone --recurse-submodules https://github.com/meet-brad-ch/FluidX3D-API.git`.
3. Build an example as [BUILD.md](BUILD.md) tells, for example `cmake --build build --target benchmark`.

## Setup API

The **Setup API** (`#include "setup/setup.hpp"`) configures a simulation in SI units, with method chaining. Its values
are quantities of the Au units library (`2.4 * m`, `10.0 * m / s`), and the compiler checks them. Each example except
`benchmark` (a speed test of the core) uses it. `poiseuille_flow` and `stokes_drag` compare the flow with analytic
solutions in SI units. [SETUP_API.md](SETUP_API.md) shows how to write a simulation with it.
[lib/setup/README.md](lib/setup/README.md) describes its layout and conventions.

| Component | Purpose |
|-----------|---------|
| `Domain`, `Model` | The domain in physical units: `Domain::box(1.0 * m, 5.0 * m, 0.75 * m)` or `Domain::around(Model("hill.stl")).clearances(...)`, with `.vram(2000.0 * MiB)` or `.cell_size(8.0 * m)`. A model is an STL or an SDF file |
| `Simulation` | The simulation. The domain, the fluid and the reference speed set the units. Gravity, surface tension, temperatures and particles are set before its first use. It owns the core's LBM (made at first use: the model is voxelized, an STL becomes a cached SDF). It gives the builders and components below, and runs for a simulated time with tasks at each interval |
| `Shape` | Regions in metres for objects, water and gas: sphere, cylinder, box, triangle, torus, combined with `!`, `&` and `\|` |
| `BoundaryBuilder` | Solid, open and periodic faces, solid and moving shapes, initial velocity and pressure (uniform or a field in metres), force field, wind profile, lid-driven cavity |
| `SurfaceBuilder`, `WaveBoundary` | Free surface in metres and m/s: water level and shapes (columns, drops) with a velocity, gas bubbles, solid objects, inflows, outflows and drains. An oscillating wave maker (SURFACE) |
| `ThermalBuilder` | Hot and cold walls in kelvin, hydrostatic and perturbed start. The lattice temperatures and the buoyancy come from the simulation's temperature range (TEMPERATURE) |
| `MovingPartsManager`, `MovingPart` | Rotating and tumbling parts, voxelized again while the simulation runs (`sim.parts()`) |
| `ParticleManager` | Particle seeding in m (PARTICLES, `sim.particles()`) |
| `ForceAnalyzer` | Force in N and drag coefficient on the measured solids (FORCE_FIELD, `sim.forces()`) |
| `si::LatticeConverter` | The one conversion between the quantities and the core's lattice numbers (`sim.converter()`) |
| `GraphicsConfig`, `VideoRecorder`, `CameraView` | Visualization modes (`sim.graphics()`). A video from several cameras, placed in metres and degrees, fixed or moving (`sim.video()`, written by `run_for()` when built for video) |

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

## Releases

FluidX3D-API makes its own releases. A version has the form `<FluidX3D version>-api.<n>`, for example `3.8-api.1`:
the FluidX3D release that it is based on, and the number of the FluidX3D-API release on that base.
[CHANGELOG.md](CHANGELOG.md) and the [releases page](https://github.com/meet-brad-ch/FluidX3D-API/releases) tell what
each release changes. To use a release, check out its tag, for example `git checkout v3.8-api.1`.

To make a release:

1. Add a section `## [<version>]` to CHANGELOG.md.
2. Run `scripts/release.sh <version>`. It checks the version and the changelog, and makes the tag.
3. Push the tag: `git push origin v<version>`. The workflow `release-api.yml` makes the GitHub release from the
   changelog section. The release has no files to download.
