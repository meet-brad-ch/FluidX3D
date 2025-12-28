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

This fork provides a high-level **Setup API** that simplifies simulation configuration using SI units (meters, m/s, kg/m³). The API uses fluent method chaining for readable configuration.

### Components

| Component | Purpose |
|-----------|---------|
| `SimulationConfig` | Geometry filename, VRAM budget, rotation, clearances |
| `SimulationSetup` | Domain sizing, unit conversion, voxelization |
| `SDFGenerator` | SDF generation with hash-based caching |
| `BoundaryBuilder` | Boundary conditions, forces, velocity initialization |
| `ForceAnalyzer` | Aerodynamic force and coefficient calculation |
| `GraphicsConfig` | Visualization modes and camera settings |

### Quick Example

```cpp
#include "setup/simulation_setup.hpp"
#include "setup/sdf_generator.hpp"
#include "setup/boundary_builder.hpp"
#include "setup/force_analyzer.hpp"
#include "setup/graphics_config.hpp"

void main_setup() {
    // 1. Configure geometry (SI units)
    SimulationSetup sim(SimulationConfig("aircraft.stl")
        .set_vram(4000u)                           // 4 GB VRAM budget
        .set_rotation(0.0f, 0.0f, 90.0f)           // Rotate 90° around Z
        .set_clearances_m(2.0f, 100.0f, 50.0f));   // Bottom, top, side clearances in meters

    // 2. Calculate domain size
    auto results = sim.setup();

    // 3. Optionally generate SDF (cached automatically)
    SDFGenerator sdf;
    std::string sdf_path = sdf.generate(sim.get_stl_path(),
        results.base_grid.x, results.base_grid.y, results.base_grid.z);
    sim.use_sdf(sdf_path);

    // 4. Configure units (SI velocity and density)
    sim.configure_units(/*si_velocity=*/60.0f, /*si_density=*/1.225f);

    // 5. Create LBM simulation
    LBM lbm(results.Nx, results.Ny, results.Nz, sim.to_lbm_viscosity(1.48e-5f));

    // 6. Configure boundaries
    BoundaryBuilder(lbm)
        .set_solid_floor()
        .set_equilibrium_all_faces()
        .initialize_velocity(0.0f, sim.to_lbm_velocity(60.0f), 0.0f)
        .apply();

    // 7. Voxelize geometry
    sim.enable_force_tracking();  // Enable for Cd/Cl calculation
    sim.voxelize(lbm);

    // 8. Configure graphics
    GraphicsConfig(lbm)
        .show_surface()
        .show_vortices()
        .set_camera_isometric()
        .apply();

    // 9. Set up force analyzer
    ForceAnalyzer forces(lbm);
    forces.set_reference_area(10.0f)       // m²
          .set_reference_velocity(60.0f)   // m/s
          .set_flow_direction(Axis::Y);

    // 10. Run simulation
    lbm.run(0u);
    while(lbm.get_t() <= sim.to_lbm_timesteps(5.0f)) {
        lbm.run(1u);
        forces.print_coefficients();  // Prints Cd, Cl
    }
}
```

### SimulationConfig

Configure geometry file, VRAM budget, positioning, and clearances:

```cpp
SimulationConfig config("mesh.stl")
    .set_vram(4000u)                        // VRAM budget in MB
    .set_voxel_size_m(10.0f)                // OR: set voxel size in meters
    .set_rotation(0.0f, 90.0f, 0.0f)        // X, Y, Z rotation in degrees
    .set_offset(0.0f, 0.0f, 5.0f)           // X, Y, Z offset in meters
    .set_clearances_m(2.0f, 100.0f, 50.0f)  // Bottom, top, side in meters
    .set_reference_axis(SimulationConfig::ReferenceAxis::Y);
```

### SimulationSetup

Main setup class for domain calculation and unit conversion:

```cpp
SimulationSetup sim(config);
auto results = sim.setup();              // Calculate domain size

sim.configure_units(15.0f, 1.225f);      // SI velocity (m/s), density (kg/m³)
sim.enable_force_tracking();             // Enable Cd/Cl calculation

LBM lbm(results.Nx, results.Ny, results.Nz, sim.to_lbm_viscosity(1.48e-5f));
sim.voxelize(lbm);                       // Voxelize geometry

// Unit conversions
float lbm_vel = sim.to_lbm_velocity(15.0f);    // SI → LBM velocity
float si_force = sim.to_si_force(lbm_force);   // LBM → SI force
ulong steps = sim.to_lbm_timesteps(10.0f);     // Seconds → timesteps
```

### SDFGenerator

Generate and cache signed distance fields:

```cpp
SDFGenerator sdf;
sdf.set_cache_dir("resources/sdf_cache/")
   .enable_cache(true)
   .set_verbose(true);

std::string sdf_path = sdf.generate("mesh.stl", nx, ny, nz);
sim.use_sdf(sdf_path);  // Use generated SDF
```

### BoundaryBuilder

Configure boundaries, forces, and velocity:

```cpp
BoundaryBuilder(lbm)
    // Solid boundaries
    .set_solid_floor()
    .set_solid_box()

    // Equilibrium (inflow/outflow) boundaries
    .set_equilibrium_all_faces()
    .set_equilibrium_face(Face::Y_MAX)

    // Velocity initialization
    .initialize_velocity(ux, uy, uz)

    // Volume forces
    .set_gravity(9.81f, 1000.0f)               // Gravity with density
    .initialize_hydrostatic_pressure(10.0f)    // Fluid height in meters
    .set_wind_profile_power_law(15.0f, 10.0f)  // Reference velocity, height

    .apply();
```

### ForceAnalyzer

Calculate aerodynamic forces and coefficients:

```cpp
ForceAnalyzer forces(lbm);
forces.set_reference_area(0.5f)        // Frontal area in m²
      .set_reference_velocity(60.0f)   // Freestream velocity in m/s
      .set_fluid_density(1.225f)       // Air density in kg/m³
      .set_flow_direction(Axis::Y);    // Flow along Y axis

// After simulation steps
float3 force_si = forces.get_force_si();    // Force in Newtons
float Cd = forces.get_drag_coefficient();   // Drag coefficient
float Cl = forces.get_lift_coefficient();   // Lift coefficient

forces.print_forces();        // Print to console
forces.print_coefficients();  // Print Cd, Cl
forces.log_to_file();         // Log to file
```

### GraphicsConfig

Configure visualization modes and camera:

```cpp
GraphicsConfig(lbm)
    // Visualization modes
    .show_surface()           // Solid surfaces
    .show_vortices()          // Q-criterion
    .show_velocity_field()    // Velocity magnitude
    .show_streamlines()       // Streamlines
    .show_free_surface()      // Free surface (SURFACE extension)

    // Slicing
    .slice_x(0.5f)            // YZ plane at 50%
    .slice_y(0.5f)            // XZ plane at 50%
    .slice_z(0.5f)            // XY plane at 50%

    // Camera presets
    .set_camera_isometric()
    .set_camera_top_view()
    .set_camera_side_view()
    .set_camera(-40.0f, 20.0f, 1.25f)  // Custom: pitch, yaw, zoom

    .apply();
```
