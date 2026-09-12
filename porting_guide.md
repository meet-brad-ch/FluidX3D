# Porting Guide: Low-Level to High-Level API

This guide explains how to port FluidX3D examples from the low-level API to the new high-level Setup API.

---

## Overview

The high-level API provides:
- SI units throughout (no manual cell calculations)
- Fluent builder pattern for configuration
- Automatic mesh loading, scaling, and positioning
- Predefined fluid properties (`Fluid::AIR`, `Fluid::WATER`, etc.)

---

## Quick Reference

| Old (Low-Level) | New (High-Level) |
|-----------------|------------------|
| `resolution(float3(1,2,1), 1000u)` | `.set_domain_aspect_ratio(1.0f, 2.0f, 1.0f).set_vram_mb(1000u)` |
| `lbm_length = 0.65f * lbm_N.y` | `.set_geometry_scale(0.65f)` |
| `units.set_m_kg_s(...)` | `sim.configure_units_with_length(...)` |
| `read_stl(...) + mesh->translate(...)` | `.set_pmin_offset_ratio(...)` |
| `lbm.voxelize_mesh_on_device(mesh)` | `sim.voxelize(lbm)` |
| `parallel_for` boundary setup | `BoundaryBuilder(lbm).set_solid_floor()...` |
| `lbm.graphics.visualization_modes = ...` | `GraphicsConfig(lbm).show_surface()...` |

---

## Complete Example: Cow Aerodynamics

### Old Low-Level API

```cpp
#include "defines.hpp"
#include "info.hpp"
#include "lbm.hpp"
#include "graphics.hpp"
#include "setup.hpp"
#include "shapes.hpp"

void main_setup() {
    // Domain and units
    const uint3 lbm_N = resolution(float3(1.0f, 2.0f, 1.0f), 1000u);
    const float si_u = 1.0f;
    const float si_length = 2.4f;
    const float si_nu = 1.48E-5f, si_rho = 1.225f;
    const float lbm_length = 0.65f * (float)lbm_N.y;
    const float lbm_u = 0.075f;
    units.set_m_kg_s(lbm_length, lbm_u, 1.0f, si_length, si_u, si_rho);
    const float lbm_nu = units.nu(si_nu);
    print_info("Re = " + to_string(to_uint(units.si_Re(si_length, si_u, si_nu))));
    LBM lbm(lbm_N, lbm_nu);

    // Geometry
    const float3x3 rotation = float3x3(float3(1, 0, 0), radians(180.0f))
                            * float3x3(float3(0, 0, 1), radians(180.0f));
    Mesh* mesh = read_stl(get_resource_path("Cow_t.stl"), lbm.size(),
                          lbm.center(), rotation, lbm_length);
    mesh->translate(float3(0.0f, 1.0f-mesh->pmin.y+0.1f*lbm_length, 1.0f-mesh->pmin.z));
    lbm.voxelize_mesh_on_device(mesh);

    // Boundaries
    const uint Nx=lbm.get_Nx(), Ny=lbm.get_Ny(), Nz=lbm.get_Nz();
    parallel_for(lbm.get_N(), [&](ulong n) {
        uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
        if(z==0u) lbm.flags[n] = TYPE_S;
        if(lbm.flags[n]!=TYPE_S) lbm.u.y[n] = lbm_u;
        if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==Nz-1u) lbm.flags[n] = TYPE_E;
    });

    // Graphics
    lbm.graphics.visualization_modes = VIS_FLAG_SURFACE|VIS_Q_CRITERION;

    lbm.run();
}
```

### New High-Level API

```cpp
#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() {
    const float32_t flow_velocity_mps = 1.0f;
    const float32_t cow_size_m = 2.4f;

    SimulationSetup sim(SimulationConfig("Cow_t.stl")
        .set_domain_aspect_ratio(1.0f, 2.0f, 1.0f)
        .set_vram_mb(1000u)
        .set_geometry_scale(0.65f)
        .set_rotation_deg(180.0f, 0.0f, 180.0f)
        .set_reference_axis(SimulationConfig::ReferenceAxis::Y)
        .set_pmin_offset_ratio(0.0f, 0.1f, 0.006f));

    sim.setup();
    sim.configure_units_with_length(cow_size_m, flow_velocity_mps, Fluid::AIR, 0.075f);
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

---

## Porting Step by Step

### 1. Domain Configuration

**Old:**
```cpp
const uint3 lbm_N = resolution(float3(1.0f, 2.0f, 1.0f), 1000u);
const float lbm_length = 0.65f * (float)lbm_N.y;
```

**New:**
```cpp
SimulationConfig("geometry.stl")
    .set_domain_aspect_ratio(1.0f, 2.0f, 1.0f)  // X:Y:Z ratio
    .set_vram_mb(1000u)                          // VRAM budget
    .set_geometry_scale(0.65f)                   // 65% of reference axis
    .set_reference_axis(SimulationConfig::ReferenceAxis::Y)
```

### 2. Unit Conversion

**Old:**
```cpp
const float si_length = 2.4f;
const float si_u = 1.0f;
const float si_rho = 1.225f;
const float lbm_u = 0.075f;
units.set_m_kg_s(lbm_length, lbm_u, 1.0f, si_length, si_u, si_rho);
const float lbm_nu = units.nu(si_nu);
```

**New:**
```cpp
sim.configure_units_with_length(
    2.4f,           // SI reference length (meters)
    1.0f,           // SI velocity (m/s)
    Fluid::AIR,     // Fluid properties (density, viscosity)
    0.075f          // LBM reference velocity
);
```

### 3. Geometry Positioning

**Old (manual mesh translation):**
```cpp
Mesh* mesh = read_stl(path, lbm.size(), lbm.center(), rotation, lbm_length);
mesh->translate(float3(0.0f, 1.0f-mesh->pmin.y+0.1f*lbm_length, 1.0f-mesh->pmin.z));
lbm.voxelize_mesh_on_device(mesh);
```

**New (declarative positioning):**
```cpp
SimulationConfig("geometry.stl")
    .set_rotation_deg(180.0f, 0.0f, 180.0f)
    .set_pmin_offset_ratio(0.0f, 0.1f, 0.006f)  // Y: 10% gap, Z: ~1 cell
// ...
sim.voxelize(lbm);
```

The `set_pmin_offset_ratio(x, y, z)` method positions the geometry's bounding box minimum:
- **X**: Ignored (geometry stays centered in X)
- **Y**: `pmin.y = y * lbm_reference_size` (e.g., 0.1 = 10% of geometry length from inlet)
- **Z**: `pmin.z = z * lbm_reference_size` (e.g., 0.006 ≈ 1 cell above floor)

### 4. Boundary Conditions

**Old (parallel_for loop):**
```cpp
parallel_for(lbm.get_N(), [&](ulong n) {
    uint x=0u, y=0u, z=0u; lbm.coordinates(n, x, y, z);
    if(z==0u) lbm.flags[n] = TYPE_S;                    // Solid floor
    if(lbm.flags[n]!=TYPE_S) lbm.u.y[n] = lbm_u;        // Initialize velocity
    if(x==0u||x==Nx-1u||y==0u||y==Ny-1u||z==Nz-1u)      // Open boundaries
        lbm.flags[n] = TYPE_E;
});
```

**New (fluent builder):**
```cpp
BoundaryBuilder(lbm)
    .set_solid_floor()
    .set_open_boundaries()
    .initialize_velocity_y(flow_velocity_mps)  // Uses SI units
    .apply();
```

### 5. Graphics Configuration

**Old:**
```cpp
lbm.graphics.visualization_modes = VIS_FLAG_SURFACE | VIS_Q_CRITERION;
lbm.graphics.set_camera_centered(-40.0f, 20.0f, 78.0f, 1.25f);
```

**New:**
```cpp
GraphicsConfig(lbm)
    .show_surface()
    .show_vortices()
    .apply();

// For video recording:
VideoRecorder()
    .add(CameraConfig().set_angles(-40.0f, 20.0f).set_fov(78.0f).set_zoom(1.25f))
    .set_video_length_s(10.0f)
    .record(lbm, 10.0f, units);  // 10 seconds
```

---

## Parameter Mapping

### Geometry Positioning

| Old Code | New API | Notes |
|----------|---------|-------|
| `mesh->translate(float3(0, dy, dz))` | `.set_pmin_offset_ratio(0, y_ratio, z_ratio)` | X stays centered |
| `1.0f - mesh->pmin.z` (1 cell) | `z_ratio ≈ 0.006` | Ratio of lbm_reference_size |
| `0.1f * lbm_length` (10% gap) | `y_ratio = 0.1` | Direct ratio |

### Unit Conversion

| Old Code | New API |
|----------|---------|
| `units.set_m_kg_s(lbm_L, lbm_u, 1, si_L, si_u, si_rho)` | `sim.configure_units_with_length(si_L, si_u, fluid, lbm_u)` |
| `units.nu(si_nu)` | `sim.to_lbm_viscosity(si_nu)` or use `Fluid::AIR` |
| `units.t(si_time)` | `sim.to_lbm_timesteps(si_time)` |

### Fluid Properties

Instead of manually specifying `si_nu` and `si_rho`, use predefined fluids:

```cpp
Fluid::AIR      // density=1.225, nu=1.48e-5
Fluid::WATER    // density=998.2, nu=1.004e-6
```

---

## Common Patterns

### Creating LBM with Fluid Properties

```cpp
// Old
LBM lbm(lbm_N, units.nu(1.48e-5f));

// New
LBM lbm = sim.create_lbm(Fluid::AIR);
```

### Reynolds Number

```cpp
// Old
print_info("Re = " + to_string(to_uint(units.si_Re(si_length, si_u, si_nu))));

// New
sim.print_reynolds_number(Fluid::AIR);
```

---

## Checklist for Porting

- [ ] Replace includes with `#include "setup/setup.hpp"`
- [ ] Create `SimulationConfig` with geometry filename
- [ ] Set domain aspect ratio and VRAM budget
- [ ] Set geometry scale (fraction of reference axis)
- [ ] Set rotation using degrees
- [ ] Set positioning with `set_pmin_offset_ratio()` or `set_center_offset_ratio()`
- [ ] Call `sim.setup()` and `sim.configure_units_with_length()`
- [ ] Create LBM with `sim.create_lbm(Fluid::...)`
- [ ] Voxelize with `sim.voxelize(lbm)`
- [ ] Replace boundary loops with `BoundaryBuilder`
- [ ] Replace graphics setup with `GraphicsConfig`
- [ ] Use SI units for velocities in boundary initialization
