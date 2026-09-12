# Porting Guide: Low-Level to High-Level API

This guide explains how to port FluidX3D examples from the low-level API to the Setup API (`#include "setup/setup.hpp"`).

---

## Overview

The Setup API provides:
- Physical units with compile-time checks: `2.4_m`, `10_mps`, `180_deg`, `1000_mb` (a length where a speed is expected does not compile)
- One description of the simulation box (`Domain`) and the geometry in it (`Model`)
- Automatic domain sizing, mesh loading, scaling, placement and voxelization
- Fluent builders for boundaries, free surfaces, thermal walls, graphics and video
- Predefined fluids (`Fluid::AIR`, `Fluid::WATER`)
- Clear errors: conflicting settings stop the program at startup with a message (`SetupError`)

---

## Quick Reference

| Old (Low-Level) | New (Setup API) |
|-----------------|-----------------|
| `resolution(float3(1, 5, 0.75), 2000u)` | `Domain::box(1_m, 5_m, 0.75_m).vram(2000_mb)` |
| grid from a mesh plus margins | `Domain::around(Model("hill.stl")).clearances(2_m, 500_m, 100_m).cell_size(8_m)` |
| `resolution(...)` + `lbm_length = 0.65f*lbm_N.y` | `Domain::around(Model(file).length(2.4_m)).size(x, y, z).vram(1000_mb)` |
| `read_stl(...)` + `mesh->translate(...)` | `.gap_to_inlet(...)`, `.gap_to_floor(...)` or `.model_offset(...)` |
| `units.set_m_kg_s(...)` | `sim.configure_units(10.0_mps, Fluid::AIR)`, optionally with `LatticeMach(0.13f)` for `lbm_u = 0.075` |
| `lbm.run(n)` loops with `lbm.get_t()` | `Runner(lbm).every(0.1_s, [&](Duration t) { ... }).run_for(10.0_s)` |
| parts re-voxelized every n time steps | `MovingPart::set_update_interval(0.8_us)`; by default when the tip has moved half a cell |
| `lbm.voxelize_mesh_on_device(mesh)` | `sim.voxelize(lbm)` |
| `parallel_for` boundary setup | `BoundaryBuilder(lbm).set_solid_floor()...` |
| `sphere(x, y, z, p, r)` and the other shapes in a `parallel_for` | `Shape::sphere(center, radius)` with `BoundaryBuilder::add_solid()`, `SurfaceBuilder::add_water()` or `add_gas()` |
| `LBM lbm(Nx, Ny, Nz, nu, fx, fy, fz)` | `sim.create_lbm(viscosity, { fx, fy, fz })`, a body force per mass in m/s² |
| `lbm.set_f(fx, fy, fz)` while running | `const float3 f = sim.to_lbm_acceleration({ ... }); lbm.set_f(f.x, f.y, f.z);` |
| `units.nu_from_Re(Re, L, u)` | `speed * length / reynolds`, a `KinematicViscosity` |
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

### New Setup API

```cpp
#include "defines.hpp"
#include "lbm.hpp"
#include "setup/setup.hpp"

void main_setup() {
    const Speed flow_velocity = 1.0_mps;
    const Length cow_length = 2.4_m;
    const Length domain_length = cow_length / 0.65f; // the cow is 65 % of the domain length

    SimulationSetup sim(Domain::around(Model("Cow_t.stl").rotation(180_deg, 0_deg, 180_deg).length(cow_length))
        .size(0.5f * domain_length, domain_length, 0.5f * domain_length)
        .gap_to_inlet(0.1f * cow_length) // the cow's nose
        .on_floor()
        .vram(1000_mb));

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

---

## Porting Step by Step

### 1. Domain

Pick the form that matches how the original sized its grid.

**A box without geometry** (the original used `resolution()` with a fixed aspect ratio, or a fixed grid):
```cpp
SimulationSetup sim(Domain::box(1.0_m, 5.0_m, 0.75_m).vram(2000_mb));
SimulationSetup sim(Domain::box(1.0_m, 5.0_m, 0.75_m).cell_size(1.0_m / 128.0f)); // the original's 128 x 640 x 96 cells
```

**Clearances around a model** whose STL is already in metres (terrain, measured parts):
```cpp
SimulationSetup sim(Domain::around(Model("hill.stl"))
    .clearances(2_m, 500_m, 100_m) // below, above, on each side
    .cell_size(8_m)                // or .vram(...)
    .max_vram(20000_mb));          // limit for cell_size()
```
The model sits centered in X and Y, on the bottom clearance.

**A size around a model of known length** (the original scaled the mesh to a fraction of the domain):
```cpp
SimulationSetup sim(Domain::around(Model("X-Wing.stl").length(13.4_m))  // real length along Y (default axis)
    .size(12_m, 40_m, 6_m)
    .model_offset(0_m, -6_m, 0_m) // model center 6 m upstream of the domain center
    .vram(880_mb));
```
`Model::length(L, axis)` gives the model's real size along an axis after its rotation, before its angle of attack; it scales the whole model, the STL's own units then do not matter, and it is the reference length of the units and the Reynolds number. If the original only knew a ratio (the model is 65 % of the domain), write the size as `length / 0.65f`, which keeps the grid and the scale of the original.

The resolution is either `vram()` (the largest grid that fits the budget, default 2000 MB) or `cell_size()` with an optional `max_vram()`. With `box()` and `size()`, the cell size gives whole cells along each side (rounded): the way to keep an original's fixed grid.

Check a port against its original: configure with `-DFLUIDX3D_BUILD_ORIGINALS=ON`, record the original with `FLUIDX3D_BLESS=1 ctest -R original_<name>`, then compare the two baselines side by side with `cmake -DCOMPARE=<build>/tests/fluidx3d_baseline_compare -DNAMES=<name> -P tests/originals/compare_ports.cmake`.

### 2. Geometry

```cpp
Model("aircraft.stl")
    .rotation(90_deg, 0_deg, 90_deg) // applied in the order X, Y, Z
    .angle_of_attack(-10_deg)        // additional pitch, positive: nose up
    .length(62_m, Axis::Y)           // for Domain::size()
    .repair_mesh()                   // fill holes before the SDF is generated
    .mirrored(Axis::X)               // half model: add its mirror image
```

A model can also be an SDF file (`Model("Cow_t_sdf_128x428x258.sdf")`, see `cow_sdf`): the SDF's grid is the model's box, as the core's `read_sdf()` sizes it. It needs a `size()`.

Placement (with `size()` only); a centered model is at the domain's center, the core's `lbm.center()`:
- `.model_offset(x, y, z)`: the model's center, this far from the domain's center
- `.gap_to_inlet(d)`: the model's front (bounding box minimum in Y) this far from the inlet at y = 0; X stays centered
- `.gap_to_floor(d)`: the model's bottom this far above the floor at z = 0
- `.on_floor()`: the model's bottom one cell above z = 0, resting on a solid floor there (the originals' `1.0f-mesh->pmin.z`)

### 3. Units

```cpp
sim.setup();                                        // plans the domain
sim.configure_units(flow_velocity, Fluid::AIR);     // optional third argument: LatticeMach(0.13f)
LBM lbm = sim.create_lbm(Fluid::AIR);               // or create_lbm_surface / create_lbm_thermal / create_lbm_particles_reynolds
```
The reference length comes from the domain (the box's longest side, or the model's `length()`), so there is no separate length to pass.

The LBM's speed of sound is not the fluid's: it is 1/sqrt(3) cells per time step. The lattice Mach number, the reference velocity over it, sets the time step and how compressible the simulated flow is (the error grows as its square); `configure_units()` prints it. The default, about 0.17, is the originals' `lbm_u = 0.1`; an original's other `lbm_u` is `LatticeMach(sqrtf(3.0f) * lbm_u)`, or rounded: `LatticeMach(0.13f)` for 0.075, `LatticeMach(0.0866f)` for 0.05.

An original set up in lattice units (a viscosity of 0.02, a gravity of 0.0005) has no physical size. Its port picks one, such as a 1 cm cylinder in water, and keeps the original's dimensionless numbers: its lattice speed as the lattice Mach number, the Reynolds number for the viscosity (`speed * length / reynolds`), the Froude number with real gravity for free surfaces (`froude * sqrt(9.81_mps2 * depth)`), and the original's lattice surface tension with `sim.unit_scale().si_surface_tension(0.01f)`. The lattice setup then equals the original's; see `lid_driven_cavity`, `karman_vortex_street`, `river` and `cube_gravity`.

Gravity and other volume forces are given in m/s²: `sim.create_lbm_surface(Fluid::WATER, 9.81_mps2)`, or as a vector for any direction: `sim.create_lbm(viscosity, { Acceleration{}, drive, Acceleration{} })` for a pressure gradient per density (`poiseuille_flow`), `create_lbm_surface(viscosity, { Acceleration{}, -0.14f * g, -g }, sigma)` for a sloped river bed. Times are durations (`Runner`, below; `parts.run(1.0_min)`), temperatures absolute (`330.0_K` or `57.0_C`). A thermal LBM needs the range of its temperatures first: `sim.configure_temperatures(300.0_K, 330.0_K)` before `create_lbm_thermal()`, then `ThermalBuilder(lbm, sim.temperature_scale()).set_hot_wall(Face::Z_MIN, 330.0_K)`. For free-surface cases with a sub-cell water depth, see `dam_break` and `breaking_waves`: they match the original's Reynolds number, because real water viscosity would need a much finer grid.

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

**New (fluent builder, SI units):**
```cpp
BoundaryBuilder(lbm)
    .set_solid_floor()
    .set_open_boundaries()
    .initialize_velocity_y(flow_velocity)
    .apply();
```
`set_open_boundaries()` opens every face that is not set solid: with `set_solid_floor()` a wind tunnel, alone all six faces (an aircraft in free flow, the originals' "all non periodic").

Free surfaces use `SurfaceBuilder`, in metres and m/s (`hydraulic_jump`):
```cpp
SurfaceBuilder(lbm)
    .set_water_level(water_height)
    .initialize_hydrostatic() // with the LBM's own gravity
    .set_solid_faces({Face::X_MIN, Face::X_MAX, Face::Y_MIN, Face::Z_MIN})
    .add_solid(Shape::box({0_m, 0_m, 0_m}, {domain_x, socket_length, socket_height}))
    .add_inflow(Face::Y_MIN, inlet_velocity, socket_height, water_height)
    .add_outflow(Face::Y_MAX, outlet_velocity)
    .apply();
```
Positions are measured from the domain's origin corner. The water level, the inflows' heights and boxes cover whole cells, rounded as the originals' `to_uint(units.x(...))`; a bound in the last cell reaches the domain's end.

**Objects and regions** are `Shape`s in metres, in place of the core's cell tests (`sphere()`, `cylinder()`, ...) in a `parallel_for`:
```cpp
BoundaryBuilder(lbm)                                             // karman_vortex_street
    .add_solid(Shape::cylinder({ 4_cm, 4_cm, 0.5f * cell }, Axis::Z, 0.5_cm, cell))
    .set_open_boundaries()
    .set_periodic(Axis::Z)                                       // a 2D domain, one cell high
    .initialize_velocity_y(flow_speed)
    .apply();

SurfaceBuilder(lbm)
    .set_water_level(depth)
    .add_water(Shape::sphere(drop, 0.5f * D), { Speed{}, s * u, -c * u }) // a falling drop (raindrop)
    .add_gas(Shape::sphere(bubble, 2_mm))                                  // a bubble (bursting_bubble)
    .add_solid(floor_and_ceiling & !hole)                                  // walls with a hole (periodic_faucet)
    .apply();
```
The shapes are `sphere`, `cylinder` (axis, radius, length), `box` (two corners, whole cells), `triangle` (a plate about one cell thick) and `torus`; `!a` is everything outside `a`, `a & b` the cells in both, `a | b` those in either. A cell belongs to a shape when its center does (cell i spans i to i+1 cell sizes), so the domain's center is the core's `lbm.center()`; water and gas shapes are smooth at a sphere's surface, as the core's `sphere_plic()`.

Fields are functions of the position in metres: `BoundaryBuilder::initialize_velocity([](Position p) { return Velocity{...}; })` and `initialize_pressure()` for an analytic start (`taylor_green_2d`, `stokes_drag`), `add_moving_solid(shape, wall_velocity)` for a turning cylinder (`taylor_couette`), `set_force_field()` for a volume force per cell (FORCE_FIELD, `colliding_droplets`). They are evaluated at each cell's center.

Thermal walls use `ThermalBuilder` (temperatures in Kelvin), wave makers `WaveBoundary`, rotating parts `MovingPartsManager`.

A moving part gets the model's own transform (its rotation, its scale, its move to the planned center), so a rotor split from the same CAD file stays where it was on the model — the old `mesh->scale(scale); mesh->translate(offset)` with the body's `scale` and `offset`. A part in other coordinates is centered on the model instead, as the old code did with `translate(center - part->get_bounding_box_center() + offset)`:
```cpp
MovingPartsManager parts(sim, lbm);                 // after configure_units()
parts.add(MovingPart("rotor.stl")
    .set_rotation_axis(RotationAxis::Y)
    .set_tip_speed(tip_speed)
    .centered_on_model(Position{0_m, -0.21f * fan_diameter, 0_m})); // only for an STL in other coordinates
parts.initialize();
parts.run(0.5_s);                                   // or VideoRecorder().record(lbm, 0.5_s, parts)
```
A part is re-voxelized, turned by the angle since its last update, whenever its tip has moved half a cell, or every `set_update_interval(0.8_us)`. A tumbling part turns at an angular speed: `MovingPart(file).set_tumble(axis, 180_deg / 1.0_s).set_update_interval(2.2_ms)` (`tie_fighter`). `ModelPlacement::of(sim.get_results())` gives the same transform for meshes placed by hand (`cells_per_unit()`, `load(path)`).

### 5. Graphics and Video

```cpp
GraphicsConfig(lbm)
    .show_surface()
    .show_vortices()
    .set_camera(CameraView::orbit(-40_deg, 25_deg).field_of_view(70_deg)) // also the interactive graphics' first view
    .apply();

// headless video (GRAPHICS without INTERACTIVE_GRAPHICS)
VideoRecorder()
    .add(CameraView::orbit(-40_deg, 20_deg).field_of_view(78_deg).view_height(domain_length / 1.25f)) // export/
    .add("front", CameraView::at({ 20_m, 2.7_m, 15_m }, -33_deg, 42_deg))                           // export/front/
    .add("pan", [](float progress) { return CameraView::orbit(-70_deg + progress * 100_deg, 2_deg); }) // moving
    .set_video_length(10.0_s)
    .record(lbm, 10.0_s); // 10 simulated seconds; record(lbm, time, parts) also turns MovingPartsManager's parts
```
A camera's azimuth (about Z, from +X toward +Y) and elevation (above the horizontal) give the direction from what it sees to the camera, as the core's angles; `CameraView::at(position).look_at(target)` computes them. The old calls map to:

| Old | New |
|-----|-----|
| `set_camera_centered(rx, ry, fov, zoom)` | `CameraView::orbit(rx_deg, ry_deg).field_of_view(fov_deg).view_height(L / zoom)`, with `L` the domain's longest side: the frame's smaller side spans `view_height` at the domain's center |
| `set_camera_free(float3(fx*Nx, fy*Ny, fz*Nz), rx, ry, fov)` | `CameraView::at({(fx + 0.5) * X, (fy + 0.5) * Y, (fz + 0.5) * Z}, rx_deg, ry_deg).field_of_view(fov_deg)` for a domain of `X` x `Y` x `Z` metres: positions are from the domain's origin corner, the core's from its center |
| `next_frame(lbm_T, 30.0f)` in the run loop | `VideoRecorder().set_video_length(30.0_s).record(lbm, time)` |
| a camera computed from `lbm.get_t()/lbm_T` | `.add(name, [](float progress) { return CameraView::...; })` |

### 6. Running

`lbm.run()` runs until the window is closed. A run loop that changes something as the simulation runs becomes a `Runner` with tasks in simulated time:
```cpp
Runner runner(lbm);
runner.every(0.1_s, [&](Duration t) { wave.update(t); })   // at 0 s, then every 0.1 s of simulated time (breaking_waves)
      .every_step([&](Duration t) { /* ... */ });           // every time step (liquid_metal)
runner.run_for(20.0_s);                                     // or run(): until a task calls runner.stop() (stokes_drag)
```
A task gets the simulated time since the start. `run_for()` continues from where the last run ended, so phases follow one another: `set_gravity(...); runner.run_for(0.8_s);` (`cube_gravity`). `MovingPartsManager::run()` and `VideoRecorder::record()` run a `Runner` themselves.

---

## Units and Literals

| Quantity | Literals |
|----------|----------|
| Length | `_m`, `_cm`, `_mm`, `_km` |
| Model lengths (relative) | `_lengths` |
| Duration, frequency | `_s`, `_min`, `_Hz` |
| Speed, acceleration | `_mps`, `_kmh`, `_mps2` |
| Mass, density, viscosity | `_kg`, `_kgpm3`, `_m2ps` |
| Volume flow rate | `_m3ps` |
| Force, pressure, surface tension | `_N`, `_Pa`, `_Npm` |
| Temperature (absolute) | `_K`, `_C` |
| Angle | `_deg`, `_rad` |
| Device memory | `_mb`, `_gb` (1 GB = 1024 MB) |

Quantities multiply and divide into new dimensions (`10_m / 2_s` is a `Speed`), scale with plain numbers (`0.5f * domain_length`) and give their SI value with `.si()`.

---

## Mapping from SimulationConfig

Earlier versions of the Setup API configured the domain with `SimulationConfig`, which is removed. Its setters map to:

| SimulationConfig | Domain / Model |
|------------------|----------------|
| `SimulationConfig().set_domain_size_m(x, y, z)` | `Domain::box(x, y, z)` |
| `SimulationConfig(file).set_clearances_m(b, t, s)` | `Domain::around(Model(file)).clearances(b, t, s)` |
| `.set_vram_mb(n)` | `.vram(n_mb)` |
| `.set_voxel_size_m(v).set_max_vram_mb(n)` | `.cell_size(v).max_vram(n_mb)` |
| `.set_domain_aspect_ratio(ax, ay, az).set_geometry_scale(s)` with reference axis A and `configure_units_with_length(L, ...)` | `Model(file).length(L, A)` and `.size(...)`, where the size along A is `L / s` and the other sides follow the ratio. The old scale applied to the model's longest side after the whole rotation (the core's `voxelize_stl()` size); give `L` along that side to keep the geometry |
| `.set_center_offset_ratio(x, y, z)` | `.model_offset(x * L, y * L, z * L)` |
| `.set_pmin_offset_ratio(0, y, z)` | `.gap_to_inlet(y * L).gap_to_floor(z * L)`, or `.on_floor()` for a z of about one cell |
| `.set_rotation_deg(x, y, z)` | `Model::rotation(x_deg, y_deg, z_deg)` |
| `.set_angle_of_attack_deg(a)` | `Model::angle_of_attack(a_deg)` |
| `.set_fix_mesh(true)` | `Model::repair_mesh()` |
| `.set_mirror_plane(MirrorPlane::X)` | `Model::mirrored(Axis::X)` |
| `configure_units_with_length(L, v, fluid)` | `Model::length(L)` and `configure_units(v, fluid)` |

---

## Checklist for Porting

- [ ] Replace includes with `#include "setup/setup.hpp"`
- [ ] Describe the domain with `Domain::box`, `Domain::around(...).clearances(...)` or `Domain::around(...).size(...)`
- [ ] Give the model's real length if the domain is sized around it
- [ ] Set rotation, angle of attack and placement in degrees and metres
- [ ] Call `sim.setup()` and `sim.configure_units()`
- [ ] Create the LBM with `sim.create_lbm*(Fluid::...)`
- [ ] Voxelize with `sim.voxelize(lbm)`
- [ ] Replace boundary loops with the builders
- [ ] Replace graphics setup with `GraphicsConfig`
- [ ] Record the example's baseline: `FLUIDX3D_BLESS=1 ctest -R baseline_<example>` (see [CMAKE.md](CMAKE.md#tests))
