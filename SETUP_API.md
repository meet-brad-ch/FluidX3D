# Setup API Guide

The Setup API (`#include "setup/setup.hpp"`) describes a FluidX3D simulation in physical units. This guide shows how
to write one; [README.md](README.md#setup-api) lists the components, [lib/setup/README.md](lib/setup/README.md) the
layout and conventions of the code, and the [appendix](#appendix-coming-from-the-low-level-api) maps the core's
low-level calls to the API.

- Physical quantities with compile-time checks: `2.4_m`, `10_mps`, `180_deg`, `1000_mb` (a length where a speed is expected does not compile)
- One description of the simulation box (`Domain`) and the geometry in it (`Model`)
- A `Simulation` that owns the core's LBM: it sizes the grid, sets the units, creates the LBM on its first use with the physics set before (gravity, surface tension, temperatures, particles), voxelizes the model and runs for a simulated time
- Fluent builders for boundaries, free surfaces, thermal walls and graphics; moving parts, a video, forces, particles and a wave maker that act while it runs
- Predefined fluids (`Fluid::AIR`, `Fluid::WATER`)
- Clear errors: conflicting settings stop the program at startup with a message (`SetupError`); a physics setting after the simulation's first use is an error, not wrong physics

An example is one `main.cpp` with a `main_setup()` function and a `CMakeLists.txt` that names the core's extensions
it needs (see [CMAKE.md](CMAKE.md#configuration-per-example)):
```cmake
add_fluidx3d_example(NAME cow EXTENSIONS FP16S EQUILIBRIUM_BOUNDARIES SUBGRID INTERACTIVE_GRAPHICS)
```

---

## At a Glance

| Task | Call |
|------|------|
| A box of fluid | `Domain::box(1_m, 5_m, 0.75_m).vram(2000_mb)` |
| A model with clearances around it (STL in metres) | `Domain::around(Model("hill.stl")).clearances(2_m, 500_m, 100_m).cell_size(8_m)` |
| A model of known size in a box | `Domain::around(Model(file).length(2.4_m)).size(x, y, z).vram(1000_mb)` |
| Where the model sits | `.gap_to_inlet(...)`, `.on_floor()`, `.gap_to_floor(...)` or `.model_offset(...)` |
| Several GPUs | `Domain::...gpus(2, 4, 1)` |
| The simulation and its units | `Simulation sim(domain, Fluid::AIR, 10.0_mps)`, optionally with `LatticeMach(0.13f)` |
| Gravity, a body force | `sim.set_gravity(9.81_mps2)`, `sim.set_body_force({ fx, fy, fz })` in m/s², also while running |
| Surface tension, temperatures, particles | `sim.set_surface_tension(0.072_Npm)`, `sim.set_temperatures(300_K, 330_K)`, `sim.set_particles(32768u, 2.0f)` |
| A fluid with another viscosity | `Fluid::WATER.with_viscosity(speed * length / reynolds)` |
| Boundaries and the initial flow | `sim.boundaries().set_solid_floor().set_open_boundaries().initialize_velocity_y(10_mps).apply()` |
| Objects and regions | `Shape::sphere(center, radius)` with `add_solid()`, `SurfaceBuilder::add_water()` or `add_gas()` |
| The force on a solid | `sim.measure_forces()` for the model, `add_solid(shape, Solid::MEASURED)` for a shape; `sim.forces().force()` in newtons |
| Graphics | `sim.graphics().show_surface().show_vortices().apply()` |
| A video | `sim.video().add(CameraView::orbit(-40_deg, 20_deg)).set_length(10_s)`, written by `run_for()` when built for video |
| Running | `sim.every(0.1_s, [&](Duration t) { ... }); sim.run_for(10.0_s);` |
| Reading the fields | `sim.fields().for_each_cell([&](const FieldReader::Cell& c) { ... })`, `velocity_at({ x, y, z })` |
| Moving parts | `sim.parts().add(MovingPart("rotor.stl").set_rotation_axis(Axis::Y).set_tip_speed(100_mps)).initialize()` |

---

## A Complete Example

```cpp
#include "setup/setup.hpp"

void main_setup() { // extensions (CMakeLists.txt): FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS
    const Speed flow_velocity = 1.0_mps;
    const Length cow_length = 2.4_m;
    const Length domain_length = cow_length / 0.65f; // the cow is 65 % of the domain length

    Simulation sim(Domain::around(Model("Cow_t.stl").rotation(180_deg, 0_deg, 180_deg).length(cow_length))
        .size(0.5f * domain_length, domain_length, 0.5f * domain_length)
        .gap_to_inlet(0.1f * cow_length) // the cow's nose
        .on_floor()
        .vram(1000_mb),
        Fluid::AIR, flow_velocity, LatticeMach(0.13f)); // optional: how compressible the flow is simulated

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
    sim.run_for(10.0_s);
}
```

---

## Writing a Simulation

### 1. Domain

Pick the form that matches what you know.

**A box without geometry:**
```cpp
Domain::box(1.0_m, 5.0_m, 0.75_m).vram(2000_mb)
Domain::box(1.0_m, 5.0_m, 0.75_m).cell_size(1.0_m / 128.0f) // 128 x 640 x 96 cells
```

**Clearances around a model** whose STL is already in metres (terrain, measured parts):
```cpp
Domain::around(Model("hill.stl"))
    .clearances(2_m, 500_m, 100_m) // below, above, on each side
    .cell_size(8_m)                // or .vram(...)
    .vram_limit(20000_mb)          // limit for cell_size()
```
The model sits centered in X and Y, on the bottom clearance.

**A size around a model of known length** (an STL in any units):
```cpp
Domain::around(Model("X-Wing.stl").length(13.4_m))  // real length along Y (default axis)
    .size(12_m, 40_m, 6_m)
    .model_offset(0_m, -6_m, 0_m) // model center 6 m upstream of the domain center
    .vram(880_mb)
```
`Model::length(L, axis)` gives the model's real size along an axis after its rotation, before its angle of attack; it scales the whole model, the STL's own units then do not matter, and it is the reference length of the units and the Reynolds number. To make the model a fraction of the domain (65 %), write the size as `length / 0.65f`.

The resolution is either `vram()` (the largest grid that fits the budget, default 2000 MB) or `cell_size()` with an optional `vram_limit()`. With `box()` and `size()`, the cell size gives whole cells along each side (rounded). `gpus(2, 4, 1)` splits the grid among GPUs (`space_shuttle`).

### 2. Geometry

```cpp
Model("aircraft.stl")
    .rotation(90_deg, 0_deg, 90_deg) // applied in the order X, Y, Z
    .angle_of_attack(-10_deg)        // additional pitch, positive: nose up
    .length(62_m, Axis::Y)           // for Domain::size()
    .repair_mesh()                   // fill holes before the SDF is generated
    .mirrored(Axis::X)               // half model: add its mirror image
    .needs({ "rotor.stl" })          // other files of the assembly (moving parts), checked with the model's
    .instructions({ "1. Download ...", "2. Split ..." }) // printed when a file is missing
```

A model can also be an SDF file (`Model("Cow_t_sdf_128x428x258.sdf")`, see `cow_sdf`): the SDF's grid is the model's box, as the core's `read_sdf()` sizes it. It needs a `size()`.

Placement (with `size()` only); a centered model is at the domain's center:
- `.model_offset(x, y, z)`: the model's center, this far from the domain's center
- `.gap_to_inlet(d)`: the model's front (bounding box minimum in Y) this far from the inlet at y = 0; X stays centered
- `.gap_to_floor(d)`: the model's bottom this far above the floor at z = 0
- `.on_floor()`: the model's bottom one cell above z = 0, resting on a solid floor there

The simulation voxelizes the model when it creates the LBM (an STL converted to a cached SDF with `clearances()`). A model that only sizes the domain and moves as a part (`radial_fan`, `tie_fighter`) says `sim.skip_model_voxelization()`.

### 3. The Simulation and Its Units

```cpp
Simulation sim(domain, Fluid::AIR, flow_velocity);                  // optional fourth argument: LatticeMach(0.13f)
sim.set_gravity(9.81_mps2);                                          // VOLUME_FORCE; or set_body_force({ fx, fy, fz })
sim.set_surface_tension(0.072_Npm);                                  // SURFACE
sim.set_temperatures(300.0_K, 330.0_K);                              // TEMPERATURE: the range that becomes 0.5 to 1.5
sim.set_particles(32768u, 2.0f);                                     // PARTICLES: count, density relative to the fluid's
sim.measure_forces();                                                // FORCE_FIELD: the model's force, sim.forces()
```
The fluid's density sets the lattice density 1 and its viscosity the lattice viscosity; the reference length comes from the domain (the box's longest side, or the model's `length()`), the reference speed is usually the fastest in the flow. The Reynolds number of these three is printed. A custom fluid: `FluidProperties{ .density = 1260_kgpm3, .kinematic_viscosity = 1.12E-3_m2ps }`, or `Fluid::WATER.with_viscosity(nu)`.

The physics setters come before the simulation's first use (a builder, the graphics, a run): the LBM is created then, and a setter after it stops with a message. `set_body_force()` is the exception: it also changes the force while the simulation runs (`cube_gravity`).

The LBM's speed of sound is not the fluid's: it is 1/sqrt(3) cells per time step. The lattice Mach number, the reference velocity over it, sets the time step and how compressible the simulated flow is (the error grows as its square); the simulation prints it. The default, about 0.17, is the core's usual lattice speed of 0.1; a lower number is more accurate and needs more time steps.

Gravity and other body forces are in m/s²: `set_gravity(9.81_mps2)` (along -Z; `set_gravity(g, Axis::Y)` for another axis), `set_body_force({ Acceleration{}, drive, Acceleration{} })` for a pressure gradient per density (`poiseuille_flow`), `set_body_force({ Acceleration{}, -0.14f * g, -g })` for a sloped river bed. Times are durations, temperatures absolute (`330.0_K` or `57.0_C`).

**A setup given in lattice units** (a viscosity of 0.02, a gravity of 0.0005, as the core's own examples) has no physical size. Pick one, such as a 1 cm cylinder in water, and keep the dimensionless numbers: the lattice speed as the lattice Mach number (`LatticeMach(sqrtf(3.0f) * lbm_u)`, or rounded: `LatticeMach(0.13f)` for 0.075), the Reynolds number for the viscosity (`Fluid::WATER.with_viscosity(speed * length / reynolds)`), the Froude number with real gravity for free surfaces (`froude * sqrt(9.81_mps2 * depth)`), and the surface tension the lattice value is at that scale (`sim.unit_scale().si_surface_tension(0.01f)` computes it). The lattice setup then equals the given one; see `lid_driven_cavity`, `karman_vortex_street`, `river` and `cube_gravity`. Free-surface cases with a sub-cell water depth (`dam_break`, `breaking_waves`) keep a Reynolds number this way, because real water viscosity would need a much finer grid.

### 4. Boundary Conditions

```cpp
sim.boundaries()
    .set_solid_floor()
    .set_open_boundaries()
    .initialize_velocity_y(flow_velocity)
    .apply();
```
Faces are periodic unless set solid or open. `set_open_boundaries()` opens every face that is not set solid: with `set_solid_floor()` a wind tunnel, alone all six faces (an aircraft in free flow). `set_solid_sides()` closes the four X and Y faces, `set_solid_box()` all six; `set_periodic(Axis::Z)` keeps the Z faces periodic with open boundaries (a 2D domain). A wind profile: `set_wind_profile_power_law(10_mps, 100_m, 0.25f).set_wind_direction(Face::Y_MIN)`.

Free surfaces use `sim.surface()`, in metres and m/s (`hydraulic_jump`):
```cpp
sim.surface()
    .set_water_level(water_height)
    .initialize_hydrostatic() // with the simulation's gravity
    .set_solid_faces({Face::X_MIN, Face::X_MAX, Face::Y_MIN, Face::Z_MIN})
    .add_solid(Shape::box({0_m, 0_m, 0_m}, {domain_x, socket_length, socket_height}))
    .add_inflow(Face::Y_MIN, inlet_velocity, socket_height, water_height)
    .add_outflow(Face::Y_MAX, outlet_velocity)
    .apply();
```
Positions are measured from the domain's origin corner. The water level, the inflows' heights and boxes cover whole cells (rounded); a bound in the last cell reaches the domain's end.

**Objects and regions** are `Shape`s in metres:
```cpp
sim.boundaries()                                                 // karman_vortex_street
    .add_solid(Shape::cylinder({ 4_cm, 4_cm, 0.5f * cell }, Axis::Z, 0.5_cm, cell))
    .set_open_boundaries()
    .set_periodic(Axis::Z)                                       // a 2D domain, one cell high
    .initialize_velocity_y(flow_speed)
    .apply();

sim.surface()
    .set_water_level(depth)
    .add_water(Shape::sphere(drop, 0.5f * D), { Speed{}, s * u, -c * u })   // a falling drop (raindrop)
    .add_gas(Shape::sphere(bubble, 2_mm))                                    // a bubble (bursting_bubble)
    .add_solid(floor_and_ceiling & !hole)                                    // walls with a hole (periodic_faucet)
    .add_solid(Shape::half_space({ 0.5_m, 1_m, 0_m }, float3(0, -1, 8)))     // a sloped beach (breaking_waves)
    .apply();
```
The shapes are `sphere`, `cylinder` (axis, radius, length), `box` (two corners, whole cells), `triangle` (a plate about one cell thick), `torus` and `half_space` (one side of a plane, the side its normal points away from); `!a` is everything outside `a`, `a & b` the cells in both, `a | b` those in either. A cell belongs to a shape when its center does (cell i spans i to i+1 cell sizes); water and gas shapes are smooth at a sphere's surface.

Fields are functions of the position in metres: `initialize_velocity([](Position p) { return Velocity{...}; })` and `initialize_pressure()` for an analytic start (`taylor_green_2d`, `stokes_drag`), `add_moving_solid(shape, wall_velocity)` for a turning cylinder (`taylor_couette`), `add_moving_solid(shape, [](Position, Duration t) { ... })` for a wall whose velocity changes in time, set every time step (`liquid_metal`'s membrane), `set_force_field()` for a volume force per cell (FORCE_FIELD, `colliding_droplets`). They are evaluated at each cell's center. A solid whose force is measured: `add_solid(shape, Solid::MEASURED)` (`stokes_drag`).

Thermal walls use `sim.thermal()` (temperatures in Kelvin; the hydrostatic start uses the simulation's gravity axis), wave makers `sim.wave_maker()`, rotating parts `sim.parts()`:
```cpp
sim.parts()
    .add(MovingPart("rotor.stl")
        .set_rotation_axis(Axis::Y)
        .set_tip_speed(tip_speed)
        .centered_on_model(Position{0_m, -0.21f * fan_diameter, 0_m})) // only for an STL in other coordinates
    .initialize();                                       // after the boundaries; the parts turn as the simulation runs
```
A moving part gets the model's own transform (its rotation, its scale, its move to the planned center), so a rotor split from the same CAD file stays where it was on the model; a part in other coordinates is centered on the model instead. A part is re-voxelized, turned by the angle since its last update, whenever its tip has moved half a cell, or every `set_update_interval(0.8_us)`. A tumbling part turns at an angular speed: `MovingPart(sim.model_file()).set_tumble(axis, 180_deg / 1.0_s)` (`tie_fighter`). `ModelPlacement::of(sim.plan())` gives the same transform for meshes placed by hand (`mercedes_f1`).

### 5. Graphics and Video

```cpp
sim.graphics()
    .show_surface()
    .show_vortices()
    .set_camera(CameraView::orbit(-40_deg, 25_deg).field_of_view(70_deg)) // also the interactive graphics' first view
    .apply();

sim.video() // written by run_for() when built with GRAPHICS but not INTERACTIVE_GRAPHICS
    .add(CameraView::orbit(-40_deg, 20_deg).field_of_view(78_deg).view_height(domain_length / 1.25f)) // export/
    .add("front", CameraView::at({ 20_m, 2.7_m, 15_m }, -33_deg, 42_deg))                           // export/front/
    .add("pan", [](float progress) { return CameraView::orbit(-70_deg + progress * 100_deg, 2_deg); }) // moving
    .set_length(10.0_s);       // 60 frames per second of video, spread over the simulated time; or set_frame_interval(0.1_s)
sim.run_for(10.0_s);           // 10 simulated seconds
```
A camera's azimuth (about Z, from +X toward +Y) and elevation (above the horizontal) give the direction from what it sees to the camera. An `orbit()` camera circles the domain's center and `view_height()` is the length the frame's smaller side spans there; an `at()` camera stands at a position in metres from the domain's origin corner, and `look_at(target)` turns it. The graphics modes are `show_surface()`, `show_flags()`, `show_vortices()`, `show_velocity_field()`, `show_density_field()`, `show_streamlines()`, `show_free_surface()` and `show_free_surface_mesh()`, with `set_slice_mode(SliceMode::Z)` for the fields.

### 6. Running and Reading the Fields

`sim.run()` runs until a task stops it (with interactive graphics: until the window is closed); `sim.run_for(t)` runs t of simulated time in every build. Anything that changes as the simulation runs is a task in simulated time:
```cpp
sim.every(0.1_s, [&](Duration t) { ... });   // at 0 s, then every 0.1 s of simulated time
sim.every_step([&](Duration t) { ... });     // every time step
sim.run_for(20.0_s);                         // or run(): until a task calls sim.stop() (stokes_drag)
```
A task gets the simulated time since the start. `run_for()` continues from where the last run ended, so phases follow one another: `sim.set_body_force(...); sim.run_for(0.8_s);` (`cube_gravity`). The video, the moving parts and the wave maker register their own tasks.

The fields are read as a snapshot in SI units:
```cpp
const FieldReader fields = sim.fields();
fields.for_each_cell([&](const FieldReader::Cell& c) {
    if(c.solid) return;
    error += sq(magnitude(c.velocity).si() - analytic(c.center));   // poiseuille_flow
});
const Pressure p = fields.pressure_at({ 1.25_cm, 1.25_cm, 2.5_cm });  // the cell containing the point
```
`sim.forces()` gives the force on the measured solids in newtons and the drag coefficient (`ahmed_body`); the core's `LBM` is there for anything else (`sim.lbm()`), with `sim.unit_scale()` to convert.

---

## Units and Literals

| Quantity | Literals |
|----------|----------|
| Length | `_m`, `_cm`, `_mm`, `_km` |
| Model lengths (relative) | `_lengths` |
| Duration, frequency | `_s`, `_ms`, `_us`, `_min`, `_Hz` |
| Speed, acceleration | `_mps`, `_kmh`, `_mps2` |
| Mass, density, viscosity | `_kg`, `_kgpm3`, `_m2ps` |
| Volume flow rate | `_m3ps` |
| Force, pressure, surface tension | `_N`, `_Pa`, `_Npm` |
| Temperature (absolute) | `_K`, `_C` |
| Angle | `_deg`, `_rad` |
| Device memory | `_mb`, `_gb` (1 GB = 1024 MB) |

Quantities multiply and divide into new dimensions (`10_m / 2_s` is a `Speed`, `0.389_m * 0.288_m` an `Area`, `180_deg / 1.0_s` an `AngularSpeed`), scale with plain numbers (`0.5f * domain_length`) and give their SI value with `.si()`. `Vector3<Q>` (`Position`, `Velocity`, `ForceVector`) has `magnitude()`.

---

## Testing a Setup

`tests/physics/` holds simulations checked against analytic solutions (`ctest -L physics`): a `main_setup()` that ends with `PhysicsCheck::report()`, registered with `fluidx3d_add_physics_test()`. A new check of this kind is the way to test a setup. See [CMAKE.md](CMAKE.md#tests).

---

## Appendix: Coming from the Low-Level API

The core's own examples configure everything in lattice units and cell indices. Their calls map to the API as follows.

| Low-level | Setup API |
|-----------|-----------|
| `#define` lines in `defines.hpp` | `EXTENSIONS` in the example's `CMakeLists.txt` |
| `resolution(float3(1, 5, 0.75), 2000u)` | `Domain::box(1_m, 5_m, 0.75_m).vram(2000_mb)` |
| `resolution(...)` + `lbm_length = 0.65f*lbm_N.y` | `Domain::around(Model(file).length(2.4_m)).size(x, y, z).vram(1000_mb)` |
| `read_stl(...)` + `mesh->translate(...)` | `.gap_to_inlet(...)`, `.gap_to_floor(...)` or `.model_offset(...)` |
| `units.set_m_kg_s(...)`, `LBM lbm(lbm_N, lbm_nu)` | `Simulation sim(domain, Fluid::AIR, 10.0_mps)`; an `lbm_u` of 0.075 is `LatticeMach(0.13f)` |
| `LBM lbm(Nx, Ny, Nz, nu, fx, fy, fz)`, `lbm.set_f(...)` | `sim.set_gravity(...)` or `sim.set_body_force({ fx, fy, fz })` |
| `LBM lbm(..., sigma)`, `(..., alpha, beta)`, `(..., particles_N, particles_rho)` | `sim.set_surface_tension()`, `sim.set_temperatures()`, `sim.set_particles()` |
| `units.nu_from_Re(Re, L, u)` | `Fluid::WATER.with_viscosity(speed * length / reynolds)` |
| `lbm.voxelize_mesh_on_device(mesh)` | done by the simulation |
| `parallel_for` boundary setup with `TYPE_S`, `TYPE_E` | `sim.boundaries()...apply()` |
| `sphere(x, y, z, p, r)` and the other cell tests | `Shape::sphere(center, radius)` and the other shapes, in metres |
| `lbm.flags[n] = TYPE_S\|TYPE_X` for `object_force()` | `sim.measure_forces()`, `add_solid(shape, Solid::MEASURED)`; `sim.forces()` |
| `lbm.graphics.visualization_modes = ...` | `sim.graphics().show_...().apply()` |
| `set_camera_centered(rx, ry, fov, zoom)` | `CameraView::orbit(rx_deg, ry_deg).field_of_view(fov_deg).view_height(L / zoom)`, `L` the domain's longest side |
| `set_camera_free(float3(fx*Nx, fy*Ny, fz*Nz), rx, ry, fov)` | `CameraView::at({(fx + 0.5) * X, (fy + 0.5) * Y, (fz + 0.5) * Z}, rx_deg, ry_deg).field_of_view(fov_deg)` for a domain of `X` x `Y` x `Z` metres |
| `next_frame(lbm_T, 30.0f)` in the run loop, inside `#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)` | `sim.video().set_length(30.0_s)` and `sim.run_for(time)`, no `#if` |
| `lbm.run(n)` loops with `lbm.get_t()` | `sim.every(...)` tasks and `sim.run_for()` |
| `lbm.u.read_from_device()` and index arithmetic | `sim.fields()` |
