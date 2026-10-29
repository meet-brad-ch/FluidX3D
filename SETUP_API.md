# Setup API Guide

The Setup API (`#include "setup/setup.hpp"`) describes a FluidX3D simulation in physical units. This guide shows how
to write one; [README.md](README.md#setup-api) lists the components, [lib/setup/README.md](lib/setup/README.md) the
layout and conventions of the code, and the [appendix](#appendix-coming-from-the-low-level-api) maps the core's
low-level calls to the API.

- Physical quantities with compile-time checks, on the Au units library: `2.4 * m`, `10.0 * m / s`, `180.0 * deg`, `1000.0 * MiB` (a length where a speed is expected does not compile; see [Units and Symbols](#units-and-symbols))
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
| A box of fluid | `Domain::box(1.0 * m, 5.0 * m, 0.75 * m).vram(2000.0 * MiB)` |
| A model with clearances around it (STL in metres) | `Domain::around(Model("hill.stl")).clearances(2.0 * m, 500.0 * m, 100.0 * m).cell_size(8.0 * m)` |
| A model of known size in a box | `Domain::around(Model(file).length(2.4 * m)).size(x, y, z).vram(1000.0 * MiB)` |
| Where the model sits | `.gap_to_inlet(...)`, `.on_floor()`, `.gap_to_floor(...)` or `.model_offset(...)` |
| Several GPUs | `Domain::...gpus(2, 4, 1)` |
| The simulation and its units | `Simulation sim(domain, Fluid::AIR, 10.0 * m / s)`, optionally with `si::LatticeMach(0.13)` |
| Gravity, a body force | `sim.set_gravity(9.81 * m / (s * s))`, `sim.set_body_force({ fx, fy, fz })` in m/s², also while running |
| Surface tension, temperatures, particles | `sim.set_surface_tension(0.072 * N / m)`, `sim.set_temperatures(si::kelvin_point(300.0), si::kelvin_point(330.0))`, `sim.set_particles(32768u, 2.0f)` |
| A fluid with another viscosity | `Fluid::WATER.with_viscosity(speed * length / reynolds)` |
| Boundaries and the initial flow | `sim.boundaries().set_solid_floor().set_open_boundaries().initialize_velocity_y(10.0 * m / s).apply()` |
| Objects and regions | `Shape::sphere(center, radius)` with `add_solid()`, `SurfaceBuilder::add_water()` or `add_gas()` |
| The force on a solid | `sim.measure_forces()` for the model, `add_solid(shape, Solid::MEASURED)` for a shape; `sim.forces().force()` in newtons |
| Graphics | `sim.graphics().show_surface().show_vortices().apply()` |
| A video | `sim.video().add(CameraView::orbit(-40.0 * deg, 20.0 * deg)).set_length(10.0 * s)`, written by `run_for()` when built for video |
| Running | `sim.every(0.1 * s, [&](si::Duration t) { ... }); sim.run_for(10.0 * s);` |
| Reading the fields | `sim.fields().for_each_cell([&](const FieldReader::Cell& c) { ... })`, `velocity_at({ x, y, z })` |
| Moving parts | `sim.parts().add(MovingPart("rotor.stl").set_rotation_axis(Axis::Y).set_tip_speed(100.0 * m / s)).initialize()` |
| A convective boundary layer | `ConvectiveBoundaryLayer run(CblDescription().with_cell(10.0 * m)); run.observe(csv); run.run();` ([7.](#7-atmospheric-convection)) |
| Its cost before a run | `CblDescription(...).estimate(Simulation::build_lattice_memory(), std::nullopt).describe()` |
| One part of it, alone | `AtmosphereSimulation atm(grid, theta_0, CblNumerics{}, std::nullopt); atm.start().sounding(s).apply();` |
| A sounding | `Sounding::idealized(si::kelvin_point(300.0), { 974.0 * m, 8.0 * K, 100.0 * m, 3.0 * K / km })`, `Sounding::from_csv("sounding.csv")` |

---

## A Complete Example

```cpp
#include "setup/setup.hpp"

void main_setup() { // extensions (CMakeLists.txt): FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS
    using namespace si::symbols;
    const si::Speed flow_velocity = 1.0 * m / s;
    const si::Length cow_length = 2.4 * m;
    const si::Length domain_length = cow_length / 0.65; // the cow is 65 % of the domain length

    Simulation sim(Domain::around(Model("Cow_t.stl").rotation(180.0 * deg, 0.0 * deg, 180.0 * deg).length(cow_length))
        .size(0.5 * domain_length, domain_length, 0.5 * domain_length)
        .gap_to_inlet(0.1 * cow_length) // the cow's nose
        .on_floor()
        .vram(1000.0 * MiB),
        Fluid::AIR, flow_velocity, si::LatticeMach(0.13)); // optional: how compressible the flow is simulated

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
    sim.run_for(10.0 * s);
}
```

---

## Writing a Simulation

### 1. Domain

Pick the form that matches what you know.

**A box without geometry:**
```cpp
Domain::box(1.0 * m, 5.0 * m, 0.75 * m).vram(2000.0 * MiB)
Domain::box(1.0 * m, 5.0 * m, 0.75 * m).cell_size(1.0 * m / 128.0) // 128 x 640 x 96 cells
```

**Clearances around a model** whose STL is already in metres (terrain, measured parts):
```cpp
Domain::around(Model("hill.stl"))
    .clearances(2.0 * m, 500.0 * m, 100.0 * m) // below, above, on each side
    .cell_size(8.0 * m)                // or .vram(...)
    .vram_limit(20000.0 * MiB)          // limit for cell_size()
```
The model sits centered in X and Y, on the bottom clearance.

**A size around a model of known length** (an STL in any units):
```cpp
Domain::around(Model("X-Wing.stl").length(13.4 * m))  // real length along Y (default axis)
    .size(12.0 * m, 40.0 * m, 6.0 * m)
    .model_offset(0.0 * m, -6.0 * m, 0.0 * m) // model center 6 m upstream of the domain center
    .vram(880.0 * MiB)
```
`Model::length(L, axis)` gives the model's real size along an axis after its rotation, before its angle of attack; it scales the whole model, the STL's own units then do not matter, and it is the reference length of the units and the Reynolds number. To make the model a fraction of the domain (65 %), write the size as `length / 0.65`.

The resolution is either `vram()` (the largest grid that fits the budget, a whole number of MiB, default 2000 MiB) or `cell_size()` with an optional `vram_limit()`. With `box()` and `size()`, the cell size gives whole cells along each side (rounded). `gpus(2, 4, 1)` splits the grid among GPUs (`space_shuttle`). Without `gpus()`, a grid whose DDF buffer is larger than a device accepts in one buffer (`CL_DEVICE_MAX_MEM_ALLOC_SIZE`, a quarter of the memory on NVIDIA cards: 6140 MB on a 24 GB card) is split by itself into the fewest domains that fit, along its longest horizontal axis, and the grid along that axis is trimmed to a multiple of them (at most domains - 1 cells); the domains run on the devices there are, all on one card if it is alone, and exchange their halos each time step. The run prints the split. With `gpus()` the split is yours, and a domain's buffer over the limit is refused.

### 2. Geometry

```cpp
Model("aircraft.stl")
    .rotation(90.0 * deg, 0.0 * deg, 90.0 * deg) // applied in the order X, Y, Z
    .angle_of_attack(-10.0 * deg)        // additional pitch, positive: nose up
    .length(62.0 * m, Axis::Y)           // for Domain::size()
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
Simulation sim(domain, Fluid::AIR, flow_velocity);                  // optional fourth argument: si::LatticeMach(0.13)
sim.set_gravity(9.81 * m / (s * s));                                          // VOLUME_FORCE; or set_body_force({ fx, fy, fz })
sim.set_surface_tension(0.072 * N / m);                                  // SURFACE
sim.set_temperatures(si::kelvin_point(300.0), si::kelvin_point(330.0)); // TEMPERATURE: the range that becomes 0.5 to 1.5
sim.set_particles(32768u, 2.0f);                                     // PARTICLES: count, density relative to the fluid's
sim.measure_forces();                                                // FORCE_FIELD: the model's force, sim.forces()
```
The fluid's density sets the lattice density 1 and its viscosity the lattice viscosity; the reference length comes from the domain (the box's longest side, or the model's `length()`), the reference speed is usually the fastest in the flow. The Reynolds number of these three is printed. A custom fluid: `FluidProperties{ .density = 1260.0 * kg / (m * m * m), .kinematic_viscosity = 1.12E-3 * m * m / s }`, or `Fluid::WATER.with_viscosity(nu)`.

The physics setters come before the simulation's first use (a builder, the graphics, a run): the LBM is created then, and a setter after it stops with a message. `set_body_force()` is the exception: it also changes the force while the simulation runs (`cube_gravity`).

The LBM's speed of sound is not the fluid's: it is 1/sqrt(3) cells per time step. The lattice Mach number, the reference velocity over it, sets the time step and how compressible the simulated flow is (the error grows as its square); the simulation prints it. The default, about 0.17, is the core's usual lattice speed of 0.1; a lower number is more accurate and needs more time steps.

Gravity and other body forces are in m/s²: `set_gravity(9.81 * m / (s * s))` (along -Z; `set_gravity(g, Axis::Y)` for another axis), `set_body_force({ si::Acceleration{}, drive, si::Acceleration{} })` for a pressure gradient per density (`poiseuille_flow`), `set_body_force({ si::Acceleration{}, -0.14 * g, -g })` for a sloped river bed. Times are durations. A temperature is a reading (`si::kelvin_point(330.0)` or `si::celsius_point(57.0)`), a temperature range a difference (`30.0 * K`). `sim.set_temperature_scale(reference, unit)` sets the scale directly: the lattice temperature 1 at the reference and 1 more per unit.

**A setup given in lattice units** (a viscosity of 0.02, a gravity of 0.0005, as the core's own examples) has no physical size. Pick one, such as a 1 cm cylinder in water, and keep the dimensionless numbers: the lattice speed as the lattice Mach number (`si::LatticeMach(std::sqrt(3.0) * lbm_u)`, or rounded: `si::LatticeMach(0.13)` for 0.075), the Reynolds number for the viscosity (`Fluid::WATER.with_viscosity(speed * length / reynolds)`), the Froude number with real gravity for free surfaces (`froude * au::sqrt(9.81 * m / (s * s) * depth)`), and the surface tension the lattice value is at that scale (`sim.converter().si_surface_tension(0.01)` computes it). The lattice setup then equals the given one; see `lid_driven_cavity`, `karman_vortex_street`, `river` and `cube_gravity`. Free-surface cases with a sub-cell water depth (`dam_break`, `breaking_waves`) keep a Reynolds number this way, because real water viscosity would need a much finer grid.

### 4. Boundary Conditions

```cpp
sim.boundaries()
    .set_solid_floor()
    .set_open_boundaries()
    .initialize_velocity_y(flow_velocity)
    .apply();
```
Faces are periodic unless set solid or open. `set_open_boundaries()` opens every face that is not set solid: with `set_solid_floor()` a wind tunnel, alone all six faces (an aircraft in free flow). `set_solid_sides()` closes the four X and Y faces, `set_solid_box()` all six; `set_periodic(Axis::Z)` keeps the Z faces periodic with open boundaries (a 2D domain). A wind profile: `set_wind_profile_power_law(10.0 * m / s, 100.0 * m, 0.25).set_wind_direction(Face::Y_MIN)`.

Free surfaces use `sim.surface()`, in metres and m/s (`hydraulic_jump`):
```cpp
sim.surface()
    .set_water_level(water_height)
    .initialize_hydrostatic() // with the simulation's gravity
    .set_solid_faces({Face::X_MIN, Face::X_MAX, Face::Y_MIN, Face::Z_MIN})
    .add_solid(Shape::box({0.0 * m, 0.0 * m, 0.0 * m}, {domain_x, socket_length, socket_height}))
    .add_inflow(Face::Y_MIN, inlet_velocity, socket_height, water_height)
    .add_outflow(Face::Y_MAX, outlet_velocity)
    .apply();
```
Positions are measured from the domain's origin corner. The water level, the inflows' heights and boxes cover whole cells (rounded); a bound in the last cell reaches the domain's end.

**Objects and regions** are `Shape`s in metres:
```cpp
sim.boundaries()                                                 // karman_vortex_street
    .add_solid(Shape::cylinder({ 4.0 * cm, 4.0 * cm, 0.5 * cell }, Axis::Z, 0.5 * cm, cell))
    .set_open_boundaries()
    .set_periodic(Axis::Z)                                       // a 2D domain, one cell high
    .initialize_velocity_y(flow_speed)
    .apply();

sim.surface()
    .set_water_level(depth)
    .add_water(Shape::sphere(drop, 0.5 * diameter), { si::Speed{}, side_speed, -fall_speed }) // a falling drop (raindrop)
    .add_gas(Shape::sphere(bubble, 2.0 * mm))                                    // a bubble (bursting_bubble)
    .add_solid(floor_and_ceiling & !hole)                                    // walls with a hole (periodic_faucet)
    .add_solid(Shape::half_space({ 0.5 * m, 1.0 * m, 0.0 * m }, si::Direction{ 0.0, -1.0, 8.0 })) // a sloped beach (breaking_waves)
    .apply();
```
The shapes are `sphere`, `cylinder` (axis, radius, length), `box` (two corners, whole cells), `triangle` (a plate about one cell thick), `torus` and `half_space` (one side of a plane, the side its normal points away from); `!a` is everything outside `a`, `a & b` the cells in both, `a | b` those in either. A cell belongs to a shape when its center does (cell i spans i to i+1 cell sizes); water and gas shapes are smooth at a sphere's surface.

Fields are functions of the position in metres: `initialize_velocity([](si::Position p) { return si::Velocity{...}; })` and `initialize_pressure()` for an analytic start (`taylor_green_2d`, `stokes_drag`), `add_moving_solid(shape, wall_velocity)` for a turning cylinder (`taylor_couette`), `add_moving_solid(shape, [](si::Position, si::Duration t) { ... })` for a wall whose velocity changes in time, set every time step (`liquid_metal`'s membrane), `set_force_field()` for a volume force per cell (FORCE_FIELD, `colliding_droplets`). They are evaluated at each cell's center. A solid whose force is measured: `add_solid(shape, Solid::MEASURED)` (`stokes_drag`).

Thermal walls use `sim.thermal()` (temperatures in Kelvin; the hydrostatic start uses the simulation's gravity axis), wave makers `sim.wave_maker()`, rotating parts `sim.parts()`:
```cpp
sim.parts()
    .add(MovingPart("rotor.stl")
        .set_rotation_axis(Axis::Y)
        .set_tip_speed(tip_speed)
        .centered_on_model(si::Position{ 0.0 * m, -0.21 * fan_diameter, 0.0 * m })) // only for an STL in other coordinates
    .initialize();                                       // after the boundaries; the parts turn as the simulation runs
```
A moving part gets the model's own transform (its rotation, its scale, its move to the planned center), so a rotor split from the same CAD file stays where it was on the model; a part in other coordinates is centered on the model instead. A part is re-voxelized, turned by the angle since its last update, whenever its tip has moved half a cell, or every `set_update_interval(0.8 * us)`. A tumbling part turns at an angular speed: `MovingPart(sim.model_file()).set_tumble(si::Direction{ 0.2, 1.0, 0.1 }, 180.0 * deg / s)` (`tie_fighter`). `ModelPlacement::of(sim.plan())` gives the same transform for meshes placed by hand (`mercedes_f1`).

### 5. Graphics and Video

```cpp
sim.graphics()
    .show_surface()
    .show_vortices()
    .set_camera(CameraView::orbit(-40.0 * deg, 25.0 * deg).field_of_view(70.0 * deg)) // also the interactive graphics' first view
    .apply();

sim.video() // written by run_for() when built with GRAPHICS but not INTERACTIVE_GRAPHICS
    .add(CameraView::orbit(-40.0 * deg, 20.0 * deg).field_of_view(78.0 * deg).view_height(domain_length / 1.25)) // export/
    .add("front", CameraView::at({ 20.0 * m, 2.7 * m, 15.0 * m }, -33.0 * deg, 42.0 * deg))                           // export/front/
    .add("pan", [](float progress) { return CameraView::orbit(-70.0 * deg + progress * 100.0 * deg, 2.0 * deg); }) // moving
    .set_length(10.0 * s);       // 60 frames per second of video, spread over the simulated time; or set_frame_interval(0.1 * s)
sim.run_for(10.0 * s);           // 10 simulated seconds
```
A camera's azimuth (about Z, from +X toward +Y) and elevation (above the horizontal) give the direction from what it sees to the camera. An `orbit()` camera circles the domain's center and `view_height()` is the length the frame's smaller side spans there; an `at()` camera stands at a position in metres from the domain's origin corner, and `look_at(target)` turns it. The graphics modes are `show_surface()`, `show_flags()`, `show_vortices()`, `show_velocity_field()`, `show_density_field()`, `show_streamlines()`, `show_free_surface()` and `show_free_surface_mesh()`, with `set_slice_mode(SliceMode::Z)` for the fields.

### 6. Running and Reading the Fields

`sim.run()` runs until a task stops it (with interactive graphics: until the window is closed); `sim.run_for(t)` runs t of simulated time in every build. Anything that changes as the simulation runs is a task in simulated time:
```cpp
sim.every(0.1 * s, [&](si::Duration t) { ... });   // at 0 s, then every 0.1 s of simulated time
sim.every_step([&](si::Duration t) { ... });       // every time step
sim.run_for(20.0 * s);                         // or run(): until a task calls sim.stop() (stokes_drag)
```
A task gets the simulated time since the start. `run_for()` continues from where the last run ended, so phases follow one another: `sim.set_body_force(...); sim.run_for(0.8 * s);` (`cube_gravity`). The video, the moving parts and the wave maker register their own tasks.

The fields are read as a snapshot in SI units:
```cpp
const FieldReader fields = sim.fields();
fields.for_each_cell([&](const FieldReader::Cell& c) {
    if(c.solid) return;
    const double speed = c.velocity.magnitude().in(m / s);
    error += sq(speed - analytic(c.center));                        // poiseuille_flow
});
const si::Pressure p = fields.pressure_at({ 1.25 * cm, 1.25 * cm, 2.5 * cm }); // the cell containing the point
```
`sim.forces()` gives the force on the measured solids in newtons and the drag coefficient (`ahmed_body`); the core's `LBM` is there for anything else (`sim.lbm()`), with `sim.converter()` to convert.

### 7. Atmospheric Convection

`setup/atmosphere/` describes a convective boundary layer in physical units: a column of air over a heated floor, under a lid, periodic across. It needs the fork's convection extensions `VOLUME_FORCE TEMPERATURE SUBGRID THERMAL_DIFFUSIVITY_FIX LEVEL_FORCING`, and `MOISTURE` for water. Use `SCALAR_FV` for heat and water (see [research/convection/README.md](research/convection/README.md)). The API has three layers. Use the highest one that does the job.

**High: a description and a run.** `CblDescription` is a value with documented defaults: Sullivan and Patton 2011's dry CBL at 10 m cells (0.24 K m/s, 300 K to 974 m, +8 K over 100 m, 3 K/km above, the lid at 2048 m, 1280 m square). Change it by copies:
```cpp
const CblDescription cbl = CblDescription()
    .with_cell(5.0 * m)
    .with_sounding(Sounding::from_csv("sounding.csv").with_reference_from_mixed_layer(1864.0 * m), 1864.0 * m)
    .with_extent({ 1920.0 * m, 1920.0 * m, LidRule::in_inversion_heights(2.0) })
    .with_surface({ 0.262 * K * m / s, HeatedPatch::disc(400.0 * m, 0.0 * m, PatchMode::SURFACE_LAYER), 0.5 })
    .with_sponge(SpongeProfile::quadratic(3000.0 * m, 450.0 * s))
    .with_wind(WindForcing{ PowerLawWind(5.0 * m / s, 100.0 * m, 0.25, 0.0 * deg), 600.0 * s })
    .with_moisture(true);
std::printf("%s\n", cbl.estimate(Simulation::build_lattice_memory(), std::nullopt).describe().c_str()); // before any device memory
ConvectiveBoundaryLayer run(cbl);   // validates, then builds the start, the heat sources, the sponge, the nudging, the moist air
run.observe(my_observer);           // an IntervalObserver: called after every interval
const RunSummary summary = run.run();
```
`validate()` names the first fault of these, in this order:
- a tabulated sounding has no theta_0
- the cell size is not a float exactly (the core's cell)
- the interval is shorter than half a time step
- the spin-up or the recorded time is not a whole number of intervals
- the box is less than one cell across
- a cell is larger than zi / 10
- the lid is not above zi
- the sponge does not start below the lid
- the patch is not inside the box
- a moist run has no total water in the sounding
- the domains split along z

`run()` prints a PROGRESS line per interval, the CLOUD lines of a moist run and the RESULT lines of the checks. A w that is not finite (NaN or infinite) or |w| over 40 m/s stops the program (FAILED). The example `convective_boundary_layer` is written on this layer.

**Middle: the parts.** `AtmosphereSimulation` owns the `Simulation` and its parts. Each part is usable alone:
```cpp
AtmosphereSimulation atm(cbl.grid(), sounding.reference(), CblNumerics{}, std::nullopt);
atm.start().sounding(sounding).noise(StartNoise::per_cell(0.1 * K, 300.0 * m, 2u)).apply();      // floor, lid, T, Q, u
atm.heat_sources().surface_flux(0.24 * K * m / s).patch(patch).apply();                          // TYPE_Y, the 4x patch
atm.level_forcing().add_term(std::make_unique<SpongeTerm>(SpongeProfile::none(), sounding));
atm.level_forcing().update(nullptr, std::nullopt);                                               // the core's level arrays
atm.add_check(std::make_unique<BlowUpGuard>());
const RunSummary summary = atm.run(cbl.schedule(), zi, std::nullopt, 0.24 * K * m / s);         // floor flux: the zi threshold
```
After each interval, the run does these steps in this order:
1. It reads the fields to the host.
2. It computes the layer means (`LevelStatistics`).
3. It finds the inversion height (and the mixed layer's top while it records).
4. It calls the observers.
5. It updates the level forcing (the nudging).
6. It prints the PROGRESS line.
7. It calls the checks.

A user term (a `LevelTerm`) adds to step 5, a user check (a `RunCheck`) to step 7. `fingerprint()` hashes the set-up before the first run: two set-ups with the same hash give the same run.

**Low: the core.** `atm.lbm()` and `run.lbm()` give the core for expert use, and `LatticeParameters::of(description, build)` lists the numbers it gets. A part's next update overwrites a direct write to the state that the part owns (the level arrays, the moist terms).

**The values.** Unit tests check each value alone on the CPU, and the layers above use them unchanged:
- the column: `Sounding` (idealized, tabulated, CSV), `TemperatureProfile`, `WaterContentProfile` and `SpeedProfile` (on one `ProfileTable` of plain numbers), `LevelGrid` and `AtmosphereGrid` (the heights of the cell centres above the floor's top), `MoistColumn` (the hydrostatic pressure, theta_v, the saturation adjustment)
- the forcing: `PowerLawWind` (a `WindTarget`), `SpongeProfile`, `StartNoise`, `HeatedPatch`, `WindNudging`
- the diagnostics and the checks: `BoundaryLayerDiagnostics`, `CloudExtent`, `CloudBaseCheck`, `BlowUpCheck`. One converter, `si::LatticeConverter` (the cell, the time step, theta_0 as the lattice temperature 1 with 1 K per unit, 0.01 kg/kg per unit of the lattice moisture), gives every lattice number. `BoussinesqBuoyancy` gives the carrier force fz and beta = g dt^2 / (dx theta_0 |fz|).

**Cell sizes.** Every length is in metres, so a description holds at any cell size. `TimeStepRule::constant_lattice_speed(0.04 * s, 2.5 * m)` keeps dx/dt at 62.5 m/s. The cost grows as dx^-4: `estimate()` gives the cells, the device memory per domain (with the halo) and the steps before a run. Domains split along x or y, never along z. The numerical floors stay lattice values: the viscosity 1e-5, and the scalars' 1e-4 with `SCALAR_FV` or 0.014 on the D3Q7 lattice. They scale with the grid, as an LES regularisation does. `Diffusivity::physical()` sets a physical value instead. `StartNoise::blocks()` gives the same start field at every cell size that divides the block.

**Reproducible bits.** The Setup library, its unit tests and the convective boundary layer example compile without fast math (the target property `FLUIDX3D_PRECISE_FLOAT`, see `cmake/configs.cmake`). So their set-up values round as written, the same on every compiler. A program that compares set-ups bit for bit gives its own target the same property. The core and the kernel keep fast math.

---

## Units and Symbols

Every value with a unit is an `si::` type of the Au units library (`lib/setup/units/units.hpp`). The values are
double. Write a value as a number times a symbol, with `using namespace si::symbols;` in the function:
```cpp
using namespace si::symbols;
const si::Length lid = 2048.0 * m;
const si::Speed cruise = 300.0 * kmh;
const si::KinematicHeatFlux flux = 0.24 * K * m / s;
const si::LapseRate lapse = 3.0 * K / km;
const si::Temperature theta_0 = si::kelvin_point(300.0);
const si::MemoryAmount budget = 2000.0 * MiB;
```

| Quantity | `si::` type | Symbols |
|----------|-------------|---------|
| Length, area, volume | `Length`, `Area`, `Volume` | `m`, `cm`, `mm`, `km` |
| Duration, frequency | `Duration`, `Frequency` | `s`, `ms`, `us`, `Hz`; minutes and hours: `au::minutes(2.0)`, `au::hours(1.0)` |
| Speed, acceleration | `Speed`, `Acceleration` | `m / s`, `kmh`, `m / (s * s)` |
| Mass, density | `Mass`, `Density` | `kg`, `kg / (m * m * m)` |
| Kinematic viscosity, diffusivity | `KinematicViscosity` | `m * m / s` |
| Volume flow rate | `VolumeFlowRate` | `m * m * m / s` |
| Force, pressure, surface tension | `Force`, `Pressure`, `SurfaceTension` | `N`, `Pa`, `hPa`, `N / m` |
| Temperature reading | `Temperature` | `si::kelvin_point(300.0)`, `si::celsius_point(20.0)` |
| Temperature difference | `TemperatureDifference` | `K` |
| Lapse rate, kinematic heat flux | `LapseRate`, `KinematicHeatFlux` | `K / km`, `K * m / s` |
| Water content, moisture flux | `WaterContent`, `MoistureFlux` | `g_w / kg`, `kg_w / kg * m / s` |
| Energy, specific and latent heat | `SpecificHeat`, `LatentHeat` | `J`, `J / (kg * K)`, `J / kg_w` |
| Angle, rotation rate | `Angle`, `AngularSpeed` | `deg`, `rad`, `deg / s` |
| Device memory | `MemoryAmount` | `MiB`, `GiB` |
| Lattice speed of a build | `LatticeUpdateRate` | `MLUPs` |

Rules:
- Write a double number: `2.0 * m`, not `2 * m`. An int number makes a quantity that stores an int, and a float
  number one that stores a float.
- A temperature reading is a point on the scale, a temperature difference is a quantity. A reading minus a reading
  is a difference; a reading plus a reading does not compile. `si::in_kelvin(t)` and `si::in_celsius(t)` give the
  number of a reading.
- `MiB` is 2^20 bytes and `GiB` 2^30 bytes: the core's MB and GB. There is no `MB` symbol.
- `.in(unit)` gives the number in a unit (`length.in(m)`); `au::as_raw_number()` gives the number of a ratio of
  the same dimension (`au::as_raw_number(length / cell)`).
- Quantities multiply and divide into new dimensions (`10.0 * m / (2.0 * s)` is a speed) and scale with plain
  numbers (`0.5 * domain_length`). A length where a speed is expected does not compile.
- `si::Vector3<Q>` holds three components of one quantity: `si::Position`, `si::Velocity`, `si::AccelerationVector`,
  `si::ForceVector`, and `si::Direction` for a direction without a unit. It has `magnitude()`, `+`, `-` and a scale
  by a number or a quantity (a velocity times a duration is a position).
- `si::LatticeMach(0.13)` is the lattice Mach number: a numerical choice, not a physical value.
- A local variable named like a symbol (`N`, `s`, `m`, `K`) hides the symbol under `using namespace si::symbols;`,
  and MSVC warns (C4459). Give the variable a descriptive name.
- There is no symbol for minutes or hours: `min` is ambiguous with the core's `min()` functions.

`si::LatticeConverter` (`sim.converter()`) is the only place where a value becomes one of the core's floats. It
holds the cell, the time step (the float the core holds), the reference density and the temperature scale.

---

## Testing a Setup

`tests/physics/` holds simulations checked against analytic solutions (`ctest -L physics`): a `main_setup()` that ends with `PhysicsCheck::report()`, registered with `fluidx3d_add_physics_test()`. A new check of this kind is the way to test a setup. See [CMAKE.md](CMAKE.md#tests).

---

## Appendix: Coming from the Low-Level API

The core's own examples configure everything in lattice units and cell indices. Their calls map to the API as follows.

| Low-level | Setup API |
|-----------|-----------|
| `#define` lines in `defines.hpp` | `EXTENSIONS` in the example's `CMakeLists.txt` |
| `resolution(float3(1, 5, 0.75), 2000u)` | `Domain::box(1.0 * m, 5.0 * m, 0.75 * m).vram(2000.0 * MiB)` |
| `resolution(...)` + `lbm_length = 0.65f*lbm_N.y` | `Domain::around(Model(file).length(2.4 * m)).size(x, y, z).vram(1000.0 * MiB)` |
| `read_stl(...)` + `mesh->translate(...)` | `.gap_to_inlet(...)`, `.gap_to_floor(...)` or `.model_offset(...)` |
| `units.set_m_kg_s(...)`, `LBM lbm(lbm_N, lbm_nu)` | `Simulation sim(domain, Fluid::AIR, 10.0 * m / s)`; an `lbm_u` of 0.075 is `si::LatticeMach(0.13)` |
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
| `next_frame(lbm_T, 30.0f)` in the run loop, inside `#if defined(GRAPHICS) && !defined(INTERACTIVE_GRAPHICS)` | `sim.video().set_length(30.0 * s)` and `sim.run_for(time)`, no `#if` |
| `lbm.run(n)` loops with `lbm.get_t()` | `sim.every(...)` tasks and `sim.run_for()` |
| `lbm.u.read_from_device()` and index arithmetic | `sim.fields()` |
| `beta = g dt^2 / dx / (theta_0 * -fz)`, `LBM(..., fz, ..., alpha, beta)` | `BoussinesqBuoyancy(9.81 * m / (s * s), theta_0)` with `sim.set_boussinesq_buoyancy()`; `CblNumerics` in the atmosphere layer |
| `lbm.set_heat_rate(flux * dt / dx)`, `flags = TYPE_Y` (`TYPE_X\|TYPE_Y` for 4x) | `atm.heat_sources().surface_flux(0.24 * K * m / s).patch(HeatedPatch::disc(...)).apply()` |
| `lbm.set_level_forcing(fx, fy, rate, T_ref)` | `LevelForcing` with `SpongeTerm`, `NudgingTerm` or a user `LevelTerm` |
| `lbm.set_level_moist(p, theta_v)`, `set_theta_0()`, `set_moisture_rate()` | `atm.moisture(MoistColumn::hydrostatic(sounding, levels), flux)` |
| `T[n] = 1 + theta' + noise`, `Q[n] = 1 + qt / 0.01` per cell | `atm.start().sounding(s).noise(StartNoise::per_cell(...)).moisture(true).apply()` |
