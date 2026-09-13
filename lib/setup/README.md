# Setup API (`lib/setup`)

The Setup API describes a FluidX3D simulation in physical units. Examples include `setup/setup.hpp`; the
[README](../../README.md#setup-api) lists the components, [SETUP_API.md](../../SETUP_API.md) shows how to write a
simulation with them.

## Layout

| Folder | Contents | Built |
|--------|----------|-------|
| `core/` | Quantities and literals (`quantity.hpp`), `UnitScale`, `TemperatureScale`, `Fluid`, `SetupError` | library |
| `domain/` | `Domain`, `Model`, `DomainPlanner`, `Shape`, lattice sizing (`lattice.hpp`), `ModelPlacement`, `GeometryScaler` | library |
| `graphics/` | `CameraView` (library); `GraphicsConfig`, `VideoRecorder` | library, with each example |
| `sdf/` | The cached SDF of an STL model | library |
| `simulation/` | `StepSchedule` (library); `Simulation`, `Runner`, `FieldReader`, `MeshLoader` | library, with each example |
| `boundaries/`, `surface/`, `moving/`, `particles/`, `analysis/` | The builders that write to the LBM | with each example |

The static library `fluidx3d::setup` holds the code that does not need the LBM: it is compiled once and unit-tested on
the CPU. The headers that use the LBM depend on each example's configuration (the extensions and velocity set of its
`add_fluidx3d_example()`, which generates its `defines.hpp`), so they are compiled with the example.

## Conventions

- Public setters take quantities (`Length`, `Speed`, `Acceleration`, `Duration`, ...), which convert to SI floats and
  time steps only inside the API. The one numerical choice left to the user is dimensionless: the lattice Mach number
  (`LatticeMach`) of the `Simulation`, which sets the time step.
- `Simulation` owns the core's `LBM`, creates it on its first use with the physics set before (gravity, surface
  tension, temperatures, particles), and hands out the builders and the components that act while it runs. There is
  no call order to remember: a physics setting after the first use is an error, not wrong physics.
- Positions are in metres from the domain's origin corner. Cell i spans i to i+1 cell sizes, its center at i+0.5; the
  core's cell coordinates put cell i's center at i, so its `lbm.center()` is the domain's center. Shapes, fields and
  the wind profile are evaluated at cell centers.
- The builders convert with the simulation's `UnitScale`, which they are given; the core's global `units` is set once,
  for its labels and file output.
- Every header includes what it uses and compiles on its own (checked by `tests/headers/`); a header that needs an
  extension (`SURFACE`, `TEMPERATURE`, `FORCE_FIELD`, `PARTICLES`, `GRAPHICS`) stops with an `#error` that names it.
- Conflicting or impossible settings throw `SetupError` in the library; `Simulation` reports them with the core's
  `print_error()`, which stops the program.

## Tests

`ctest -L unit` runs the library's unit tests (GoogleTest) and `ctest -L physics` simulations set up with the API
against analytic solutions (`tests/physics/`); the build compiles every header on its own (`tests/headers/`).
See [CMAKE.md](../../CMAKE.md#tests).
