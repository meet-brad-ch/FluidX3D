# Setup API (`lib/setup`)

The Setup API describes a FluidX3D simulation in physical units. Examples include `setup/setup.hpp`; the
[README](../../README.md#setup-api) lists the components, [porting_guide.md](../../porting_guide.md) maps the core's
calls to them.

## Layout

| Folder | Contents | Built |
|--------|----------|-------|
| `core/` | Quantities and literals (`quantity.hpp`), `UnitScale`, `TemperatureScale`, `Fluid`, `SetupError` | library |
| `domain/` | `Domain`, `Model`, `DomainPlanner`, `Shape`, lattice sizing (`lattice.hpp`), `ModelPlacement`, `GeometryScaler` | library |
| `graphics/` | `CameraView` (library); `GraphicsConfig`, `VideoRecorder` | library, with each example |
| `sdf/` | The cached SDF of an STL model | library |
| `simulation/` | `StepSchedule` (library); `SimulationSetup`, `MeshLoader`, `Runner` | library, with each example |
| `boundaries/`, `surface/`, `moving/`, `particles/`, `analysis/` | The builders that write to the LBM | with each example |

The static library `fluidx3d::setup` holds the code that does not need the LBM: it is compiled once and unit-tested on
the CPU. The headers that use the LBM depend on each example's configuration (the extensions and velocity set of its
`add_fluidx3d_example()`, which generates its `defines.hpp`), so they are compiled with the example.

## Conventions

- Public setters take quantities (`Length`, `Speed`, `Acceleration`, `Duration`, ...), which convert to SI floats and
  time steps only inside the API. The one numerical choice left to the user is dimensionless: the lattice Mach number
  (`LatticeMach`) of `SimulationSetup::configure_units()`, which sets the time step.
- Positions are in metres from the domain's origin corner. Cell i spans i to i+1 cell sizes, its center at i+0.5; the
  core's cell coordinates put cell i's center at i, so its `lbm.center()` is the domain's center. Shapes, fields and
  the wind profile are evaluated at cell centers.
- The builders convert with the global `units` that `configure_units()` sets.
- Every header includes what it uses and compiles on its own (checked by `tests/headers/`); a header that needs an
  extension (`SURFACE`, `TEMPERATURE`, `FORCE_FIELD`, `PARTICLES`, `GRAPHICS`) stops with an `#error` that names it.
- Conflicting or impossible settings throw `SetupError` in the library; `SimulationSetup` reports them with the core's
  `print_error()`, which stops the program.

## Tests

`ctest -L unit` runs the library's unit tests (GoogleTest), `ctest -L baseline` checks each example's setup against
`tests/baselines/`, `ctest -L physics` runs simulations set up with the API against analytic solutions (`tests/physics/`),
and `ctest -L original` the original examples (configured with `-DFLUIDX3D_BUILD_ORIGINALS=ON`).
See [CMAKE.md](../../CMAKE.md#tests).
