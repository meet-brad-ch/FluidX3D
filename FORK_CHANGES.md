# What this fork changes

This file lists the fork's convection work on top of ProjectPhysX/FluidX3D (v3.8 + 7). Each entry names the change and
the measurement that supports it. How to use the features, their limits and the validation are on
[research/convection/README.md](research/convection/README.md). The findings for upstream are in `upstream_issues.md`.

## Defects fixed

| Defect | Where | State |
|---|---|---|
| The thermal lattice diffuses with `alpha / 2`. The code sets `def_w_T = 1 / (2 alpha + 1/2)`, but the D3Q7 diffusivity is `(tauT - 1/2) / 4`. A decaying temperature wave gives 0.5007 alpha. | `src/lbm.cpp` | Fixed behind `THERMAL_DIFFUSIVITY_FIX` (`def_w_T = 1 / (4 alpha + 1/2)`, measured 1.000 alpha). The stock rate stays the default, so the upstream examples run as upstream. A candidate for an upstream issue. |
| The example CMake function wrote the `defines.hpp` of every target into one `defines/` folder. Two targets in one example folder (FP32 and FP16S) used the same file, and the last one won without a message. | `cmake/FluidX3DExample.cmake` | Fixed: `defines_<target>/`. |
| The Setup API's `Simulation` gave the core the fluid's thermal diffusivity as `alpha`. Without `THERMAL_DIFFUSIVITY_FIX` the lattice diffuses with `alpha / 2`, so every Setup-API thermal simulation without the fix had half its diffusivity. | `lib/setup/simulation/simulation.hpp` | Fixed: the core gets `ScalarTransportRules::core_alpha()` (D with the fix, 2 D without), measured in the 3 builds (`thermal_diffusivity_api`). The upstream examples `rayleigh_benard` and `thermal_convection` build without the fix and use `set_temperatures()`. They now diffuse with the fluid's diffusivity, twice their earlier value. |
| The Release build compiles with fast math and link-time code generation. One float expression can then round differently at each place where the compiler inlines it. The convective boundary layer example's dt at 10 m was 0x1.47ae16p-3 s (the compiler's `cell*(0.04f*0.4f)`), not the written `0.04f*cell/2.5f` (0x1.47ae14p-3). One API rule gave both values in one program. | `cmake/configs.cmake` | Fixed for the set-up code: a target with the property `FLUIDX3D_PRECISE_FLOAT` gets no fast math (GCC also `-ffp-contract=off`). The Setup library, its unit tests and the convective boundary layer example use it. The core and the kernel keep fast math. |
| The Setup API's power-law wind called `pow(float, float)`, which is ambiguous between `pow(double, double)` and `pow(float, uint)` of `utilities.hpp`. The `uint` overload truncates the exponent to 0, and the wind profile is then uniform. | `lib/setup/boundaries/boundary_builder.hpp` | Fixed: `std::pow`. |

## Limits (documented, not changed)

| Limit | Where | State |
|---|---|---|
| Next to `TYPE_S` walls in a buoyant flow at `tauT` close to 0.5, the D3Q7 thermal lattice creates heat and negative temperatures: +39 % of the added heat in 2000 steps, T down to -0.05. The error gets smaller as `tauT` increases, and it does not occur without walls. At `tauT` 0.508 to 0.528 the budget is within 3.3 %. | the D3Q7 thermal lattice | Use `SCALAR_FV`, or `tauT >= 0.556`. A runtime warning for `TEMPERATURE` with `VOLUME_FORCE` below `tauT` 0.52 is not implemented. |
| `FP16S` with the D3Q7 `TEMPERATURE` lattice: the thermal DDFs do not diffuse a wave in a flow at `alpha` 2e-3 (D = 0). The range-shifted half floats overflow for T above about 8. | the D3Q7 thermal lattice | Use FP32 with D3Q7, or `SCALAR_FV`. A runtime warning is not implemented. The upstream `rayleigh_benard` and `thermal_convection` examples use this combination. |
| The D3Q7 scalar lattices (heat and water) do not keep a uniform scalar uniform in a flow that is not divergence-free on 7 points. In 2700 s of a convective boundary layer at 10 m cells, a uniform Q drifts by up to 8 %. A host reference of the lattice gives the same result as the GPU, bit for bit. So this is a property of the linear-equilibrium scheme. | the D3Q7 scalar lattices | `SCALAR_FV` carries heat and water by finite volumes instead. |
| `SCALAR_FV` diffuses more across a flow at high cell Peclet numbers. The diagonal links carry mass in both directions, and there the limiter changes the scheme to first order. At alpha 2e-3 and u 0.1 (Peclet 50), D/alpha is 1.074 across the flow and 1.305 along it. At alpha 0.014 it is 1.007 and 1.026. | `SCALAR_FV` | Documented. `thermal_diffusivity_fv` and `eddy_heat_diffusivity_fv` print these cases as information. |
| With tau 600 s the nudged wind misses the power-law target of the nudging by 27 to 38 % below zi, as MicroHH's nudged wind does. | `LEVEL_FORCING` | The example's `WIND` line prints the deviation as information. The wind is checked against MicroHH's nudged profile (within 6.6 %). |

## New functionality

| Extension / API | What it does | Its test |
|---|---|---|
| `THERMAL_DIFFUSIVITY_FIX` | Makes `alpha` the diffusivity. | `thermal_diffusivity`, `thermal_diffusivity_fix` |
| `TYPE_Y` heat source, `TYPE_X` as a 4x marker, `LBM::set_heat_rate()` | Adds a heated layer or patch as a flux boundary, in place of a fixed temperature. | `heat_source` |
| `LBM::set_inv_prandtl_t()` with `SUBGRID` | Lets the Smagorinsky eddy viscosity mix heat (and water) too, at `Pr_t`. | `eddy_heat_diffusivity` |
| `LEVEL_FORCING`, `LBM::set_level_forcing()`, `set_level_moist()` | Applies terms per horizontal level: a force (nudging toward a wind profile), a damping of w and a temperature relaxation (a sponge under a lid), the buoyancy against the level's reference profile (no hydrostatic ramp), and the pressure and reference theta_v for the moisture. | `level_forcing` |
| `MOISTURE`, `LBM::set_moisture_rate()`, `set_theta_0()` | Carries total water as a second scalar, with the thermal diffusivity and a surface source. The buoyancy comes from the virtual potential temperature after a saturation adjustment in each cell (Newton on the liquid-water potential temperature). `src/moist_air.hpp` is the host version. | `moisture_source`, `moist_buoyancy`, `moist_bubble_fv` |
| `SCALAR_FV` | Carries heat and water by finite volumes in conservative form (MUSCL, van Albada, midpoint step), beside the LBM flow. The LBM's own link mass fluxes move them. Runs in any number of domains: with `SCALAR_FV` the domain halo is 2 cells wide. | the `*_fv` tests, `heat_conservation_fv`, `scalar_fv_domains` |
| The Setup API's atmosphere classes (`lib/setup/atmosphere`, SETUP_API.md section 7) | Describe and run a convective boundary layer in physical units, in three layers. High: `CblDescription` and `ConvectiveBoundaryLayer`. Middle: `AtmosphereSimulation` with its builders, the per-level forcing, the moist air, the interval pipeline and the checks. Low: the core (`lbm()`, `LatticeParameters`). Unit tests check the values (`Sounding`, `MoistColumn`, `PowerLawWind`, `SpongeProfile`, `StartNoise`, `HeatedPatch`, `BoundaryLayerDiagnostics`, the checks) on the CPU. | their unit tests (`tests/unit`, each value bit for bit against the verified example where it keeps its float order). `atmosphere_column` and `atmosphere_profiles` (the set-up's fingerprint and `profiles.csv` of 5 cases equal the verified example's) |
| The example `convective_boundary_layer` (targets `convective_boundary_layer_fv`, `convective_boundary_layer_fv_fp16s`) | Simulates a dry or moist convective boundary layer through `ConvectiveBoundaryLayer`: a sounding (idealized or tabulated), the surface flux, an optional heated patch, the lid rule with a sponge, the wind by nudging, and the horizontal-mean statistics (`profiles.csv`). The options `--diffusivity` (the scalars' lattice diffusivity), `--moist` with `--zi` (total water from the sounding, the floor's moisture flux, the clouds) and `--estimate`. It writes `profiles.csv` and its result lines, and no frames. `FP16S` DDFs: 95 bytes per cell (FP32: 133). | the tests above, the dry and the moist CBL against MicroHH, and the example against its verified version on 5 cases |

## The Setup API on the Au units library

| Change | What it means for a user |
|---|---|
| The Setup API's quantities are the `si::` types of the Au units library (aurora-opensource/au 0.6.1, fetched by CMake), in double. They replace the float quantities and the user-defined literals (`2.0_m`). | A value is a number times a symbol: `2.0 * m`, `10.0 * m / s`, `300.0 * kmh`, `2000.0 * MiB`, with `using namespace si::symbols;`. A temperature reading is `si::kelvin_point(300.0)`, a difference `1.0 * K`. SETUP_API.md, section "Units and Symbols", lists the types and the symbols. |
| One converter, `si::LatticeConverter`, makes every lattice number the core gets, in double with one cast to float. It holds the cell (a float exactly), the time step (as the core's float), the reference density and the temperature scale. | `sim.converter()` gives it. `set_temperature_scale(reference, unit)` takes a reading and a difference. |
| Vectors are `si::Position`, `si::Velocity`, `si::AccelerationVector`, `si::ForceVector` and `si::Direction` (`si::Vector3`); the lattice Mach number is `si::LatticeMach`. | A field is a function of an `si::Position`; a task gets an `si::Duration`. |
| The API computes each lattice value in double and casts it once. A value that it computed in float before can differ in the last bit of the float. | The model voxelization keeps the core's float order: the rotation angles go to the core as float degrees. |

## Measured costs worth knowing

- Two domains along x are bit-identical to one (18 750 steps with temperature). The split does not change the result.
- With `SCALAR_FV`, 2 x 1, 1 x 2 and 2 x 2 domains are bit-identical to one (`scalar_fv_domains`). Each step
  has 4 exchanges through the host: the DDFs and 3 finite-volume fields. On one GPU, two domains of a 10 m CBL
  (320 x 320 x 375) take 48 % more time than one.
- A 5 m CBL with `SCALAR_FV` and `FP16S` (480 x 480 x 748, 172 M cells) runs in two domains on one RTX 4090 at
  1660 MLUPs. 45 minutes of model time (33 750 steps) take 58 minutes.
- The buoyancy force cap: with the absolute buoyancy `|fz| beta (T - 1)`, a tall stable layer reaches the force limit
  of the lattice (1e-3). For a 10 m convective layer this occurs at about 2.3 zi. The reference-profile buoyancy
  removes this cap.
- A thermal layer that has only 3 to 4 cells carries 15 to 40 % too much heat (first-order wall treatment). For 5 %,
  a layer needs 6 to 7 cells.
- `SCALAR_FV` with moisture needs 132 bytes per cell (the D3Q7 lattices: 157). On 13.57 M cells of a CBL a step takes
  7.3 ms (D3Q7: 4.4 ms).
