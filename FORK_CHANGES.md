# What this fork changes, and what it found

This file lists the fork's own work on top of ProjectPhysX/FluidX3D (v3.8 + 7). Each entry names the change and the
measurement that justifies it. The convection work has its own page with the method, the results and the
references: [research/convection/README.md](research/convection/README.md). The findings for upstream are in
`upstream_issues.md`.

## Defects found and fixed

| Defect | Where | State |
|---|---|---|
| The thermal lattice diffuses with `alpha / 2`. The code sets `def_w_T = 1 / (2 alpha + 1/2)`, but the D3Q7 diffusivity is `(tauT - 1/2) / 4`. A decaying temperature wave gives 0.5007 alpha. | `src/lbm.cpp` | Fixed behind `THERMAL_DIFFUSIVITY_FIX` (`def_w_T = 1 / (4 alpha + 1/2)`, measured 1.000 alpha). The stock rate stays the default, so the upstream examples run as upstream. A candidate for an upstream issue. |
| `LBM::initialize()` did not write the moisture field Q to the device. | `src/lbm.cpp` | Fixed. |
| The example CMake function wrote the `defines.hpp` of every target into one `defines/` folder. Two targets in one example folder (FP32 and FP16S) used the same file, and the last one won without a message. | `cmake/FluidX3DExample.cmake` | Fixed: `defines_<target>/`. |

## Limits found (documented, not changed)

| Limit | Where | State |
|---|---|---|
| Next to `TYPE_S` walls in a buoyant flow at `tauT` close to 0.5, the thermal lattice creates heat and negative temperatures: +39 % of the added heat in 2000 steps, T down to -0.05. The error gets smaller as `tauT` increases, and it does not occur without walls. With the force-corrected thermal equilibrium of upstream v3.8, the budget at `tauT` 0.508 to 0.528 is within 3.3 %. At 0.503 it still fails (-18 %, negative T). | `tests/physics/heat_conservation.cpp` | Every thermal set-up of this fork uses `tauT >= 0.556`. A runtime warning for `TEMPERATURE` with `VOLUME_FORCE` below `tauT` 0.52 is not implemented (open). |
| `FP16S` with `TEMPERATURE`: the thermal DDFs do not diffuse a wave at rest or across a flow at `alpha` 2e-3 (D = 0). The range-shifted half floats overflow for T above about 8, so every cell above a sounding's inversion reads inf. | `tests/physics/thermal_diffusivity_fp16s.cpp` | The two FP16S tests are registered `WILL_FAIL`. A runtime warning for the combination is not implemented (open). The upstream `rayleigh_benard` and `thermal_convection` examples use this combination. |
| The D3Q7 scalar lattices (heat and water) do not keep a uniform scalar uniform in a flow that is not divergence-free on 7 points. In 2700 s of a convective boundary layer at 10 m cells, a uniform Q drifts by up to 8 %. A host reference gives the same result as the GPU, bit for bit. So this is a property of the linear-equilibrium scheme, not a code defect. | `tests/physics/scalar_reference.cpp` | `SCALAR_FV` carries heat and water by finite volumes instead (below). |
| A finite-volume scalar in advective form on the LBM velocity is not conservative with buoyancy. In the plume heat-budget case the heat grows by 28 % (161 % without eddy diffusivity). | `heat_conservation_fv` | `SCALAR_FV` uses the conservative transport by the LBM's link mass fluxes. On the GPU the budget holds within 0.1 % in every case of the test. |
| The link-flux finite volumes diffuse more across a flow at high cell Peclet numbers. The diagonal links carry mass in both directions, and there the limiter changes the scheme to first order. At alpha 2e-3 and u 0.1 (Peclet 50), D/alpha is 1.074 across the flow and 1.305 along it. At alpha 0.014 it is 1.007 and 1.026. | `tests/physics/scalar_fv_cross_lab.cpp`, `thermal_diffusivity_fv` | Documented. The CBL cases use alpha 0.014. `thermal_diffusivity_fv` and `eddy_heat_diffusivity_fv` record the failures at alpha 2e-3 and 1e-4. |
| The `WIND` line of `convective_boundary_layer` printed "(limit 5 %)" for the deviation from the power-law target of the nudging, but it checked nothing. With tau 600 s the nudged wind misses that target by 27 to 38 % below zi, as MicroHH's nudged wind does. | `examples/convective_boundary_layer/main.cpp` | Fixed: the line names its target and prints the deviation as information. The wind is checked against MicroHH's nudged profile (within 6.6 %). |

## New functionality

| Extension / API | What it does | Its test |
|---|---|---|
| `THERMAL_DIFFUSIVITY_FIX` | Makes `alpha` the diffusivity. | `thermal_diffusivity`, `thermal_diffusivity_fixed` |
| `TYPE_Y` heat source, `TYPE_X` as a 4x marker, `LBM::set_heat_rate()` | Adds a heated layer or patch as a flux boundary, instead of a fixed temperature. | `heat_source`, `heat_conservation` |
| `LBM::set_inv_prandtl_t()` with `SUBGRID` | Lets the Smagorinsky eddy viscosity mix heat (and water) too, at `Pr_t`. | `eddy_heat_diffusivity` |
| `LEVEL_FORCING`, `LBM::set_level_forcing()`, `set_level_moist()` | Applies terms per horizontal level: a force (nudging toward a wind profile), a damping of w and a temperature relaxation (a sponge under a lid), the buoyancy against the level's reference profile (no hydrostatic ramp), and the pressure and reference theta_v for the moisture. | `level_forcing` |
| `MOISTURE`, `LBM::set_moisture_rate()`, `set_theta_0()` | Carries total water as a second scalar, with the thermal diffusivity and a surface source. The buoyancy comes from the virtual potential temperature after a saturation adjustment in each cell (Newton on the liquid-water potential temperature). `src/moist_air.hpp` is the host version. | `moisture_source`, `moist_buoyancy`, `moist_bubble` |
| `DEBUG_PROBE`, `LBM::debug_cells()`, `debug_trace()`, `set_trace_cell()`, `debug_populations()` | Writes the state of every cell of a step to a probe buffer, the populations of one cell to a ring of 256 steps, and the DDFs to the host (one domain). | `scalar_reference`, the host references below |
| `SCALAR_FV` | Carries heat and water by finite volumes in conservative form (MUSCL, van Albada, midpoint step), beside the LBM flow. The LBM's own link mass fluxes move them. Runs in any number of domains: with `SCALAR_FV` the domain halo is 2 cells wide. | `scalar_fv_kernel_check`, `scalar_fv_wall_exchange`, `scalar_fv_cross_lab`, `scalar_fv_lab`, `scalar_fv_domains`, the `*_fv` physics tests |
| `LBM::time_fv_step()` (with `SCALAR_FV`) | Runs one time step and returns the time of `stream_collide` and of the two finite-volume stages, each measured alone. | `scalar_fv_timing` |
| Host references in `tests/physics/scalar_fv_reference.hpp` | `FiniteVolumeScalar` (advective), `ConservativeFiniteVolumeScalars` (own density, 7-point fluxes), `LinkFluxFiniteVolumeScalars` (the LBM's link mass fluxes, the twin of `SCALAR_FV`). | `scalar_fv_conservative` (records a failure, `WILL_FAIL`), `scalar_fv_link_flux`, `scalar_fv_kernel_check` |
| The example target `convective_boundary_layer_fv` | The example with `SCALAR_FV`, in any number of domains. The default `alpha` is 1e-4. | the dry CBL against the D3Q7 build and MicroHH |
| The example target `convective_boundary_layer_fv_fp16s` | The same with `FP16S` storage of the DDFs: 95 bytes per cell (FP32: 133). The scalars stay FP32. | the dry CBL against FP32, inside the seed spread |
| The example `convective_boundary_layer` | Simulates a dry or moist convective boundary layer in physical units: a sounding (idealized or tabulated), the surface flux, an optional heated patch, the lid rule with a sponge, the wind by nudging, and the statistics and frames of the flow in a box. The options `--alpha` (the molecular diffusivity), `--tracer` (a uniform passive scalar in Q) and `--moist` with `--zi` (total water from the sounding, the floor's moisture flux, the clouds). | the physics tests above, the dry and the moist CBL against MicroHH |
| The thermal validation suite in `tests/physics` | Tests Rayleigh-Benard onset and Nu(Ra) with grid refinement, a plume, the heat budget, the diffusivity, the per-level terms, the moisture and the scalar transport. | `ctest -L physics` |

## Examples worth adding (not done)

- A Rayleigh-Benard Nusselt-number example in physical units, made from `tests/physics/rayleigh_benard_case.hpp`. It
  prints Nu against Kerr 1996 and notes the first-order wall convergence (15 % high with 3 to 4 cells per thermal
  layer).
- A nudged channel with a sponge: the smallest demonstration of `LEVEL_FORCING`.
- A moist thermal: the `moist_bubble` test as an example with graphics.

## Measured costs worth knowing

- Two domains along x are bit-identical to one (18 750 steps with temperature). The split does not change the result.
- With `SCALAR_FV`, 2 x 1, 1 x 2 and 2 x 2 domains are bit-identical to one (`scalar_fv_domains`). Each step
  has 4 exchanges through the host: the DDFs and 3 finite-volume fields. On one GPU, two domains of a 10 m CBL
  (320 x 320 x 375) take 48 % more time than one.
- A 5 m CBL with `SCALAR_FV` and `FP16S` (480 x 480 x 748, 172 M cells) runs in two domains on one RTX 4090 at
  1660 MLUPs: 45 minutes of model time (33 750 steps) in 58 minutes.
- The buoyancy force cap: with the absolute buoyancy `|fz| beta (T - 1)`, a tall stable layer reaches the lattice's
  force limit (1e-3) at about 2.3 zi for a 10 m convective layer. The reference-profile buoyancy removes this cap.
- A thermal layer that has only 3 to 4 cells carries 15 to 40 % too much heat (first-order wall treatment). For 5 %,
  a layer needs 6 to 7 cells.
- `SCALAR_FV` with moisture needs 132 bytes per cell (the D3Q7 lattices: 157). The D3Q7 buffers are one-cell
  placeholders. On 13.57 M cells of a CBL a step takes 7.3 ms (D3Q7: 4.4 ms). Of this, `stream_collide` takes 2.5 ms
  and each of the two finite-volume stages 2.4 ms.
