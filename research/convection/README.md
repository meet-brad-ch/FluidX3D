# Atmospheric convection in FluidX3D

FluidX3D-API adds thermal convection and moist air to FluidX3D. The target is the convective boundary layer (CBL) of
the atmosphere: a heated ground, a capping inversion, a mean wind and cumulus clouds. This page tells you how to use
the features and what their limits are. It also compares the results with the large-eddy simulation (LES) code MicroHH
(van Heerwaarden et al. 2017). All numbers come from one RTX 4090.

## The features

Each feature is a compile-time extension: add it to `EXTENSIONS ...` in the example's `CMakeLists.txt`, or define it
in `defines.hpp`.

| Extension or API | What it does | How to use it | Test |
|---|---|---|---|
| `THERMAL_DIFFUSIVITY_FIX` | Makes `alpha` the diffusivity of the thermal lattice. The stock rate diffuses with `alpha / 2`. | Add the extension and give alpha in lattice units. | `thermal_diffusivity_fix` |
| Heat source in `TYPE_Y` cells | Adds a heat flux in the first fluid layer, in place of a fixed temperature. `TYPE_X\|TYPE_Y` marks a patch with 4 times the rate. | Flag the cells `TYPE_Y`. Call `lbm.set_heat_rate(rate)`: the temperature added per step and cell. | `heat_source` |
| Eddy heat diffusivity | With `SUBGRID`, the Smagorinsky eddy viscosity also mixes heat and water, at a turbulent Prandtl number. | `lbm.set_inv_prandtl_t(1/Pr_t)`, for example 3 for Pr_t = 1/3. | `eddy_heat_diffusivity` |
| `LEVEL_FORCING` | Per horizontal level: a force (fx, fy), a damping of w and a relaxation of T toward T_ref at the same rate. The buoyancy uses the level's T_ref, so the lattice holds no hydrostatic ramp. Uses: wind nudging toward a profile, and a sponge layer under the lid. | `lbm.set_level_forcing(fx, fy, rate, T_ref)`, one value per level, at any time. | `level_forcing` |
| `MOISTURE` | Carries total water qt as Q = 1 + qt / 0.01, with the thermal diffusivity and a surface source. With `LEVEL_FORCING` the buoyancy comes from the virtual potential temperature after a saturation adjustment in each cell. `src/moist_air.hpp` is the same model on the host. | `lbm.set_moisture_rate(rate)` for the `TYPE_Y` source, `lbm.set_theta_0(theta)`, `lbm.set_level_moist(pressure, theta_v_ref)` per level. | `moisture_source`, `moist_buoyancy`, `moist_bubble_fv` |
| `SCALAR_FV` | Carries T and Q by finite volumes in conservative form, beside the unchanged LBM flow. The LBM's own link mass fluxes move them. Runs in any number of domains. | Add it with `TEMPERATURE` and `THERMAL_DIFFUSIVITY_FIX`. The API does not change. | the `*_fv` tests, `scalar_fv_domains` |

**For heat and water, use `SCALAR_FV`.** Without it, `TEMPERATURE` and `MOISTURE` use D3Q7 lattices, which have the
limits below. With `SCALAR_FV` the D3Q7 buffers are one-cell placeholders.

## How SCALAR_FV moves heat and water

The scheme carries `rho_FV` and `rho phi`, where phi is T - 1 or Q - 1. It moves them with the net mass on each of
the 18 D3Q19 links. The net mass comes from the post-collision populations (Dubois and Lallemand 2008, Zhao et al.
2020):

    psi_i(x) = f*_i(x) - f*_ibar(x + c_i)                    (the net mass from x to x + c_i in one step)
    rho_FV(t + 1) = rho_FV(t) - sum_i psi_i(x)
    (rho phi)(t + 1) = (rho phi)(t) - sum_i psi_i(x) phi_face,i + diffusion + sources
    phi = (rho phi) / rho_FV

- `phi_face,i` is the upwind MUSCL value along the link (kappa 1/3, the van Albada limiter, after ProLB: Feng et al.
  2021). The diffusion is centred on the 7 axis neighbours with face diffusivities, which include the eddy part.
- A midpoint time step uses the fluxes of the step in both stages.
- **Conservation:** `psi_ibar(x + c_i) = -psi_i(x)` exactly, so the sum of `rho phi` changes only by the sources.
- **Uniformity:** a uniform phi stays uniform, because `rho phi` changes by phi times the change of `rho_FV`.
- `rho_FV` is the scheme's own density (Guo et al. 2021). Away from walls it equals the LBM density. Next to walls it
  differs by the mass in transit in the bounce-back, and this does not affect conservation or uniformity.
- FluidX3D streams with Esoteric-Pull (Lehmann 2022), so all 18 fluxes come from the DDFs that a step stores.

**Domains.** A face value reads along each link the cell behind and the cell 2 ahead. So with `SCALAR_FV` the domain
halo is 2 cells wide. After each finite-volume stage the domains exchange T and Q (2 layers), their half-step values
(2 layers) and the diffusivity (1 layer). 2 x 1, 1 x 2 and 2 x 2 domains are bit-identical to one domain.

**Cost and memory** (a 10 m CBL with moisture, 256 x 256 x 207 = 13.57 M cells):

| Transport | ms per step | Bytes per cell |
|---|---|---|
| D3Q7 lattices | 4.4 | 157 |
| `SCALAR_FV` | 7.3 | 132 |
| `SCALAR_FV` with `FP16S` DDFs | not measured | 95 |

On one GPU, two domains take 48 % more time than one: each step has 4 exchanges through the host. A 5 m CBL of 172 M
cells (`FP16S`, two domains) runs at 1660 MLUPs: 45 minutes of model time in 58 minutes.

## The example

[`examples/convective_boundary_layer`](../../examples/convective_boundary_layer) simulates a CBL in physical units
through the Setup API's atmosphere classes ([SETUP_API.md](../../SETUP_API.md#7-atmospheric-convection)). It writes
the horizontal-mean statistics (`profiles.csv`) and the SETUP, FINGERPRINT, PROGRESS, CLOUD, WIND and RESULT lines.
Its targets are `convective_boundary_layer_fv` (FP32 DDFs) and `convective_boundary_layer_fv_fp16s` (`FP16S` DDFs,
with FP32 scalars). Run it with an output folder only to see its cases:

- `quick`: a 128 x 128 x 100 column for 100 s, to check the outputs.
- `sp2011`: the dry CBL of Sullivan and Patton 2011.
- `sounding`: a CBL on a tabulated start sounding, with a sponge under the lid, a heated patch and an optional wind
  held by nudging.

The options:

- `--diffusivity` sets the molecular diffusivity of T and Q in lattice units. The default is 1e-4 with `SCALAR_FV`.
- `--moist` with `--zi` takes the total water from the sounding's `qt_kgkg` column and adds the floor's moisture
  flux (Bowen ratio 0.5). It writes the clouds: the cloud fraction per level and a `CLOUD` line per interval. At the
  end it checks the mean cloud base against the mixed layer's condensation level (limit 10 %).
- `--spinup` and `--record` set the two times in seconds.
- `--estimate` prints the cells, the device memory per domain and the steps, and ends without a run.

## Limits

| Limit | What to do |
|---|---|
| The D3Q7 scalar lattices do not keep a uniform scalar uniform when the flow is not divergence-free on 7 points. In 2700 s of a 10 m CBL a uniform Q drifts by up to 8 %. This is a property of the linear-equilibrium scheme. | Use `SCALAR_FV`. |
| Next to `TYPE_S` walls in a buoyant flow, the D3Q7 thermal lattice creates heat when `tauT` is close to 1/2. At 0.503 the excess is 39 % of the added heat after 2000 steps. | Use `SCALAR_FV`, or `tauT >= 0.556` (alpha >= 0.014 with the fix). |
| With `FP16S` storage of the D3Q7 thermal DDFs, a wave does not diffuse in a flow (D = 0 at alpha 2e-3). The half floats overflow above T = 8. | With D3Q7, use FP32. With `SCALAR_FV`, `FP16S` compresses only the flow's DDFs and works. |
| `SCALAR_FV` adds diffusion across a flow at high cell Peclet numbers. The diagonal links carry mass both ways, and there the limiter changes the scheme to first order. At u 0.1: D/alpha 1.074 across and 1.305 along at alpha 2e-3, 1.007 and 1.026 at 0.014. | Known. The CBL results below include it. |
| With tau 600 s, the nudged wind misses the power-law target of the nudging by 27 to 38 % below zi, as MicroHH's nudged wind does. The example's `WIND` line prints this deviation as information. | Compare the wind with an LES reference, not with the target. |
| A thermal boundary layer with only 3 to 4 cells carries 15 to 40 % too much heat (first-order wall treatment). | Give a thermal layer 6 to 7 cells for 5 %. |

No runtime warning exists yet for `FP16S` with the D3Q7 `TEMPERATURE` lattice, or for `tauT` below 0.52 with
`VOLUME_FORCE`.

## Validation

**Thermal convection** (with `THERMAL_DIFFUSIVITY_FIX`):

- Rayleigh-Benard convection: the onset between Ra 1600 and 1900 (critical Ra 1708). The Richardson-extrapolated Nu is
  4.78 at Ra 1e5 and 8.72 at Ra 1e6. MicroHH in the same box gives 4.78 and 8.67, Kerr 1996 gives 4.46 and 8.42.
- A plume from a heated patch stays laminar at this lattice's Reynolds numbers. Its width grows linearly.

**The dry CBL** (Sullivan and Patton 2011, `sp2011`):

- At 5 m cells (512 x 512 x 412, D3Q7, two domains): w'^2/w*^2 within 10 to 20 % of MicroHH, the entrainment ratio
  -0.18, a k^-5/3 spectrum down to 4 cells. Two seeds differ by up to 17 %.
- At 10 m, `SCALAR_FV` against the D3Q7 lattice over 0.1 to 0.8 zi: w'^2 within 3.8 %, theta'^2 within 1.2 %, the
  heat flux within 1.8 %. This is inside the seed spread.
- The molecular diffusivity sets the inversion. At alpha 0.014, zi is 802 m against MicroHH's 1001 m. With
  `SCALAR_FV` at alpha 1e-4: zi 979 m, the entrainment ratio -0.230 (MicroHH -0.218).
- `FP16S` DDFs with `SCALAR_FV` against FP32: w'^2 2.7 %, theta'^2 1.4 %, the heat flux 0.8 % (the FP32 seed spread:
  3.0, 3.8 and 1.1 %).

**The moisture:** the moisture budget holds within 0.004 %, and theta_v after the saturation adjustment is within
0.013 % of the host's moist-air model. A moist thermal in saturated air rises within 0.14 % of its dry twin. Two checks
of Bryan and Fritsch 2002 pass:

- At rest, the water adds 3.5e-5 m/s to the float32 floor of the lattice (limit 1e-4 m/s).
- In a closed box, the totals of qt and thl change by 5.7e-10 and 9.8e-9 of themselves (limit 1e-6).

**The per-level forcing:** a sponge from 1.6 zi with the lid at 2 zi leaves the CBL statistics unchanged. The nudged
wind stays within 6.6 % of MicroHH's nudged wind (tau 600 s).

**The moist CBL** (`sounding ... --moist`, a heated patch, seed 2, the lid at 5000 m and at 4000 m for 5 m cells):

- At 10 m cells, the mean cloud base is within 1.8 % (normal sounding) and 2.6 % (humid sounding) of the mixed
  layer's condensation level. At 5 m cells: 1.5 % and 2.8 % (seeds 2 and 3).
- Against MicroHH on the same normal sounding (three seeds), frame by frame over 30 minutes:

| Mean over the frames | FluidX3D 10 m | FluidX3D 5 m, seeds 2 / 3 | MicroHH (3 seeds) |
|---|---|---|---|
| Cover from above | 0.31 | 0.22 / 0.40 | 0.34 to 0.45 |
| Clouds of 0.01 km2 or more, per km2 | 0.86 | 0.92 / 0.66 | 0.76 to 1.25 |
| All clouds, per km2 | 2.2 | 4.9 / 3.9 | 10.6 to 13.5 |
| Cloud base / top (m) | 1905 / 2492 | 1906 / 2416, 1896 / 2683 | 1832 to 1839 / 2606 to 2669 |

The larger clouds agree. FluidX3D makes 2 to 6 times fewer clouds in all: it makes fewer small clouds (under
0.01 km2).

## Fixes in FluidX3D

| Defect | Fix |
|---|---|
| The D3Q7 thermal lattice diffuses with `alpha / 2`: the code sets `def_w_T = 1 / (2 alpha + 1/2)`, but the D3Q7 diffusivity is `(tauT - 1/2) / 4` (measured 0.5007 alpha). | `THERMAL_DIFFUSIVITY_FIX` sets `def_w_T = 1 / (4 alpha + 1/2)` (measured 1.000 alpha). It is off by default, so the upstream examples run as upstream. |
| The example CMake function wrote the `defines.hpp` of every target into one folder, so two targets in one example folder used the same file. | One `defines_<target>/` folder per target. |

## How to run the tests

Build as [BUILD.md](../../BUILD.md) describes. The tests are CTest targets with the label `physics`. Each runs in
seconds to a few minutes and prints `PHYSICS <check>: error x % (limit y %) PASSED/FAILED`:

    cmake -S . -B build-tests -DFLUIDX3D_BUILD_TESTS=ON
    cmake --build build-tests --target physics_moist_bubble_fv
    ctest --test-dir build-tests -L physics -R _fv

The build scripts in `tools/` make the same build folder (on Linux, use the `.sh` scripts):

    tools/configure.bat Release Ninja --dir tests -- -DFLUIDX3D_BUILD_TESTS=ON
    tools/build.bat physics_moist_bubble_fv Release --dir tests

## References

- Bryan, G. H., Fritsch, J. M. (2002). A benchmark simulation for moist nonhydrostatic numerical models. Monthly
  Weather Review 130, 2917-2928. doi:10.1175/1520-0493(2002)130<2917:absfmn>2.0.co;2
- Dubois, F., Lallemand, P. (2008). On lattice Boltzmann scheme, finite volumes and boundary conditions. Progress in
  Computational Fluid Dynamics 8 (1-4), 11-24. doi:10.1504/pcfd.2008.018075. Open copy: <https://arxiv.org/abs/2306.15291>
- Feng, Y., Miranda-Fuentes, J., Guo, S., Jacob, J., Sagaut, P. (2021). ProLB: A lattice Boltzmann solver of
  large-eddy simulation for atmospheric boundary layer flows. Journal of Advances in Modeling Earth Systems 13.
  doi:10.1029/2020ms002107. Open copy: <https://hal.science/hal-03326123>
- Guo, S., Feng, Y., Sagaut, P. (2021). On the use of conservative formulation of energy equation in hybrid
  compressible lattice Boltzmann method. Computers & Fluids 219, 104866. doi:10.1016/j.compfluid.2021.104866. Open
  copy: <https://hal.science/hal-03597478>
- Kerr, R. M. (1996). Rayleigh number scaling in numerical convection. Journal of Fluid Mechanics 310, 139-179.
  doi:10.1017/s0022112096001760
- Lehmann, M. (2022). Esoteric Pull and Esoteric Push: Two simple in-place streaming schemes for the lattice
  Boltzmann method on GPUs. Computation 10, 92. doi:10.3390/computation10060092
- Sullivan, P. P., Patton, E. G. (2011). The effect of mesh resolution on convective boundary layer statistics and
  structures generated by large-eddy simulation. Journal of the Atmospheric Sciences 68, 2395-2415.
  doi:10.1175/jas-d-10-05010.1
- van Heerwaarden, C. C., van Stratum, B. J. H., Heus, T., Gibbs, J. A., Fedorovich, E., Mellado, J. P. (2017).
  MicroHH 1.0: a computational fluid dynamics code for direct numerical simulation and large-eddy simulation of
  atmospheric boundary layer flows. Geoscientific Model Development 10, 3145-3165. doi:10.5194/gmd-10-3145-2017
- Zhao, S., Farag, G., Boivin, P., Sagaut, P. (2020). Toward fully conservative hybrid lattice Boltzmann methods for
  compressible flows. Physics of Fluids 32, 126118. doi:10.1063/5.0033245. Open copy: <https://hal.science/hal-03087980>
