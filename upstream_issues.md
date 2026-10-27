# Issues found in the upstream FluidX3D code

Found while porting the examples; to report to the upstream project. This fork does not change the upstream code
(`src/`), so these are only fixed in the ported examples, where noted.

## Examples (upstream `src/setup.cpp`)

### 3D Taylor-Green vortices: the start is compressible
`main_setup()` of the 3D Taylor-Green vortices sets

    u.x =  A*cos(x)*sin(y)*sin(z)
    u.y = -A*sin(x)*cos(y)*sin(z)
    u.z =  A*sin(x)*sin(y)*cos(z)

with the density `rho = 1-3/4*A²*(cos(2x)+cos(2y))`. The divergence of this velocity is `-A*sin(x)*sin(y)*sin(z)`,
not zero: the flow starts compressible and sends out pressure waves. The classic Taylor-Green vortex has `u.z = 0`,
and its pressure `p = rho*A²/16*(cos(2x)+cos(2y))*(cos(2z)+2)` (for these phases:
`-rho*A²/16*(cos(2x)+cos(2y))*(2-cos(2z))`); the density above is that of the 2D vortices.
Fixed in the port (`examples/taylor_green_3d`).

### Cylinder in a duct: the force of a square duct (to confirm)
The cylinder-in-duct setup drives the flow with `units.f_from_u_rectangular_duct(w, D, 1.0f, nu, u)`: the force for
the center velocity `u` in a duct `w` x `D`, which is square (`w = D`). The simulated duct is `w` x `3*D` high, so the
laminar center velocity is higher than `u`. Perhaps intended as a rough value; kept in the port.

## Core (upstream `src/lbm.cpp`, `src/kernel.cpp`)

### TEMPERATURE: the thermal relaxation rate gives half the diffusivity
`LBM_Domain::device_defines()` sets `def_w_T = 1/(2*alpha+1/2)`. The D3Q7 temperature lattice diffuses with
`D = c^2 (tauT - 1/2)`, `c^2 = 1/4`, so this rate gives `D = alpha/2`: every thermal setup diffuses heat at half the
`alpha` it passes (air's Prandtl number 0.71 becomes about 1.42, the Rayleigh number doubles). Measured in this fork
(`tests/physics/thermal_diffusivity.cpp`, 2026-10-25: a temperature wave's decay, `D = 0.9987 alpha/2` at `tauT` 0.75,
`1.0003 alpha/2` at 0.504). The eddy part of the fork's `set_inv_prandtl_t` was written for `c^2 = 1/4` and is right.
Fixed in this fork behind `THERMAL_DIFFUSIVITY_FIX` (`def_w_T = 1/(4*alpha+1/2)`, `D = alpha` within 0.2 %); the stock
rate stays the default so the upstream examples run as upstream.

### TEMPERATURE with FP16S: the thermal DDFs do not diffuse in a flow
With `FP16S` the thermal DDFs `gi` are stored as half floats too. A temperature wave in a flow of `u = 0.1` does not decay
at all (measured `D = 0` at `alpha` 2e-3 and 0.125 across the flow, and at rest at 2e-3; `tests/physics/
thermal_diffusivity_fp16s.cpp`, 2026-10-25): the diffusive change per step is below the half-float resolution of the
populations that carry the `0.5 T u` term. The upstream `rayleigh_benard` and `thermal_convection` setups run in FP16S.
Not fixed here (the fork's thermal setups use FP32).

### TEMPERATURE: heat is created next to TYPE_S walls in a buoyant flow at tauT near 1/2
With `VOLUME_FORCE` buoyancy (`f = fz (1 - beta (T - 1))`), a heated TYPE_Y patch on a TYPE_S floor and `alpha` such that
`tauT` = 0.503, the thermal lattice does not conserve heat: the total `sum(T - 1)` over the fluid rises 39 % above the
heat added within 2000 steps, temperatures go negative (-0.046), and the floor layer then cools steadily (T 1 -> 0.09 in
20 000 steps) while the ceiling layer warms (`tests/physics/heat_conservation.cpp`, 2026-10-26). The error falls with
`tauT` (7.7 % at 0.508, 2.8 % at 0.514, 0.2 % at 0.528) and vanishes without walls (periodic in z: exact budget, but
the lowest T still undershoots to 0.17 at 0.503). The mechanism is not identified. The fork keeps `tauT >= 0.528` in
every thermal setup; the upstream `rayleigh_benard` and `thermal_convection` examples use TYPE_S plates with air's
diffusivity through the Setup API's unit conversion (their `tauT` and budget not measured here).
