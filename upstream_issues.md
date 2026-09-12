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
