# Examples

Complete list of all 41 FluidX3D examples included in the CMake build system. Every example except `benchmark` uses
the Setup API ([README.md](README.md#setup-api)); its extensions are named in its `CMakeLists.txt`.

## All Examples

| Example | Extensions | Geometry file |
|---------|------------|---------------|
| **benchmark** | BENCHMARK | - |
| **taylor_green_3d** | INTERACTIVE_GRAPHICS | - |
| **taylor_green_2d** | D2Q9, INTERACTIVE_GRAPHICS | - |
| **poiseuille_flow** | VOLUME_FORCE | - |
| **stokes_drag** | FORCE_FIELD, EQUILIBRIUM_BOUNDARIES | - |
| **cylinder_duct** | VOLUME_FORCE, INTERACTIVE_GRAPHICS | - |
| **taylor_couette** | MOVING_BOUNDARIES, INTERACTIVE_GRAPHICS | - |
| **lid_driven_cavity** | MOVING_BOUNDARIES, INTERACTIVE_GRAPHICS | - |
| **karman_vortex_street** | D2Q9, FP16S, EQUILIBRIUM_BOUNDARIES, INTERACTIVE_GRAPHICS | - |
| **particle_test** | VOLUME_FORCE, FORCE_FIELD, MOVING_BOUNDARIES, PARTICLES, INTERACTIVE_GRAPHICS | - |
| **delta_wing** | FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS | - |
| **city** | FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS (4K frames) | city.stl (included) |
| **city_rt** | FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS | city.stl (included) |
| **hill** | FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS | hill.stl (included) |
| **nasa_crm** | FP16C, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS | ✓ |
| **concorde** | FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS | ✓ |
| **boeing_747** | FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS | ✓ |
| **xwing** | FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS | ✓ |
| **tie_fighter** | FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS | ✓ |
| **radial_fan** | FP16S, MOVING_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS | ✓ |
| **edf** | FP16S, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS | ✓ |
| **cow** | FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS | ✓ |
| **cow_sdf** | FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS | an SDF generated from Cow_t.stl (see the example) |
| **space_shuttle** | FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS (8 GPUs) | ✓ |
| **starship** | FP16S, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS | ✓ |
| **ahmed_body** | FP16C, FORCE_FIELD, EQUILIBRIUM_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS | ✓ |
| **cessna_172** | FP16S, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS | ✓ |
| **bell_222** | FP16C, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS | ✓ |
| **mercedes_f1** | FP16S, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SUBGRID, INTERACTIVE_GRAPHICS | ✓ |
| **hydraulic_jump** | FP16S, VOLUME_FORCE, EQUILIBRIUM_BOUNDARIES, MOVING_BOUNDARIES, SURFACE, SUBGRID, INTERACTIVE_GRAPHICS | - |
| **dam_break** | FP16S, VOLUME_FORCE, SURFACE, INTERACTIVE_GRAPHICS | - |
| **liquid_metal** | FP16S, VOLUME_FORCE, MOVING_BOUNDARIES, SURFACE, INTERACTIVE_GRAPHICS | - |
| **breaking_waves** | FP16S, VOLUME_FORCE, EQUILIBRIUM_BOUNDARIES, SURFACE, INTERACTIVE_GRAPHICS | - |
| **river** | FP16S, VOLUME_FORCE, SURFACE, INTERACTIVE_GRAPHICS | - |
| **raindrop** | FP16C, VOLUME_FORCE, EQUILIBRIUM_BOUNDARIES, SURFACE, INTERACTIVE_GRAPHICS | - |
| **bursting_bubble** | FP16C, VOLUME_FORCE, SURFACE, INTERACTIVE_GRAPHICS | - |
| **cube_gravity** | FP16S, VOLUME_FORCE, SURFACE, INTERACTIVE_GRAPHICS | - |
| **periodic_faucet** | FP16S, VOLUME_FORCE, SURFACE, INTERACTIVE_GRAPHICS | - |
| **colliding_droplets** | FP16S, VOLUME_FORCE, FORCE_FIELD, SURFACE, INTERACTIVE_GRAPHICS | - |
| **rayleigh_benard** | FP16S, VOLUME_FORCE, TEMPERATURE, INTERACTIVE_GRAPHICS | - |
| **thermal_convection** | FP16S, VOLUME_FORCE, TEMPERATURE, INTERACTIVE_GRAPHICS | - |

`INTERACTIVE_GRAPHICS` opens a window; built with `GRAPHICS` instead, the same example writes its video (`sim.video()`)
without one. The baseline tests build every example that way.

---

## STL File Requirements

18 examples need a geometry file: `city.stl` and `hill.stl` are included in `resources/`, `cow_sdf` needs an SDF
generated from `Cow_t.stl`, and 15 examples need STL files to download.

### Obtaining STL Files

**Option 1: Batch Download Script (Recommended)**
```bash
cd resources
python download_all_thingiverse_stl.py
```

This script will download all STL files from Thingiverse to the `resources/` directory.

**Option 2: Manual Download**

Download files from the sources below and place in `resources/` directory. An example whose file is missing prints
how to get it and stops.

### Thingiverse STL Files

| Example | Files | Source |
|---------|-------|--------|
| **cow** | Cow_t.stl | [Thing:182114](https://www.thingiverse.com/thing:182114/files) |
| **boeing_747** | techtris_airplane.stl | [Thing:2772812](https://www.thingiverse.com/thing:2772812/files) |
| **xwing** | X-Wing.stl | [Thing:353276](https://www.thingiverse.com/thing:353276/files) |
| **tie_fighter** | DWG_Tie_Fighter_Assembled_02.stl | [Thing:2919109](https://www.thingiverse.com/thing:2919109/files) |
| **space_shuttle** | Full_Shuttle.stl | [Thing:4975964](https://www.thingiverse.com/thing:4975964/files) |
| **starship** | StarShipV2.stl | [Thing:4912729](https://www.thingiverse.com/thing:4912729/files) |
| **concorde** | concord_cut_large.stl | [Thing:1176931](https://www.thingiverse.com/thing:1176931/files) |
| **edf** | edf_v39.stl, edf_v391.stl | [Thing:3014759](https://www.thingiverse.com/thing:3014759/files) |
| **radial_fan** | FAN_Solid_Bottom.stl | [Thing:6113](https://www.thingiverse.com/thing:6113/files) |
| **cessna_172** * | Airplane.stl | [Thing:814319](https://www.thingiverse.com/thing:814319/files) |
| **bell_222** * | BELL222__FIXED.stl | [Thing:1625155](https://www.thingiverse.com/thing:1625155/files) |

\* Requires manual splitting (see notes below)

### Other STL Files

| Example | Files | Source |
|---------|-------|--------|
| **city**, **city_rt** | city.stl | included in `resources/` |
| **hill** | hill.stl | included in `resources/` (a Unity terrain export, 4000 m x 4000 m) |
| **mercedes_f1** | mercedesf1-body.stl, mercedesf1-front-wheels.stl, mercedesf1-back-wheels.stl | [downloadfree3d.com](https://downloadfree3d.com/3d-models/vehicles/sports-car/mercedes-f1-w14/) |
| **ahmed_body** | ahmed_25deg_m.stl | [GitHub](https://github.com/nathanrooy/ahmed-bluff-body-cfd/blob/master/geometry/ahmed_25deg_m.stl) |
| **nasa_crm** | crm-hl_reference_ldg.stl | [NASA LaRC](https://commonresearchmodel.larc.nasa.gov/high-lift-crm/high-lift-crm-geometry/assembled-geometry/) |

### STL Files Requiring Manual Processing

**cessna_172** and **bell_222** download combined STL files that must be split into separate components:

**Cessna 172:**
1. Download `Airplane.stl` from Thingiverse
2. Open in 3D modeling software (Microsoft 3D Builder, Blender)
3. Separate body and propeller into 2 meshes
4. Save as `Cessna-172-Skyhawk-body.stl` and `Cessna-172-Skyhawk-rotor.stl`
5. Place both files in `resources/`

**Bell 222:**
1. Download `BELL222__FIXED.stl` from Thingiverse
2. Open in 3D modeling software
3. Separate fuselage, main rotor, and tail rotor into 3 meshes
4. Save as `Bell-222-body.stl`, `Bell-222-main.stl`, and `Bell-222-back.stl`
5. Place all 3 files in `resources/`

---

## Building Examples

See [BUILD.md](BUILD.md) for detailed build instructions.

### Quick Reference

```bash
# Build specific example
cmake --build build --target taylor_green_3d

# Run example
./bin/taylor_green_3d
```

---

## Extension Definitions

- **D2Q9 / D3Q19 / D3Q27**: Velocity set (2D or 3D lattice)
- **SRT / TRT**: Collision operator (Single/Two Relaxation Time)
- **FP16S / FP16C**: 16-bit floating point compression (S=storage, C=compute)
- **VOLUME_FORCE**: Body forces (gravity, etc.)
- **FORCE_FIELD**: Non-uniform force fields, forces on solids
- **EQUILIBRIUM_BOUNDARIES**: Equilibrium boundary conditions (inflow, outflow)
- **MOVING_BOUNDARIES**: Moving/rotating boundaries
- **SURFACE**: Free surface tracking
- **TEMPERATURE**: Temperature field
- **SUBGRID**: Sub-grid scale turbulence model
- **PARTICLES**: Particle tracking
- **INTERACTIVE_GRAPHICS / GRAPHICS**: A window, or frames written to disk
- **BENCHMARK**: Performance benchmark mode

---

## Example Categories

### Performance Testing
- **benchmark** - FP32 performance benchmark

### Validation Cases
- **taylor_green_3d** / **taylor_green_2d** - Taylor-Green vortex
- **poiseuille_flow** - Laminar pipe flow, compared with the analytic profile
- **stokes_drag** - Stokes drag on sphere, compared with Stokes' law
- **lid_driven_cavity** - Classic CFD validation

### Aerodynamics
- **delta_wing** - Delta wing aircraft
- **nasa_crm** - NASA Common Research Model
- **concorde** / **boeing_747** / **xwing** / **tie_fighter** - Various aircraft (the TIE fighter tumbles)
- **cessna_172** - Propeller aircraft with rotating prop
- **bell_222** - Helicopter with rotating rotors
- **space_shuttle** / **starship** - Spacecraft (the shuttle on 8 GPUs)
- **mercedes_f1** - Formula 1 race car
- **city** - Urban wind simulation (video, 4K frames)
- **city_rt** - Urban wind simulation (real-time interactive)
- **hill** - Wind over terrain with an atmospheric boundary layer (clearances around an STL in metres)
- **cow** - Cow aerodynamics (humor example)
- **cow_sdf** - The cow voxelized from a signed distance field

### Rotating Machinery
- **radial_fan** - Centrifugal fan
- **edf** - Electric ducted fan

### Free Surface Flows
- **hydraulic_jump** - Hydraulic jump phenomenon
- **dam_break** - Dam break simulation
- **liquid_metal** - Liquid metal on a vibrating membrane
- **breaking_waves** - Waves breaking on a beach
- **river** - River flow
- **raindrop** - Raindrop impact
- **bursting_bubble** - Bubble bursting
- **cube_gravity** - A cube of water under turning gravity
- **periodic_faucet** - Dripping faucet
- **colliding_droplets** - Droplet collision

### Thermal
- **rayleigh_benard** - Rayleigh-Bénard convection
- **thermal_convection** - Natural convection

### Other
- **cylinder_duct** - Flow around cylinder in duct
- **taylor_couette** - Taylor-Couette flow
- **karman_vortex_street** - Von Kármán vortex street
- **particle_test** - Particle tracking test
- **ahmed_body** - Ahmed body (automotive CFD reference), with its drag coefficient

---

## Additional Resources

- **[CMAKE.md](CMAKE.md)** - Architecture and how to add new examples
- **[BUILD.md](BUILD.md)** - Build instructions
- **[SETUP_API.md](SETUP_API.md)** - How to write a simulation with the Setup API
- **[DOCUMENTATION.md](DOCUMENTATION.md)** - Original FluidX3D documentation
