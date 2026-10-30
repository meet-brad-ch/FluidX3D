# Changelog

All notable changes to FluidX3D-API. The format follows [Keep a Changelog](https://keepachangelog.com).

A version has the form `<upstream version>-api.<n>`, for example `3.8-api.1`: the FluidX3D release that the version
is based on, and the number of the FluidX3D-API release on that base. When FluidX3D-API moves to a new FluidX3D
release, `<n>` starts at 1 again. Each section names the exact upstream commit.

## [Unreleased]

## [3.8-api.1]

First release of FluidX3D-API. Based on ProjectPhysX/FluidX3D v3.8 + 10 commits (b85d64e).

### Added
- A CMake build: one build target per example, automatic dependency fetching, IDE support (Visual Studio, VS Code,
  CLion, Qt Creator). Each of the 42 examples has its own feature set.
- The Setup API: a `Simulation` in physical units (Au units library) for domain size, STL loading, voxelization,
  boundaries, graphics and a run loop in simulated time.
- SDF files for mesh voxelization, and a hash-based SDF cache (`sdf_cache`).
- Atmospheric convection: `THERMAL_DIFFUSIVITY_FIX`, a heated layer or patch as a flux boundary
  (`LBM::set_heat_rate()`), eddy heat diffusivity with `SUBGRID` (`LBM::set_inv_prandtl_t()`), `LEVEL_FORCING`,
  `MOISTURE`, conservative finite-volume scalars (`SCALAR_FV`), the atmosphere classes of the Setup API, and the
  example `convective_boundary_layer`. See [FORK_CHANGES.md](FORK_CHANGES.md) and
  [research/convection/README.md](research/convection/README.md).
- Python extension modules of FluidX3D applications (nanobind), and applications outside the repository.
- clang-tidy configuration and documentation checks.
- `scripts/release.sh` and the release workflow.

### Fixed
- The thermal lattice diffused with `alpha / 2` (behind `THERMAL_DIFFUSIVITY_FIX`).
- Vertical slice artifacts in mesh voxelization (ray jitter).
- The example CMake function wrote the `defines.hpp` of every target into one folder, so two targets in one example
  folder used the same file.

### Changed
- The project name is FluidX3D-API. The README marks it as a modified version of FluidX3D. The FluidX3D license
  applies without change.
- The documentation names the release pages of the Intel CPU runtime, oneTBB and the source code, and has no direct
  archive URLs.
