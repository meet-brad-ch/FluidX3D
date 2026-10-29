# Golden data of the convective boundary layer example

The stage capture (`--capture`) of the convective boundary layer example on the Setup API, for three cases:

| Case | Command line | Box |
|---|---|---|
| `quick` | `quick` | 128 x 128 x 100 cells of 2.5 m (a 320 m square box), 100 s of simulated time |
| `quick_surface` | `quick surface` | as `quick`; the patch heats the surface layer at the core's 4x rate |
| `sp2011` | `sp2011 10 128` | 128 x 128 x 207 cells of 10 m |

## The files of each case

- The 12 stage files: `settings.txt`, `levels.csv`, `patch.csv`, `noise.csv`, `noise_hash.txt`, `fingerprint.txt`,
  `fields.txt`, `fields_last.txt`, `interval_1.csv` to `interval_3.csv` and `interval_last.csv`. Floats are written as
  hexadecimal floats, so a comparison can be bit for bit.
- `profiles.csv`: the example's layer means.
- `run_lines.txt`: the set-up, fingerprint, wind and result lines of the run (the output folder replaced by
  `<output folder>`).
- `sha256.txt`: the SHA-256 of `profiles.csv`, and for `quick` and `quick_surface` of `frames.bin` from a recording run
  of the same case. The recording run's `profiles.csv` is byte-identical to the capture run's.

## The tests that read it

- `tests/unit` (through `golden_capture.hpp`): the set-up values of the Setup API against the capture, bit for bit.
- `physics_atmosphere_column`: the set-up fingerprint of every case.
- `physics_atmosphere_profiles`: `profiles.csv` byte for byte and the `RESULT CLOUD` line of every case.

## How it was made

- A GPU run of the example; the example and the Setup API are compiled without fast math (`FLUIDX3D_PRECISE_FLOAT`), so
  every set-up value is the same on every compiler.
- A second run gives the same data rows.
- The example's set-up values and diagnostics were checked one by one against independent derivations before the data
  was first frozen.

## When it changes

The data changes only with a change that is meant to change the set-up or the results. The commit that writes the new
data states the cause of each difference.
