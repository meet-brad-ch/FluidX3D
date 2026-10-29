// The atmosphere API (ConvectiveBoundaryLayer over AtmosphereSimulation and its builders) gives the verified CBL
// example's set-up: for the golden cases quick, quick surface and sp2011 (tests/golden/cbl), the FNV-1a hash of every
// cell's flags, rho, u, T and Q, the level arrays and the scalars given to the core equals the example's FINGERPRINT, and the patch's cells
// and the heat rate equal its capture. With FLUIDX3D_EXTRA_CBL, FLUIDX3D_EXTRA_CBL_WIND and
// FLUIDX3D_EXTRA_SOUNDING set, the two moist cases too (the moist column's pressures in the hash, the start wind).
// No time step runs: the hash is of the host buffers before the first run.
// Passes when every case's hash equals its capture's.

#include "atmosphere_cases.hpp"
#include "physics_check.hpp"

#include <cstdio>

namespace {

/// @param c a case
/// @brief Compares one case's set-up through the API with its capture and prints the result.
/// @param fingerprint the API's fingerprint
/// @param expected the capture's fingerprint
/// @param summary the API's heat sources
/// @return whether the fingerprint, the patch's cells and the heat rate equal the capture's (read in that order, the
/// next only when the one before is equal)
bool same_as_capture(const AtmosphereCase& c, std::uint64_t fingerprint, std::uint64_t expected, const SourceSummary& summary) {
    if(fingerprint != expected) {
        return false;
    }
    const double patch_cells = (double)summary.patch_cells;
    const double expected_patch_cells = c.capture.setting("patch_cells");
    if(patch_cells != expected_patch_cells) {
        return false;
    }
    const double heat_rate = (double)summary.heat_rate;
    const double expected_heat_rate = c.capture.setting("heat_rate");
    const bool same_heat_rate = heat_rate == expected_heat_rate;
    return same_heat_rate;
}

} // namespace

void main_setup() { // extensions: VOLUME_FORCE TEMPERATURE SUBGRID THERMAL_DIFFUSIVITY_FIX LEVEL_FORCING MOISTURE SCALAR_FV
    const std::vector<AtmosphereCase> cases = AtmosphereCases::all();
    double failed = 0.0;
    for(const AtmosphereCase& c : cases) {
        const CblDescription& d = c.description;
        ConvectiveBoundaryLayer run(d); // validates d
        const SourceSummary summary = run.atmosphere().sources();
        const std::uint64_t h = run.atmosphere().fingerprint();
        const std::uint64_t expected = AtmosphereCases::captured_fingerprint(c.capture);
        const bool same = same_as_capture(c, h, expected, summary);
        const unsigned long long printed_hash = (unsigned long long)h;
        const unsigned long long printed_expected = (unsigned long long)expected;
        const unsigned long long patch_cells = (unsigned long long)summary.patch_cells;
        const double heat_rate = (double)summary.heat_rate;
        const char* verdict = same ? "the same" : "DIFFERENT";
        std::printf("\n%s: fingerprint %016llx (capture %016llx), patch %llu cells, heat rate %a: %s\n", c.name.c_str(), printed_hash, printed_expected, patch_cells,
                    heat_rate, verdict);
        if(!same) {
            failed += 1.0;
        }
    }
    const std::string count = std::to_string(cases.size());
    const std::string check = "atmosphere column: the API's set-up is the verified example's (" + count + " cases)";
    PhysicsCheck::report(check.c_str(), failed, 0.5);
}
