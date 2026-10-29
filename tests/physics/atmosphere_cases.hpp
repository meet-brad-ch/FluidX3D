#pragma once

// The golden CBL cases as descriptions (tests/physics/atmosphere_column.cpp, atmosphere_profiles.cpp).

#include "setup/setup.hpp"
#include "setup/atmosphere/convective_boundary_layer.hpp"
#include "golden_capture.hpp"

#include <array>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

/// @brief One golden case: its capture and its description.
struct AtmosphereCase { // NOLINT(bugprone-exception-escape): MSVC's std::map move allocates its head node; a failed allocation ends the test
    std::string name{};           ///< for the messages
    GoldenCapture capture;        ///< the verified example's capture (no default: it reads its files)
    CblDescription description{}; ///< the same case as a description
};

/// @brief The golden cases as descriptions.
class AtmosphereCases {
public:
    /// @brief Gives the example's quick preset as a description.
    /// @param surface the patch in the surface layer (quick surface) or over 4 heated layers (quick)
    /// @return the example's quick preset (the time step its literal 0.04 s)
    static CblDescription quick(bool surface) {
        using namespace si::symbols;
        const si::Duration time_step{ 0.04 * s };
        const si::Length zi{ 180.0 * m };
        const si::Length cell{ 2.5 * m };
        CblNumerics numerics{};
        numerics.time_step = TimeStepRule::fixed(time_step);
        const PatchMode mode = surface ? PatchMode::SURFACE_LAYER : PatchMode::HEATED_LAYERS;
        const HeatedPatch patch = HeatedPatch::disc(40.0 * m, 0.0 * m, mode);
        CblDescription d = CblDescription();
        d = d.with_numerics(numerics);
        const si::Temperature mixed_theta = si::kelvin_point(300.0);
        const IdealizedProfile profile = IdealizedProfile{ .inversion_base = zi, .jump = 1.0 * K, .jump_depth = 0.0 * m, .lapse = 0.005 * K / m };
        const Sounding sounding = Sounding::idealized(mixed_theta, profile);
        d = d.with_sounding(sounding, zi);
        d = d.with_cell(cell);
        const LidRule lid = LidRule::at(245.0 * m);
        const ColumnExtent extent = ColumnExtent{ .east = 320.0 * m, .north = 320.0 * m, .lid = lid };
        d = d.with_extent(extent);
        const SurfaceForcing forcing = SurfaceForcing{ .flux = 0.262 * K * m / s, .patch = patch, .bowen = 0.5 };
        d = d.with_surface(forcing);
        const StartNoise noise = StartNoise::per_cell(0.1 * K, 30.0 * m, 2u);
        d = d.with_noise(noise);
        const RunSchedule schedule = RunSchedule{ .spinup = 60.0 * s, .record = 40.0 * s, .interval = 10.0 * s };
        d = d.with_schedule(schedule);
        return d;
    }

    /// @brief Gives the example's sounding preset for an extra moist capture.
    /// @param g an extra moist capture (its zi, extent, flux, patch offset, domains and wind)
    /// @param csv its sounding
    /// @return the example's sounding preset: 10 m cells, the lid at 2 zi, the sponge from 1.6 zi (450 s), a 400 m
    /// surface-layer patch, moisture, 300 s + 200 s by 20 s; with wind 5 m/s at 100 m from the north (0.25, 600 s)
    static CblDescription sounding(const GoldenCapture& g, const std::string& csv) {
        using namespace si::symbols;
        const si::Length zi = g.setting("zi") * m;
        const si::Length cell = 10.0 * m;
        CblDescription d = CblDescription();
        const Sounding tabulated = Sounding::from_csv(csv);
        const Sounding sounding = tabulated.with_reference_from_mixed_layer(zi);
        d = d.with_sounding(sounding, zi);
        const si::Length east = cell * g.setting("Nx");
        const si::Length north = cell * g.setting("Ny");
        const LidRule lid = LidRule::in_inversion_heights(2.0);
        const ColumnExtent extent = ColumnExtent{ .east = east, .north = north, .lid = lid };
        d = d.with_extent(extent);
        const si::KinematicHeatFlux flux = g.setting("flux") * K * m / s;
        const si::Length patch_north = g.setting("patch_north") * m;
        const HeatedPatch patch = HeatedPatch::disc(400.0 * m, patch_north, PatchMode::SURFACE_LAYER);
        const SurfaceForcing forcing = SurfaceForcing{ .flux = flux, .patch = patch, .bowen = 0.5 };
        d = d.with_surface(forcing);
        const si::Length sponge_from = zi * 1.6;
        const SpongeProfile sponge = SpongeProfile::quadratic(sponge_from, 450.0 * s);
        d = d.with_sponge(sponge);
        const si::Duration spinup = g.setting("spinup") * s;
        const si::Duration record = g.setting("record") * s;
        const RunSchedule schedule = RunSchedule{ .spinup = spinup, .record = record, .interval = 20.0 * s };
        d = d.with_schedule(schedule);
        const std::uint32_t domains_x = (std::uint32_t)g.setting("Dx");
        const uint3 domains = uint3(domains_x, 1u, 1u);
        d = d.with_domains(domains);
        d = d.with_moisture(true);
        const double wind_speed = g.setting("wind_speed");
        if(wind_speed > 0.0) {
            const PowerLawWind wind = PowerLawWind(5.0 * m / s, 100.0 * m, 0.25, 0.0 * deg);
            const WindForcing forcing_wind = WindForcing{ .wind = wind, .nudge_tau = 600.0 * s };
            d = d.with_wind(forcing_wind);
        }
        return d;
    }

    /// @brief Gives the golden cases that can run here.
    /// @return the public cases (quick, quick surface, sp2011) and the extra moist ones (outside the repository) whose variables are set
    /// (FLUIDX3D_EXTRA_CBL, FLUIDX3D_EXTRA_CBL_WIND with FLUIDX3D_EXTRA_SOUNDING)
    static std::vector<AtmosphereCase> all() {
        std::vector<AtmosphereCase> cases{};
        const AtmosphereCase quick_case = AtmosphereCase{ .name = "quick", .capture = GoldenCapture("quick"), .description = quick(false) };
        cases.push_back(quick_case);
        const AtmosphereCase quick_surface_case = AtmosphereCase{ .name = "quick surface", .capture = GoldenCapture("quick_surface"), .description = quick(true) };
        cases.push_back(quick_surface_case);
        const AtmosphereCase sp2011_case = AtmosphereCase{ .name = "sp2011", .capture = GoldenCapture("sp2011"), .description = CblDescription() };
        cases.push_back(sp2011_case);
        const std::optional<std::string> csv = GoldenCapture::environment("FLUIDX3D_EXTRA_SOUNDING");
        const std::array<const char*, 2u> variables{ "FLUIDX3D_EXTRA_CBL", "FLUIDX3D_EXTRA_CBL_WIND" };
        for(const char* variable : variables) {
            const std::optional<std::string> folder = GoldenCapture::environment(variable);
            if(folder && csv) {
                const GoldenCapture g = GoldenCapture::at(*folder);
                const CblDescription description = sounding(g, *csv);
                const AtmosphereCase extra = AtmosphereCase{ .name = variable, .capture = g, .description = description };
                cases.push_back(extra);
            }
        }
        return cases;
    }

    /// @brief Reads a capture's set-up fingerprint (fingerprint.txt, hexadecimal).
    /// @param g a capture
    /// @return its fingerprint
    /// @throws std::runtime_error when the file has no fingerprint; std::invalid_argument or std::out_of_range from
    /// std::stoull for a text that is not a 64-bit hexadecimal number
    static std::uint64_t captured_fingerprint(const GoldenCapture& g) {
        const std::string file = g.path("fingerprint.txt");
        std::ifstream in(file);
        std::string hex{};
        in >> hex;
        if(hex.empty()) {
            throw std::runtime_error("no fingerprint in " + file);
        }
        const std::uint64_t fingerprint = std::stoull(hex, nullptr, 16);
        return fingerprint;
    }
};
