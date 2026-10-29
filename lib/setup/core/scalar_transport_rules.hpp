#pragma once

#include <optional>
#include <string>
#include <vector>

/// @brief The extensions of a build that decide how heat and water are carried, as a record.
///
/// current_build() (setup/core/transport_build.hpp) fills it from the build's defines. The unit tests set every
/// combination by hand.
struct TransportBuild {
    bool diffusivity_fix = false; ///< THERMAL_DIFFUSIVITY_FIX: the D3Q7 rate makes alpha the diffusivity
    bool finite_volumes = false;  ///< SCALAR_FV: heat and water move by finite volumes, not by the D3Q7 lattices
    bool moisture = false;        ///< MOISTURE: the total water is a second scalar
    bool level_forcing = false;   ///< LEVEL_FORCING: a forcing per level (the sponge, the nudging)
    bool subgrid = false;         ///< SUBGRID: the Smagorinsky model, and with it the eddy heat diffusivity
    bool fp16_ddfs = false;       ///< FP16S or FP16C: the DDFs are stored as half floats
};

/// @brief The surroundings of a scalar that decide whether a diffusivity is safe, as a record.
struct ScalarContext {
    bool solid_walls = false; ///< true when solid cells (TYPE_S) touch the scalar's cells
    bool buoyant = false;     ///< true when the scalar drives a buoyancy force
    bool moist = false;       ///< true when the total water is carried too
};

/// @brief The rules that connect a diffusivity to the core's thermal rate, and the combinations of diffusivity,
/// transport and build that fail or need a warning.
///
/// The core's D3Q7 lattice diffuses with D = (tauT - 1/2)/4. With THERMAL_DIFFUSIVITY_FIX it sets
/// tauT = 4 alpha + 1/2, so alpha is the diffusivity; without the fix it sets tauT = 2 alpha + 1/2 and the lattice
/// diffuses with alpha/2 (src/lbm.cpp, measured in tests/physics/thermal_diffusivity.cpp). core_alpha() gives the
/// alpha that makes the lattice diffuse with a wanted D in both builds. The finite volumes (SCALAR_FV, which needs the
/// fix) diffuse with alpha.
class ScalarTransportRules {
public:
    /// @brief Gives the alpha to hand to the core so that its scalar diffuses with a wanted diffusivity: the
    /// diffusivity itself with THERMAL_DIFFUSIVITY_FIX, twice the diffusivity without it.
    /// @param diffusivity the wanted diffusivity in cells squared per time step, finite, 0 or more
    /// @param build the build's transport extensions
    /// @return alpha in lattice units
    /// @throws SetupError when the diffusivity is not a finite number of 0 or more, or the build has SCALAR_FV without
    /// THERMAL_DIFFUSIVITY_FIX
    static float core_alpha(float diffusivity, const TransportBuild& build);

    /// @brief Gives the relaxation time of the D3Q7 lattice that diffuses with a diffusivity (in either build).
    /// @param diffusivity a diffusivity in cells squared per time step
    /// @return tauT = 4 D + 1/2 (a lattice number)
    static float relaxation_time(float diffusivity);

    /// @brief Gives the reason why a scalar set-up cannot run, for the atmosphere layer. Two set-ups are refused on
    /// the D3Q7 lattice: tauT below wall_safe_relaxation_time next to solid walls in a buoyant flow (the lattice
    /// creates heat there), and FP16S or FP16C DDFs (a temperature wave does not diffuse in a flow). With SCALAR_FV
    /// nothing is refused.
    /// @param diffusivity the scalar's diffusivity in cells squared per time step
    /// @param build the build's transport extensions
    /// @param context the scalar's surroundings
    /// @return the reason in words (with the remedy), or none when the set-up can run
    /// @throws SetupError when the reason does not fit its text buffer (a programming error)
    static std::optional<std::string> refusal(float diffusivity, const TransportBuild& build, const ScalarContext& context);

    /// @brief Gives the limits that a set-up meets but that do not stop it, for every thermal simulation on the D3Q7
    /// lattice: tauT below warning_relaxation_time in a buoyant flow, FP16S or FP16C DDFs, and the drift of a
    /// uniform water content in a moist run. With SCALAR_FV there is none.
    /// @param diffusivity the scalar's diffusivity in cells squared per time step
    /// @param build the build's transport extensions
    /// @param context the scalar's surroundings
    /// @return one line of text per limit met, empty when none is met
    /// @throws SetupError when a warning does not fit its text buffer (a programming error)
    static std::vector<std::string> warnings(float diffusivity, const TransportBuild& build, const ScalarContext& context);

    /// The smallest tauT at which the D3Q7 lattice creates no heat next to solid walls in a buoyant flow (measured in
    /// tests/physics/heat_conservation.cpp).
    static constexpr float wall_safe_relaxation_time = 0.556f;
    /// The tauT below which a buoyant set-up on the D3Q7 lattice gets a warning.
    static constexpr float warning_relaxation_time = 0.52f;
    /// The factor between the D3Q7 lattice's diffusivity and its relaxation time: D = (tauT - 1/2)/4.
    static constexpr float lattice_rate_factor = 4.0f;
};
