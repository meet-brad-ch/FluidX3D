#pragma once

#include "setup/core/scalar_transport_rules.hpp"
// the build's extensions: without this include the #ifdefs below see none of them when this header comes before
// lbm.hpp, and current_build() describes another build
#include "defines.hpp"
#if !defined(D2Q9) && !defined(D3Q15) && !defined(D3Q19) && !defined(D3Q27)
#error "transport_build.hpp: the build's defines.hpp defines no velocity set"
#endif

/// @file transport_build.hpp
/// @brief The build's transport extensions as a TransportBuild. Header-side: it reads the build's defines, so it is
/// included by the code compiled with an example (Simulation), never by the define-free library.

/// @brief Reads the transport extensions of the build that compiles this header, from its defines.hpp.
/// @return the extensions of this build that decide how heat and water are carried
inline TransportBuild current_build() {
    TransportBuild build{};
#ifdef THERMAL_DIFFUSIVITY_FIX
    build.diffusivity_fix = true;
#endif // THERMAL_DIFFUSIVITY_FIX
#ifdef SCALAR_FV
    build.finite_volumes = true;
#endif // SCALAR_FV
#ifdef MOISTURE
    build.moisture = true;
#endif // MOISTURE
#ifdef LEVEL_FORCING
    build.level_forcing = true;
#endif // LEVEL_FORCING
#ifdef SUBGRID
    build.subgrid = true;
#endif // SUBGRID
#if defined(FP16S) || defined(FP16C)
    build.fp16_ddfs = true;
#endif // FP16S || FP16C
    return build;
}
