#include "setup/core/diffusivity.hpp"
#include "setup/core/setup_error.hpp"

#include <cstdio>

Diffusivity Diffusivity::physical(si::KinematicViscosity d) {
    const bool valid{ d >= au::ZERO }; // false for NaN too
    if(!valid) {
        throw SetupError("Diffusivity::physical(): the diffusivity must be a number of at least 0 m2/s");
    }
    const Diffusivity diffusivity(d);
    return diffusivity;
}

Diffusivity Diffusivity::lattice(si::LatticeDiffusivity d) {
    const double value{ d.value() };
    const bool valid{ value >= 0.0 }; // false for NaN too
    if(!valid) {
        throw SetupError("Diffusivity::lattice(): the diffusivity must be a number of at least 0");
    }
    const Diffusivity diffusivity(d);
    return diffusivity;
}

float Diffusivity::lattice_value(const si::LatticeConverter& converter) const {
    const si::LatticeDiffusivity* given_lattice{ std::get_if<si::LatticeDiffusivity>(&value_) };
    float result{};
    if(given_lattice != nullptr) {
        const double value{ given_lattice->value() };
        result = static_cast<float>(value);
    } else {
        const si::KinematicViscosity given_physical{ std::get<si::KinematicViscosity>(value_) };
        result = converter.viscosity(given_physical);
    }
    return result;
}

si::KinematicViscosity Diffusivity::physical_value(const si::LatticeConverter& converter) const {
    const si::LatticeDiffusivity* given_lattice{ std::get_if<si::LatticeDiffusivity>(&value_) };
    si::KinematicViscosity result{ au::ZERO };
    if(given_lattice != nullptr) {
        result = converter.physical(*given_lattice);
    } else {
        result = std::get<si::KinematicViscosity>(value_);
    }
    return result;
}

std::string Diffusivity::describe(const si::LatticeConverter& converter) const {
    const float lattice_float{ lattice_value(converter) };
    const double lattice_double{ static_cast<double>(lattice_float) };
    const si::KinematicViscosity physical_d{ physical_value(converter) };
    const double physical_m2_s{ physical_d.in(au::squared(au::meters) / au::second) };
    const bool given_lattice{ is_lattice() };
    const char* const given_as{ given_lattice ? "a lattice value" : "a physical value" };
    constexpr std::size_t text_capacity{ 120 };
    char text[text_capacity]{};
    const int written{ std::snprintf(text, sizeof(text), "%.4g (%.3g m^2/s, given as %s)", lattice_double, physical_m2_s, given_as) };
    if(written < 0 || static_cast<std::size_t>(written) >= text_capacity) {
        throw SetupError("Diffusivity::describe(): the description does not fit its text buffer");
    }
    const std::string description{ text };
    return description;
}
