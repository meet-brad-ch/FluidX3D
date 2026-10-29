#include "setup/core/boussinesq_buoyancy.hpp"
#include "setup/core/setup_error.hpp"

#include <cstdio>

BoussinesqBuoyancy::BoussinesqBuoyancy(si::Acceleration gravity, si::Temperature reference, double carrier)
    : gravity_(gravity), reference_(reference), carrier_(carrier) {
    if(!(gravity > au::ZERO)) {
        throw SetupError("BoussinesqBuoyancy: the gravity must be more than 0");
    }
    const double reference_k{ si::in_kelvin(reference) };
    if(!(reference_k > 0.0)) {
        throw SetupError("BoussinesqBuoyancy: the reference temperature theta_0 must be more than 0 K");
    }
    if(!(carrier > 0.0)) {
        throw SetupError("BoussinesqBuoyancy: the carrier must be more than 0 (lattice units)");
    }
}

float BoussinesqBuoyancy::lattice_expansion(const si::LatticeConverter& converter) const {
    if(converter.temperature_unit() != si::LatticeConverter::kernel_temperature_unit) {
        throw SetupError("BoussinesqBuoyancy::lattice_expansion(): the converter's temperature unit must be 1 K (the kernel's lattice temperature unit)");
    }
    // beta g' = g dt^2/dx per (theta_0 carrier), per lattice temperature unit (1 K)
    const si::Duration step{ converter.step() };
    const si::Length cell{ converter.cell() };
    const auto speed = gravity_ * step;
    const auto distance = speed * step;
    const auto gravity_ratio = distance / cell;
    const double lattice_gravity{ au::as_raw_number(gravity_ratio) };
    const si::TemperatureDifference above_zero{ reference_ - si::kelvin_point(0.0) }; // theta_0 on the absolute scale
    const auto kelvin_ratio = si::LatticeConverter::kernel_temperature_unit / above_zero;
    const double per_kelvin{ au::as_raw_number(kelvin_ratio) };
    const double per_carrier{ lattice_gravity / carrier_ };
    const double expansion{ per_carrier * per_kelvin };
    const float lattice{ static_cast<float>(expansion) };
    return lattice;
}

void BoussinesqBuoyancy::validate(std::uint32_t levels) const {
    const double ramp{ density_ramp(levels) };
    if(ramp > density_limit) {
        const double carrier{ carrier_ };
        const unsigned level_count{ static_cast<unsigned>(levels) };
        const double limit{ density_limit };
        constexpr std::size_t text_capacity{ 200 };
        char text[text_capacity]{};
        const int written{ std::snprintf(text, sizeof(text), "BoussinesqBuoyancy: the carrier %.3g makes a density ramp of %.3g over %u levels, more than %.3g: a smaller carrier",
                                         carrier, ramp, level_count, limit) };
        if(written < 0 || static_cast<std::size_t>(written) >= text_capacity) {
            throw SetupError("BoussinesqBuoyancy::validate(): the density ramp is too large, and its message does not fit its text buffer");
        }
        throw SetupError(text);
    }
}
