#pragma once

#include "setup/core/quantity.hpp"

/// @brief Scale between absolute temperatures and the LBM's temperatures (TEMPERATURE extension).
///
/// The core keeps the lattice temperature near its average 1 and applies the buoyancy -f*beta_lbm*(T_lbm - 1) with the
/// volume force f. Here the lattice temperature is 1 at the reference temperature and changes by 1 per difference,
/// T_lbm = 1 + (T - reference)/difference, so the physical buoyancy -g*beta*(T - reference) needs
/// beta_lbm = beta*difference (expansion()).
class TemperatureScale {
public:
    /// The scale between a cold and a hot temperature: they become 0.5 and 1.5, their mean 1.
    static TemperatureScale between(Temperature cold, Temperature hot) {
        return TemperatureScale(Temperature::from_si(0.5f * (hot.si() + cold.si())), hot - cold);
    }

    /// @param reference  the temperature that becomes the lattice temperature 1
    /// @param difference the temperature difference that becomes the lattice temperature difference 1
    TemperatureScale(Temperature reference, Temperature difference) : reference_(reference), difference_(difference) {}

    /// The lattice temperature of an absolute temperature.
    float lattice(Temperature t) const { return 1.0f + (t.si() - reference_.si()) / difference_.si(); }

    /// The lattice thermal expansion coefficient of a physical one.
    float expansion(ThermalExpansion beta) const { return beta * difference_; }

    Temperature reference() const { return reference_; }   ///< lattice temperature 1
    Temperature difference() const { return difference_; } ///< lattice temperature difference 1

private:
    Temperature reference_;
    Temperature difference_;
};
