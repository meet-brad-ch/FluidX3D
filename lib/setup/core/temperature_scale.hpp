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
    /// @brief The scale between a cold and a hot temperature: they become 0.5 and 1.5, their mean 1.
    /// @param cold the cold temperature
    /// @param hot the hot temperature
    /// @return the scale
    static TemperatureScale between(Temperature cold, Temperature hot) {
        return TemperatureScale(Temperature::from_si(0.5f * (hot.si() + cold.si())), hot - cold);
    }

    /// @param reference  the temperature that becomes the lattice temperature 1
    /// @param difference the temperature difference that becomes the lattice temperature difference 1
    TemperatureScale(Temperature reference, Temperature difference) : reference_(reference), difference_(difference) {}

    /// @param t an absolute temperature
    /// @return the lattice temperature
    float lattice(Temperature t) const { return 1.0f + (t.si() - reference_.si()) / difference_.si(); }

    /// @param beta a physical thermal expansion coefficient
    /// @return the lattice thermal expansion coefficient
    float expansion(ThermalExpansion beta) const { return beta * difference_; }

    /// @return the temperature that is the lattice temperature 1
    Temperature reference() const { return reference_; }

    /// @return the temperature difference that is the lattice temperature difference 1
    Temperature difference() const { return difference_; }

private:
    Temperature reference_;  ///< the lattice temperature 1
    Temperature difference_; ///< the lattice temperature difference 1
};
