#pragma once

#include "setup/core/setup_error.hpp"

/// @brief The turbulent Prandtl number Pr_t of an eddy diffusivity: the subgrid eddies mix heat (and water) with the
/// eddy viscosity over Pr_t (SUBGRID). A ratio, without a unit.
///
/// The core takes 1/Pr_t, so the value is kept as its inverse: Pr_t 1/3 stays exactly 3 for the core.
/// @code
/// const TurbulentPrandtl pr = TurbulentPrandtl::one_third(); // pr.inverse() == 3.0
/// @endcode
class TurbulentPrandtl {
public:
    /// The inverse of Pr_t 1/3, exact in a float.
    static constexpr double one_third_inverse = 3.0;

    /// @brief Gives the turbulent Prandtl number 1/3, the usual value for a convective boundary layer.
    /// @return Pr_t 1/3 (the core gets exactly 3)
    static constexpr TurbulentPrandtl one_third() { return TurbulentPrandtl(one_third_inverse); }

    /// @brief Makes a turbulent Prandtl number from its value.
    /// @param pr the turbulent Prandtl number (a ratio), more than 0
    /// @return Pr_t, held as 1/pr
    /// @throws SetupError when pr is not more than 0 or not a number
    static TurbulentPrandtl of(double pr) {
        if(!(pr > 0.0)) {
            throw SetupError("TurbulentPrandtl::of(): Pr_t must be a number above 0");
        }
        const double inverse{ 1.0 / pr };
        const TurbulentPrandtl prandtl(inverse);
        return prandtl;
    }

    /// @brief Makes a turbulent Prandtl number from its inverse, the form that the core takes.
    /// @param inverse 1/Pr_t (a ratio), 0 or more (0: the subgrid eddies mix no heat)
    /// @return Pr_t
    /// @throws SetupError when the inverse is below 0 or not a number
    static TurbulentPrandtl from_inverse(double inverse) {
        const bool valid{ inverse >= 0.0 }; // false for NaN too
        if(!valid) {
            throw SetupError("TurbulentPrandtl::from_inverse(): 1/Pr_t must be at least 0");
        }
        const TurbulentPrandtl prandtl(inverse);
        return prandtl;
    }

    /// @brief Gives the inverse of the turbulent Prandtl number.
    /// @return 1/Pr_t (a ratio)
    constexpr double inverse() const { return inverse_; }

    /// @brief Gives the inverse of the turbulent Prandtl number in the form that the core takes.
    /// @return 1/Pr_t as the float of LBM::set_inv_prandtl_t()
    constexpr float lattice_inverse() const {
        const float inverse{ static_cast<float>(inverse_) };
        return inverse;
    }

private:
    /// @brief Makes the value from its checked inverse. Use one_third(), of() or from_inverse().
    /// @param inverse 1/Pr_t (a ratio), 0 or more
    constexpr explicit TurbulentPrandtl(double inverse) : inverse_(inverse) {}

    double inverse_{}; ///< 1/Pr_t (a ratio)
};
