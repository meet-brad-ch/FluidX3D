#pragma once

#include "setup/units/units.hpp"

#include <cmath>
#include <string>

/// @file wind_target.hpp
/// @brief The wind that a column is held to: WindTarget, PowerLawWind and HorizontalWind.

/// @brief The horizontal wind at one height: x points east, y points north.
struct HorizontalWind {
    si::Speed x{ au::ZERO }; ///< the speed toward +x (east)
    si::Speed y{ au::ZERO }; ///< the speed toward +y (north)
};

/// @brief The wind that a column is held to: a target wind for each height. The start builder sets the start wind
/// from it, and NudgingTerm nudges each level's mean wind toward it. A new wind shape is a new subclass.
class WindTarget {
public:
    virtual ~WindTarget() = default;
    /// @brief Gives the target wind at a height.
    /// @param h a height above the floor's top
    /// @return the target wind
    virtual HorizontalWind at(si::Length h) const = 0;
    /// @brief Gives the speed against which the wind's errors are given (WindHoldReport's relative errors).
    /// @return the reference speed, more than 0
    virtual si::Speed reference_speed() const = 0;
    /// @brief Describes the target for the run's log.
    /// @return one line
    /// @throws SetupError when the text does not fit its buffer (PowerLawWind)
    virtual std::string describe() const = 0;
};

/// @brief A power-law wind from one direction: u(h) = u_ref (h / h_ref)^exponent (the CBL example's wind_target()).
///
/// The direction is where the wind comes from, in degrees clockwise from north: 0 blows toward -y (south), 90 toward
/// -x (west).
/// @code
/// using namespace si::symbols;
/// const PowerLawWind wind(5.0 * m / s, 100.0 * m, 0.25, 0.0 * deg); // 5 m/s at 100 m, from the north
/// @endcode
class PowerLawWind final : public WindTarget {
public:
    /// @brief Makes the power-law wind.
    /// @param reference_speed u_ref: the speed at the reference height, more than 0
    /// @param reference_height h_ref: the height of u_ref above the floor's top, more than 0
    /// @param exponent the power law's exponent (a ratio), finite and 0 or more: 0.10 over the sea, 0.143 over open
    /// terrain, 0.25 the CBL example's
    /// @param from the direction the wind comes from, clockwise from north
    /// @throws SetupError when the speed or the height is not more than 0, or the exponent is negative or not finite
    PowerLawWind(si::Speed reference_speed, si::Length reference_height, double exponent, si::Angle from);

    /// @brief Gives the power law's shape in double: the one formula that the CBL wind and BoundaryBuilder's wind
    /// share (BoundaryBuilder uses the float form below).
    /// @param height_ratio the height over the reference height, h / h_ref, 0 or more
    /// @param exponent the power law's exponent, 0 or more
    /// @return (h / h_ref)^exponent
    static double shape(double height_ratio, double exponent) {
        // std::pow: the global pow is ambiguous with utilities.hpp's pow(float, uint), which would truncate the exponent
        const double factor{ std::pow(height_ratio, exponent) };
        return factor;
    }

    /// @brief Gives the power law's shape in float, for BoundaryBuilder's lattice numbers: the same formula as the
    /// double form, computed in float (std::pow's float overload).
    /// @param height_ratio the height over the reference height, h / h_ref, 0 or more
    /// @param exponent the power law's exponent, 0 or more
    /// @return (h / h_ref)^exponent
    static float shape(float height_ratio, float exponent) {
        const float factor{ std::pow(height_ratio, exponent) };
        return factor;
    }

    /// @brief Gives the wind speed at a height.
    /// @param h a height above the floor's top
    /// @return u_ref (h / h_ref)^exponent; 0 at and below the floor's top
    si::Speed speed(si::Length h) const;
    /// @brief Gives the wind at a height: the speed, blowing from the direction.
    /// @param h a height above the floor's top
    /// @return x = -u sin(from), y = -u cos(from)
    HorizontalWind at(si::Length h) const override;
    /// @brief Gives the speed against which the wind's errors are given.
    /// @return u_ref
    si::Speed reference_speed() const override { return speed_; }
    /// @brief Describes the wind for the run's log.
    /// @return one line: u_ref, h_ref, the exponent and the direction
    /// @throws SetupError when the text does not fit its buffer
    std::string describe() const override;

    /// @return h_ref, the height of the reference speed above the floor's top
    si::Length reference_height() const { return height_; }
    /// @return the power law's exponent (a ratio)
    double exponent() const { return exponent_; }
    /// @return the direction the wind comes from, clockwise from north
    si::Angle from() const { return from_; }

private:
    si::Speed speed_{ au::ZERO };  ///< u_ref: the speed at the reference height
    si::Length height_{ au::ZERO }; ///< h_ref: the reference height above the floor's top
    double exponent_{};             ///< the power law's exponent (a ratio)
    si::Angle from_{ au::ZERO };    ///< the direction the wind comes from, clockwise from north
};
