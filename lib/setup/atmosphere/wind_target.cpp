#include "setup/atmosphere/wind_target.hpp"
#include "setup/core/setup_error.hpp"

#include <cstddef>
#include <cstdio>

PowerLawWind::PowerLawWind(si::Speed reference_speed, si::Length reference_height, double exponent, si::Angle from)
    : speed_(reference_speed), height_(reference_height), exponent_(exponent), from_(from) {
    if(!(reference_speed > au::ZERO)) {
        throw SetupError("PowerLawWind: the reference speed must be more than 0 (no wind: no WindTarget)");
    }
    if(!(reference_height > au::ZERO)) {
        throw SetupError("PowerLawWind: the reference height must be more than 0");
    }
    const bool finite{ std::isfinite(exponent) };
    const bool negative{ exponent < 0.0 };
    if(!finite || negative) {
        throw SetupError("PowerLawWind: the exponent must be finite and at least 0");
    }
}

si::Speed PowerLawWind::speed(si::Length h) const {
    si::Speed u{ au::ZERO };
    if(!(h <= au::ZERO)) {
        const auto ratio = h / height_;
        const double height_ratio{ au::as_raw_number(ratio) };
        const double factor{ shape(height_ratio, exponent_) };
        u = speed_ * factor;
    }
    return u;
}

HorizontalWind PowerLawWind::at(si::Length h) const {
    const si::Speed u{ speed(h) }; // from the direction: toward -sin, -cos of it
    const si::Speed minus_u{ -u };
    const auto sin_from = au::sin(from_);
    const auto cos_from = au::cos(from_);
    const si::Speed x{ minus_u * sin_from };
    const si::Speed y{ minus_u * cos_from };
    const HorizontalWind wind{ .x = x, .y = y };
    return wind;
}

std::string PowerLawWind::describe() const {
    const double speed_m_s{ speed_.in(au::meters / au::second) };
    const double height_m{ height_.in(au::meters) };
    const double exponent{ exponent_ };
    const double from_deg{ from_.in(au::degrees) };
    constexpr std::size_t text_capacity{ 160u };
    char text[text_capacity]{};
    const int length = std::snprintf(text, sizeof(text), "power law: %.2f m/s at %.0f m, exponent %.3f, from %.0f deg", speed_m_s, height_m, exponent, from_deg);
    const bool cut{ length < 0 || static_cast<std::size_t>(length) >= text_capacity };
    if(cut) {
        throw SetupError("PowerLawWind::describe(): the text does not fit its buffer");
    }
    const std::string description{ text };
    return description;
}
