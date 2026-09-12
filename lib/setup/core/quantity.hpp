#pragma once

#include <compare>
#include <concepts>
// no <numbers>: the core's utilities.hpp defines a macro named pi, which breaks std::numbers::pi

// Physical quantities with their dimension checked at compile time. Values are stored in SI units as float.
// The template arguments are the exponents of length (m), mass (kg), time (s) and temperature (K):
// Quantity<1, 0, -1> is a speed in m/s. Mixing dimensions (a Length where a Speed is expected) does not compile,
// and a bare number does not convert to a quantity: write 2.0_m or Length::from_si(2.0f).
template<int L, int M, int T, int K = 0>
class Quantity {
public:
    constexpr Quantity() = default;
    static constexpr Quantity from_si(float si) { Quantity q; q.si_ = si; return q; }
    constexpr float si() const { return si_; }

    constexpr auto operator<=>(const Quantity&) const = default;

    constexpr Quantity operator-() const { return from_si(-si_); }
    constexpr Quantity& operator+=(Quantity other) { si_ += other.si_; return *this; }
    constexpr Quantity& operator-=(Quantity other) { si_ -= other.si_; return *this; }
    constexpr Quantity& operator*=(float factor) { si_ *= factor; return *this; }
    constexpr Quantity& operator/=(float divisor) { si_ /= divisor; return *this; }

    friend constexpr Quantity operator+(Quantity a, Quantity b) { return a += b; }
    friend constexpr Quantity operator-(Quantity a, Quantity b) { return a -= b; }
    friend constexpr Quantity operator*(Quantity q, float factor) { return q *= factor; }
    friend constexpr Quantity operator*(float factor, Quantity q) { return q *= factor; }
    friend constexpr Quantity operator/(Quantity q, float divisor) { return q /= divisor; }

private:
    float si_ = 0.0f;
};

namespace quantity_detail {
template<int L, int M, int T, int K>
constexpr auto make(float si) { // dimensionless results are plain numbers
    if constexpr (L == 0 && M == 0 && T == 0 && K == 0) return si;
    else return Quantity<L, M, T, K>::from_si(si);
}
} // namespace quantity_detail

template<int L1, int M1, int T1, int K1, int L2, int M2, int T2, int K2>
constexpr auto operator*(Quantity<L1, M1, T1, K1> a, Quantity<L2, M2, T2, K2> b) {
    return quantity_detail::make<L1 + L2, M1 + M2, T1 + T2, K1 + K2>(a.si() * b.si());
}

template<int L1, int M1, int T1, int K1, int L2, int M2, int T2, int K2>
constexpr auto operator/(Quantity<L1, M1, T1, K1> a, Quantity<L2, M2, T2, K2> b) {
    return quantity_detail::make<L1 - L2, M1 - M2, T1 - T2, K1 - K2>(a.si() / b.si());
}

template<int L, int M, int T, int K>
constexpr auto operator/(float a, Quantity<L, M, T, K> b) {
    return quantity_detail::make<-L, -M, -T, -K>(a / b.si());
}

template<typename Q>
concept PhysicalQuantity = requires(Q q) { { q.si() } -> std::same_as<float>; };

using Length             = Quantity< 1, 0,  0>;    // m
using Area               = Quantity< 2, 0,  0>;    // m²
using Volume             = Quantity< 3, 0,  0>;    // m³
using Duration           = Quantity< 0, 0,  1>;    // s
using Frequency          = Quantity< 0, 0, -1>;    // 1/s
using Speed              = Quantity< 1, 0, -1>;    // m/s
using Acceleration       = Quantity< 1, 0, -2>;    // m/s²
using Mass               = Quantity< 0, 1,  0>;    // kg
using Density            = Quantity<-3, 1,  0>;    // kg/m³
using KinematicViscosity = Quantity< 2, 0, -1>;    // m²/s (also thermal diffusivity)
using Force              = Quantity< 1, 1, -2>;    // N
using Pressure           = Quantity<-1, 1, -2>;    // Pa
using SurfaceTension     = Quantity< 0, 1, -2>;    // N/m
using Temperature        = Quantity< 0, 0,  0, 1>; // K
using ThermalExpansion   = Quantity< 0, 0,  0, -1>; // 1/K

// Plane angle: dimensionless, but a distinct type so that degrees cannot be passed where radians are meant.
class Angle {
public:
    constexpr Angle() = default;
    static constexpr Angle from_rad(float rad) { Angle a; a.rad_ = rad; return a; }
    static constexpr Angle from_deg(float deg) { return from_rad(deg * (half_turn_rad / 180.0f)); }
    constexpr float rad() const { return rad_; }
    constexpr float deg() const { return rad_ * (180.0f / half_turn_rad); }

    constexpr auto operator<=>(const Angle&) const = default;

    constexpr Angle operator-() const { return from_rad(-rad_); }
    friend constexpr Angle operator+(Angle a, Angle b) { return from_rad(a.rad_ + b.rad_); }
    friend constexpr Angle operator-(Angle a, Angle b) { return from_rad(a.rad_ - b.rad_); }
    friend constexpr Angle operator*(Angle a, float factor) { return from_rad(a.rad_ * factor); }
    friend constexpr Angle operator*(float factor, Angle a) { return from_rad(a.rad_ * factor); }

private:
    static constexpr float half_turn_rad = 3.14159265358979f; // π
    float rad_ = 0.0f;
};

// Literals, e.g. 2.0_m, 36_kmh, 20.0_C (absolute temperature); integer and floating-point forms.
#define FLUIDX3D_QUANTITY_LITERAL(suffix, Type, factor, offset) \
    constexpr Type operator""suffix(long double v) { return Type::from_si(static_cast<float>(v) * (factor) + (offset)); } \
    constexpr Type operator""suffix(unsigned long long v) { return Type::from_si(static_cast<float>(v) * (factor) + (offset)); }

FLUIDX3D_QUANTITY_LITERAL(_m,     Length,             1.0f,        0.0f)
FLUIDX3D_QUANTITY_LITERAL(_cm,    Length,             0.01f,       0.0f)
FLUIDX3D_QUANTITY_LITERAL(_mm,    Length,             0.001f,      0.0f)
FLUIDX3D_QUANTITY_LITERAL(_km,    Length,             1000.0f,     0.0f)
FLUIDX3D_QUANTITY_LITERAL(_s,     Duration,           1.0f,        0.0f)
FLUIDX3D_QUANTITY_LITERAL(_min,   Duration,           60.0f,       0.0f)
FLUIDX3D_QUANTITY_LITERAL(_Hz,    Frequency,          1.0f,        0.0f)
FLUIDX3D_QUANTITY_LITERAL(_mps,   Speed,              1.0f,        0.0f)
FLUIDX3D_QUANTITY_LITERAL(_kmh,   Speed,              1.0f / 3.6f, 0.0f)
FLUIDX3D_QUANTITY_LITERAL(_mps2,  Acceleration,       1.0f,        0.0f)
FLUIDX3D_QUANTITY_LITERAL(_kg,    Mass,               1.0f,        0.0f)
FLUIDX3D_QUANTITY_LITERAL(_kgpm3, Density,            1.0f,        0.0f)
FLUIDX3D_QUANTITY_LITERAL(_m2ps,  KinematicViscosity, 1.0f,        0.0f)
FLUIDX3D_QUANTITY_LITERAL(_N,     Force,              1.0f,        0.0f)
FLUIDX3D_QUANTITY_LITERAL(_Pa,    Pressure,           1.0f,        0.0f)
FLUIDX3D_QUANTITY_LITERAL(_Npm,   SurfaceTension,     1.0f,        0.0f)
FLUIDX3D_QUANTITY_LITERAL(_K,     Temperature,        1.0f,        0.0f)
FLUIDX3D_QUANTITY_LITERAL(_C,     Temperature,        1.0f,        273.15f)

#undef FLUIDX3D_QUANTITY_LITERAL

constexpr Angle operator""_deg(long double v) { return Angle::from_deg(static_cast<float>(v)); }
constexpr Angle operator""_deg(unsigned long long v) { return Angle::from_deg(static_cast<float>(v)); }
constexpr Angle operator""_rad(long double v) { return Angle::from_rad(static_cast<float>(v)); }
constexpr Angle operator""_rad(unsigned long long v) { return Angle::from_rad(static_cast<float>(v)); }
