#pragma once

#include <cmath>
#include <compare>
#include <concepts>
// no <numbers>: the core's utilities.hpp defines a macro named pi, which breaks std::numbers::pi

/// @file quantity.hpp
/// @brief Physical quantities with their dimension checked at compile time, and their literals.

/// @brief A physical quantity, stored in SI units as a float, with its dimension checked at compile time.
///
/// The template arguments are the exponents of length (m), mass (kg), time (s) and temperature (K):
/// Quantity<1, 0, -1> is a speed in m/s. Mixing dimensions (a Length where a Speed is expected) does not compile,
/// and a bare number does not convert to a quantity: write 2.0_m or Length::from_si(2.0f).
/// @tparam L the exponent of length
/// @tparam M the exponent of mass
/// @tparam T the exponent of time
/// @tparam K the exponent of temperature
template<int L, int M, int T, int K = 0>
class Quantity {
public:
    /// Zero.
    constexpr Quantity() = default;

    /// @brief A quantity from its SI value.
    /// @param si the value in SI units
    /// @return the quantity
    static constexpr Quantity from_si(float si) { Quantity q; q.si_ = si; return q; }

    /// @return the value in SI units
    constexpr float si() const { return si_; }

    /// @brief Three-way comparison of two quantities of the same dimension.
    /// @return the ordering
    constexpr auto operator<=>(const Quantity&) const = default;

    /// @return the negated quantity
    constexpr Quantity operator-() const { return from_si(-si_); }

    /// @param other the quantity to add
    /// @return this quantity, increased
    constexpr Quantity& operator+=(Quantity other) { si_ += other.si_; return *this; }

    /// @param other the quantity to subtract
    /// @return this quantity, decreased
    constexpr Quantity& operator-=(Quantity other) { si_ -= other.si_; return *this; }

    /// @param factor the plain number to multiply by
    /// @return this quantity, scaled
    constexpr Quantity& operator*=(float factor) { si_ *= factor; return *this; }

    /// @param divisor the plain number to divide by
    /// @return this quantity, scaled
    constexpr Quantity& operator/=(float divisor) { si_ /= divisor; return *this; }

    /// @param a a quantity
    /// @param b a quantity of the same dimension
    /// @return their sum
    friend constexpr Quantity operator+(Quantity a, Quantity b) { return a += b; }

    /// @param a a quantity
    /// @param b a quantity of the same dimension
    /// @return their difference
    friend constexpr Quantity operator-(Quantity a, Quantity b) { return a -= b; }

    /// @param q a quantity
    /// @param factor a plain number
    /// @return the quantity scaled by the number
    friend constexpr Quantity operator*(Quantity q, float factor) { return q *= factor; }

    /// @param factor a plain number
    /// @param q a quantity
    /// @return the quantity scaled by the number
    friend constexpr Quantity operator*(float factor, Quantity q) { return q *= factor; }

    /// @param q a quantity
    /// @param divisor a plain number
    /// @return the quantity divided by the number
    friend constexpr Quantity operator/(Quantity q, float divisor) { return q /= divisor; }

    /// @brief Square root of a quantity whose dimension exponents are all even: sqrt(9.81_mps2*2.0_m) is a Speed.
    /// @param q the quantity
    /// @return the square root, with halved exponents
    friend auto sqrt(Quantity q) requires(L % 2 == 0 && M % 2 == 0 && T % 2 == 0 && K % 2 == 0) {
        return Quantity<L / 2, M / 2, T / 2, K / 2>::from_si(std::sqrt(q.si_));
    }

private:
    float si_ = 0.0f; ///< the value in SI units
};

/// Implementation details of the quantity operators.
namespace quantity_detail {
/// @brief A quantity of the given dimension, or a plain number when the dimension is none.
/// @tparam L, M, T, K the exponents of length, mass, time and temperature
/// @param si the value in SI units
/// @return the quantity, or the number itself if all exponents are zero
template<int L, int M, int T, int K>
constexpr auto make(float si) {
    if constexpr (L == 0 && M == 0 && T == 0 && K == 0) return si;
    else return Quantity<L, M, T, K>::from_si(si);
}
} // namespace quantity_detail

/// @brief The product of two quantities: the exponents add (10_m / 2_s is a Speed).
/// @param a a quantity
/// @param b a quantity of any dimension
/// @return the product, or a plain number when the dimensions cancel
template<int L1, int M1, int T1, int K1, int L2, int M2, int T2, int K2>
constexpr auto operator*(Quantity<L1, M1, T1, K1> a, Quantity<L2, M2, T2, K2> b) {
    return quantity_detail::make<L1 + L2, M1 + M2, T1 + T2, K1 + K2>(a.si() * b.si());
}

/// @brief The quotient of two quantities: the exponents subtract.
/// @param a the dividend
/// @param b the divisor, of any dimension
/// @return the quotient, or a plain number when the dimensions cancel
template<int L1, int M1, int T1, int K1, int L2, int M2, int T2, int K2>
constexpr auto operator/(Quantity<L1, M1, T1, K1> a, Quantity<L2, M2, T2, K2> b) {
    return quantity_detail::make<L1 - L2, M1 - M2, T1 - T2, K1 - K2>(a.si() / b.si());
}

/// @brief A plain number divided by a quantity: the exponents negate (1 / 2_s is a Frequency).
/// @param a the number
/// @param b the quantity
/// @return the quotient
template<int L, int M, int T, int K>
constexpr auto operator/(float a, Quantity<L, M, T, K> b) {
    return quantity_detail::make<-L, -M, -T, -K>(a / b.si());
}

/// A type with an SI value: any Quantity.
template<typename Q>
concept PhysicalQuantity = requires(Q q) { { q.si() } -> std::same_as<float>; };

using Length             = Quantity< 1, 0,  0>;    ///< m
using Area               = Quantity< 2, 0,  0>;    ///< m²
using Volume             = Quantity< 3, 0,  0>;    ///< m³
using Duration           = Quantity< 0, 0,  1>;    ///< s
using Frequency          = Quantity< 0, 0, -1>;    ///< 1/s
using Speed              = Quantity< 1, 0, -1>;    ///< m/s
using Acceleration       = Quantity< 1, 0, -2>;    ///< m/s²
using Mass               = Quantity< 0, 1,  0>;    ///< kg
using Density            = Quantity<-3, 1,  0>;    ///< kg/m³
using KinematicViscosity = Quantity< 2, 0, -1>;    ///< m²/s (also thermal diffusivity)
using Force              = Quantity< 1, 1, -2>;    ///< N
using Pressure           = Quantity<-1, 1, -2>;    ///< Pa
using SurfaceTension     = Quantity< 0, 1, -2>;    ///< N/m
using Temperature        = Quantity< 0, 0,  0, 1>; ///< K
using ThermalExpansion   = Quantity< 0, 0,  0, -1>; ///< 1/K
using VolumeFlowRate     = Quantity< 3, 0, -1>;    ///< m³/s

/// @brief Three components of a quantity along X, Y and Z (Z is the height), such as a position or a velocity.
/// @tparam Q the quantity of each component
template<typename Q>
struct Vector3 {
    Q x{}; ///< along X
    Q y{}; ///< along Y
    Q z{}; ///< along Z, the height

    /// @param a a vector
    /// @param b a vector of the same quantity
    /// @return their sum, component by component
    friend constexpr Vector3 operator+(Vector3 a, Vector3 b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }

    /// @param a a vector
    /// @param b a vector of the same quantity
    /// @return their difference, component by component
    friend constexpr Vector3 operator-(Vector3 a, Vector3 b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }

    /// @param v a vector
    /// @param factor a plain number
    /// @return the vector scaled by the number
    friend constexpr Vector3 operator*(Vector3 v, float factor) { return { v.x * factor, v.y * factor, v.z * factor }; }

    /// @param factor a plain number
    /// @param v a vector
    /// @return the vector scaled by the number
    friend constexpr Vector3 operator*(float factor, Vector3 v) { return v * factor; }
};

/// @brief The length of a vector of quantities.
/// @tparam Q the quantity of the components
/// @param v the vector
/// @return the Euclidean length, of the same quantity
template<typename Q>
Q magnitude(const Vector3<Q>& v) {
    return Q::from_si(std::sqrt(v.x.si() * v.x.si() + v.y.si() * v.y.si() + v.z.si() * v.z.si()));
}

/// A point in the domain in metres, measured from its origin corner (the cell at 0, 0, 0): {0.5_m, 1.0_m, 0.2_m}.
using Position = Vector3<Length>;
using Velocity = Vector3<Speed>;                  ///< m/s
using AccelerationVector = Vector3<Acceleration>; ///< m/s², e.g. gravity {0_mps2, 0_mps2, -9.81_mps2}
using ForceVector = Vector3<Force>;               ///< N

/// @brief A plane angle: dimensionless, but a distinct type so that degrees cannot be passed where radians are meant.
///
/// Stored in degrees: the core takes degrees (rotations, cameras), so they pass through unchanged.
class Angle {
public:
    /// Zero.
    constexpr Angle() = default;

    /// @param deg the angle in degrees
    /// @return the angle
    static constexpr Angle from_deg(float deg) { Angle a; a.deg_ = deg; return a; }

    /// @param rad the angle in radians
    /// @return the angle
    static constexpr Angle from_rad(float rad) { return from_deg(rad * (180.0f / half_turn_rad)); }

    /// @return the angle in degrees
    constexpr float deg() const { return deg_; }

    /// @return the angle in radians
    constexpr float rad() const { return deg_ * (half_turn_rad / 180.0f); }

    /// @brief Three-way comparison of two angles.
    /// @return the ordering
    constexpr auto operator<=>(const Angle&) const = default;

    /// @return the opposite angle
    constexpr Angle operator-() const { return from_deg(-deg_); }

    /// @param a an angle
    /// @param b an angle
    /// @return their sum
    friend constexpr Angle operator+(Angle a, Angle b) { return from_deg(a.deg_ + b.deg_); }

    /// @param a an angle
    /// @param b an angle
    /// @return their difference
    friend constexpr Angle operator-(Angle a, Angle b) { return from_deg(a.deg_ - b.deg_); }

    /// @param a an angle
    /// @param factor a plain number
    /// @return the angle scaled by the number
    friend constexpr Angle operator*(Angle a, float factor) { return from_deg(a.deg_ * factor); }

    /// @param factor a plain number
    /// @param a an angle
    /// @return the angle scaled by the number
    friend constexpr Angle operator*(float factor, Angle a) { return from_deg(a.deg_ * factor); }

private:
    static constexpr float half_turn_rad = 3.14159265358979f; ///< π: half a turn in radians
    float deg_ = 0.0f; ///< the angle in degrees
};

/// A rate of rotation, written as an angle per duration: 180_deg / 1.0_s is half a turn per second.
class AngularSpeed {
public:
    /// Zero.
    constexpr AngularSpeed() = default;

    /// @param deg_per_s the rate in degrees per second
    /// @return the angular speed
    static constexpr AngularSpeed from_deg_per_s(float deg_per_s) { AngularSpeed w; w.deg_per_s_ = deg_per_s; return w; }

    /// @return the rate in degrees per second
    constexpr float deg_per_s() const { return deg_per_s_; }

    /// @return the rate in radians per second
    constexpr float rad_per_s() const { return (Angle::from_deg(deg_per_s_)).rad(); }

    /// @brief Three-way comparison of two angular speeds.
    /// @return the ordering
    constexpr auto operator<=>(const AngularSpeed&) const = default;

private:
    float deg_per_s_ = 0.0f; ///< the rate in degrees per second
};

/// @brief An angle turned in a duration.
/// @param angle the angle
/// @param time the duration
/// @return the angular speed
constexpr AngularSpeed operator/(Angle angle, Duration time) { return AngularSpeed::from_deg_per_s(angle.deg() / time.si()); }

/// @brief How compressible the simulation makes the flow: the reference velocity (Simulation's) as a fraction of
/// the lattice's speed of sound.
///
/// The LBM's speed of sound is not the fluid's: it is 1/sqrt(3) cells per time step, so this number sets the time step.
/// A lower one is more accurate (the compressibility error grows as its square) and takes more time steps; 0.1 to 0.3 is
/// usual. The default, about 0.17, is a reference velocity of 0.1 cells per time step.
class LatticeMach {
public:
    /// The default: a reference velocity of 0.1 cells per time step, a Mach number of about 0.17.
    constexpr LatticeMach() = default;

    /// @param mach the reference velocity as a fraction of the lattice's speed of sound
    constexpr explicit LatticeMach(float mach) : lattice_speed_(mach * sound_speed) {}

    /// @return the Mach number
    constexpr float value() const { return lattice_speed_ / sound_speed; }

    /// @return the reference velocity in cells per time step
    constexpr float lattice_speed() const { return lattice_speed_; }

private:
    static constexpr float sound_speed = 0.57735027f; ///< the lattice's speed of sound, 1/sqrt(3) cells per time step
    float lattice_speed_ = 0.1f; ///< the reference velocity in cells per time step
};

/// A distance in units of the model's reference length (set with Model::length()), e.g. 0.5_lengths.
class ModelLengths {
public:
    /// Zero.
    constexpr ModelLengths() = default;

    /// @param lengths the distance in model lengths
    /// @return the distance
    static constexpr ModelLengths of(float lengths) { ModelLengths m; m.value_ = lengths; return m; }

    /// @return the distance in model lengths
    constexpr float value() const { return value_; }

    /// @brief Three-way comparison of two distances.
    /// @return the ordering
    constexpr auto operator<=>(const ModelLengths&) const = default;

private:
    float value_ = 0.0f; ///< the distance in model lengths
};

/// Device memory, stored in MB (1 GB = 1024 MB, as the core's VRAM figures).
class MemorySize {
public:
    /// Zero.
    constexpr MemorySize() = default;

    /// @param mb the size in MB
    /// @return the size
    static constexpr MemorySize from_mb(unsigned int mb) { MemorySize m; m.mb_ = mb; return m; }

    /// @return the size in MB
    constexpr unsigned int mb() const { return mb_; }

    /// @brief Three-way comparison of two sizes.
    /// @return the ordering
    constexpr auto operator<=>(const MemorySize&) const = default;

private:
    unsigned int mb_ = 0u; ///< the size in MB
};

/// @name Literals of the dimensionless types
/// @{

/// @param v a distance in model lengths
/// @return the distance
consteval ModelLengths operator""_lengths(long double v) { return ModelLengths::of(static_cast<float>(v)); }

/// @copydoc operator""_lengths(long double)
consteval ModelLengths operator""_lengths(unsigned long long v) { return ModelLengths::of(static_cast<float>(v)); }

/// @param v a size in megabytes
/// @return the size
consteval MemorySize operator""_mb(unsigned long long v) { return MemorySize::from_mb(static_cast<unsigned int>(v)); }

/// @param v a size in gigabytes (1024 MB each)
/// @return the size
consteval MemorySize operator""_gb(unsigned long long v) { return MemorySize::from_mb(static_cast<unsigned int>(v * 1024ull)); }
/// @}

/// @name Literals of the quantities, e.g. 2.0_m, 36_kmh, 20.0_C (absolute temperature); integer and floating-point forms
///
/// consteval: always computed by the compiler, so /fp:fast cannot change a literal's value (x/3.6f evaluated at run
/// time becomes x*(1/3.6f), one ulp off). Each value is the literal times the SI value of its unit, plus an offset for
/// degrees Celsius.
/// @{

/// @param v a value in metres
/// @return the length
consteval Length operator""_m(long double v) { return Length::from_si(static_cast<float>(v)); }

/// @copydoc operator""_m(long double)
consteval Length operator""_m(unsigned long long v) { return Length::from_si(static_cast<float>(v)); }

/// @param v a value in centimetres
/// @return the length
consteval Length operator""_cm(long double v) { return Length::from_si(static_cast<float>(v) * 0.01f); }

/// @copydoc operator""_cm(long double)
consteval Length operator""_cm(unsigned long long v) { return Length::from_si(static_cast<float>(v) * 0.01f); }

/// @param v a value in millimetres
/// @return the length
consteval Length operator""_mm(long double v) { return Length::from_si(static_cast<float>(v) * 0.001f); }

/// @copydoc operator""_mm(long double)
consteval Length operator""_mm(unsigned long long v) { return Length::from_si(static_cast<float>(v) * 0.001f); }

/// @param v a value in kilometres
/// @return the length
consteval Length operator""_km(long double v) { return Length::from_si(static_cast<float>(v) * 1000.0f); }

/// @copydoc operator""_km(long double)
consteval Length operator""_km(unsigned long long v) { return Length::from_si(static_cast<float>(v) * 1000.0f); }

/// @param v a value in seconds
/// @return the duration
consteval Duration operator""_s(long double v) { return Duration::from_si(static_cast<float>(v)); }

/// @copydoc operator""_s(long double)
consteval Duration operator""_s(unsigned long long v) { return Duration::from_si(static_cast<float>(v)); }

/// @param v a value in milliseconds
/// @return the duration
consteval Duration operator""_ms(long double v) { return Duration::from_si(static_cast<float>(v) * 0.001f); }

/// @copydoc operator""_ms(long double)
consteval Duration operator""_ms(unsigned long long v) { return Duration::from_si(static_cast<float>(v) * 0.001f); }

/// @param v a value in microseconds
/// @return the duration
consteval Duration operator""_us(long double v) { return Duration::from_si(static_cast<float>(v) * 1.0E-6f); }

/// @copydoc operator""_us(long double)
consteval Duration operator""_us(unsigned long long v) { return Duration::from_si(static_cast<float>(v) * 1.0E-6f); }

/// @param v a value in minutes
/// @return the duration
consteval Duration operator""_min(long double v) { return Duration::from_si(static_cast<float>(v) * 60.0f); }

/// @copydoc operator""_min(long double)
consteval Duration operator""_min(unsigned long long v) { return Duration::from_si(static_cast<float>(v) * 60.0f); }

/// @param v a value in hertz
/// @return the frequency
consteval Frequency operator""_Hz(long double v) { return Frequency::from_si(static_cast<float>(v)); }

/// @copydoc operator""_Hz(long double)
consteval Frequency operator""_Hz(unsigned long long v) { return Frequency::from_si(static_cast<float>(v)); }

/// @param v a value in metres per second
/// @return the speed
consteval Speed operator""_mps(long double v) { return Speed::from_si(static_cast<float>(v)); }

/// @copydoc operator""_mps(long double)
consteval Speed operator""_mps(unsigned long long v) { return Speed::from_si(static_cast<float>(v)); }

/// @param v a value in metres per second squared
/// @return the acceleration
consteval Acceleration operator""_mps2(long double v) { return Acceleration::from_si(static_cast<float>(v)); }

/// @copydoc operator""_mps2(long double)
consteval Acceleration operator""_mps2(unsigned long long v) { return Acceleration::from_si(static_cast<float>(v)); }

/// @param v a value in kilograms
/// @return the mass
consteval Mass operator""_kg(long double v) { return Mass::from_si(static_cast<float>(v)); }

/// @copydoc operator""_kg(long double)
consteval Mass operator""_kg(unsigned long long v) { return Mass::from_si(static_cast<float>(v)); }

/// @param v a value in kilograms per cubic metre
/// @return the density
consteval Density operator""_kgpm3(long double v) { return Density::from_si(static_cast<float>(v)); }

/// @copydoc operator""_kgpm3(long double)
consteval Density operator""_kgpm3(unsigned long long v) { return Density::from_si(static_cast<float>(v)); }

/// @param v a value in square metres per second
/// @return the viscosity (or thermal diffusivity)
consteval KinematicViscosity operator""_m2ps(long double v) { return KinematicViscosity::from_si(static_cast<float>(v)); }

/// @copydoc operator""_m2ps(long double)
consteval KinematicViscosity operator""_m2ps(unsigned long long v) { return KinematicViscosity::from_si(static_cast<float>(v)); }

/// @param v a value in cubic metres per second
/// @return the flow rate
consteval VolumeFlowRate operator""_m3ps(long double v) { return VolumeFlowRate::from_si(static_cast<float>(v)); }

/// @copydoc operator""_m3ps(long double)
consteval VolumeFlowRate operator""_m3ps(unsigned long long v) { return VolumeFlowRate::from_si(static_cast<float>(v)); }

/// @param v a value in newtons
/// @return the force
consteval Force operator""_N(long double v) { return Force::from_si(static_cast<float>(v)); }

/// @copydoc operator""_N(long double)
consteval Force operator""_N(unsigned long long v) { return Force::from_si(static_cast<float>(v)); }

/// @param v a value in pascals
/// @return the pressure
consteval Pressure operator""_Pa(long double v) { return Pressure::from_si(static_cast<float>(v)); }

/// @copydoc operator""_Pa(long double)
consteval Pressure operator""_Pa(unsigned long long v) { return Pressure::from_si(static_cast<float>(v)); }

/// @param v a value in newtons per metre
/// @return the surface tension
consteval SurfaceTension operator""_Npm(long double v) { return SurfaceTension::from_si(static_cast<float>(v)); }

/// @copydoc operator""_Npm(long double)
consteval SurfaceTension operator""_Npm(unsigned long long v) { return SurfaceTension::from_si(static_cast<float>(v)); }

/// @param v a value in kelvin
/// @return the temperature
consteval Temperature operator""_K(long double v) { return Temperature::from_si(static_cast<float>(v)); }

/// @copydoc operator""_K(long double)
consteval Temperature operator""_K(unsigned long long v) { return Temperature::from_si(static_cast<float>(v)); }

/// @param v a value in degrees Celsius
/// @return the absolute temperature
consteval Temperature operator""_C(long double v) { return Temperature::from_si(static_cast<float>(v) + 273.15f); }

/// @copydoc operator""_C(long double)
consteval Temperature operator""_C(unsigned long long v) { return Temperature::from_si(static_cast<float>(v) + 273.15f); }

/// @brief km/h divided by 3.6 (not multiplied by 1/3.6), which is how speeds in km/h are usually written:
/// 300.0_kmh == 300.0f/3.6f.
/// @param v a value in kilometres per hour
/// @return the speed
consteval Speed operator""_kmh(long double v) { return Speed::from_si(static_cast<float>(v) / 3.6f); }

/// @copydoc operator""_kmh(long double)
consteval Speed operator""_kmh(unsigned long long v) { return Speed::from_si(static_cast<float>(v) / 3.6f); }

/// @param v an angle in degrees
/// @return the angle
consteval Angle operator""_deg(long double v) { return Angle::from_deg(static_cast<float>(v)); }

/// @copydoc operator""_deg(long double)
consteval Angle operator""_deg(unsigned long long v) { return Angle::from_deg(static_cast<float>(v)); }

/// @param v an angle in radians
/// @return the angle
consteval Angle operator""_rad(long double v) { return Angle::from_rad(static_cast<float>(v)); }

/// @copydoc operator""_rad(long double)
consteval Angle operator""_rad(unsigned long long v) { return Angle::from_rad(static_cast<float>(v)); }
/// @}
