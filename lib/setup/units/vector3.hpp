#pragma once

#include "setup/units/units.hpp"

#include <cmath>

/// @file vector3.hpp
/// @brief Vectors of one quantity along x, y and z (z is the height): Position, Velocity, AccelerationVector,
/// ForceVector and Direction.
///
/// Each vector is its own type with its own operators: + and - with the same type, unary -, a scale by a plain number
/// (on both sides) and a division by one, ==, and magnitude(). The operators between two types come after the types:
/// a Velocity times a Duration is a Position, a Position over a Duration is a Velocity, and a Velocity over a Duration
/// is an AccelerationVector.
/// @code
/// using namespace si::symbols;
/// const si::Velocity u{ 10.0 * m / s, 0.0 * m / s, -1.0 * m / s };
/// const si::Position moved = u * (2.0 * s);   // { 20 m, 0 m, -2 m }
/// const si::Speed speed = u.magnitude();      // about 10.05 m/s
/// @endcode
///
/// Au's scalar operators are templates; these operators are plain functions, so the compiler takes them first and a
/// vector times a quantity is never ambiguous.

namespace si {

/// @brief A point in the box, measured from its origin corner (the outer corner of the cell at 0, 0, 0).
struct Position {
    Length x{ au::ZERO }; ///< the distance along x
    Length y{ au::ZERO }; ///< the distance along y
    Length z{ au::ZERO }; ///< the distance along z, the height

    /// @brief Gives the distance from the origin corner: the square root of the sum of the squared components.
    /// @return the distance
    Length magnitude() const {
        const auto xx = x * x;
        const auto yy = y * y;
        const auto zz = z * z;
        const auto sum = xx + yy + zz;
        const Length length{ au::sqrt(sum) };
        return length;
    }

    /// @brief Adds two positions, component by component (a position and an offset).
    /// @param a the first position
    /// @param b the second position
    /// @return the sum
    friend Position operator+(const Position& a, const Position& b) {
        const Position sum{ a.x + b.x, a.y + b.y, a.z + b.z };
        return sum;
    }
    /// @brief Subtracts two positions, component by component: the offset from b to a.
    /// @param a the position to subtract from
    /// @param b the position to subtract
    /// @return a - b
    friend Position operator-(const Position& a, const Position& b) {
        const Position difference{ a.x - b.x, a.y - b.y, a.z - b.z };
        return difference;
    }
    /// @brief Negates each component.
    /// @param p the position
    /// @return the position mirrored through the origin
    friend Position operator-(const Position& p) {
        const Position opposite{ -p.x, -p.y, -p.z };
        return opposite;
    }
    /// @brief Scales each component by a plain number.
    /// @param p the position
    /// @param factor the plain number that multiplies each component
    /// @return the scaled position
    friend Position operator*(const Position& p, double factor) {
        const Position scaled{ p.x * factor, p.y * factor, p.z * factor };
        return scaled;
    }
    /// @brief Scales each component by a plain number (the factor first).
    /// @param factor the plain number that multiplies each component
    /// @param p the position
    /// @return the scaled position
    friend Position operator*(double factor, const Position& p) {
        const Position scaled{ factor * p.x, factor * p.y, factor * p.z };
        return scaled;
    }
    /// @brief Divides each component by a plain number.
    /// @param p the position
    /// @param divisor the divisor, not 0
    /// @return the divided position
    friend Position operator/(const Position& p, double divisor) {
        const Position divided{ p.x / divisor, p.y / divisor, p.z / divisor };
        return divided;
    }
    /// @brief Compares two positions component by component (exactly, as the components compare).
    /// @return true when all three components are equal
    friend bool operator==(const Position&, const Position&) = default;
};

/// @brief A velocity: a speed along x, y and z.
struct Velocity {
    Speed x{ au::ZERO }; ///< the speed along x
    Speed y{ au::ZERO }; ///< the speed along y
    Speed z{ au::ZERO }; ///< the speed along z, upward

    /// @brief Gives the speed: the square root of the sum of the squared components.
    /// @return the speed
    Speed magnitude() const {
        const auto xx = x * x;
        const auto yy = y * y;
        const auto zz = z * z;
        const auto sum = xx + yy + zz;
        const Speed speed{ au::sqrt(sum) };
        return speed;
    }

    /// @brief Adds two velocities, component by component.
    /// @param a the first velocity
    /// @param b the second velocity
    /// @return the sum
    friend Velocity operator+(const Velocity& a, const Velocity& b) {
        const Velocity sum{ a.x + b.x, a.y + b.y, a.z + b.z };
        return sum;
    }
    /// @brief Subtracts two velocities, component by component.
    /// @param a the velocity to subtract from
    /// @param b the velocity to subtract
    /// @return a - b
    friend Velocity operator-(const Velocity& a, const Velocity& b) {
        const Velocity difference{ a.x - b.x, a.y - b.y, a.z - b.z };
        return difference;
    }
    /// @brief Negates each component.
    /// @param u the velocity
    /// @return the velocity in the opposite direction
    friend Velocity operator-(const Velocity& u) {
        const Velocity opposite{ -u.x, -u.y, -u.z };
        return opposite;
    }
    /// @brief Scales each component by a plain number.
    /// @param u the velocity
    /// @param factor the plain number that multiplies each component
    /// @return the scaled velocity
    friend Velocity operator*(const Velocity& u, double factor) {
        const Velocity scaled{ u.x * factor, u.y * factor, u.z * factor };
        return scaled;
    }
    /// @brief Scales each component by a plain number (the factor first).
    /// @param factor the plain number that multiplies each component
    /// @param u the velocity
    /// @return the scaled velocity
    friend Velocity operator*(double factor, const Velocity& u) {
        const Velocity scaled{ factor * u.x, factor * u.y, factor * u.z };
        return scaled;
    }
    /// @brief Divides each component by a plain number.
    /// @param u the velocity
    /// @param divisor the divisor, not 0
    /// @return the divided velocity
    friend Velocity operator/(const Velocity& u, double divisor) {
        const Velocity divided{ u.x / divisor, u.y / divisor, u.z / divisor };
        return divided;
    }
    /// @brief Compares two velocities component by component (exactly).
    /// @return true when all three components are equal
    friend bool operator==(const Velocity&, const Velocity&) = default;
};

/// @brief An acceleration along x, y and z, such as gravity { 0, 0, -9.81 m/s² } or a force per mass.
struct AccelerationVector {
    Acceleration x{ au::ZERO }; ///< the acceleration along x
    Acceleration y{ au::ZERO }; ///< the acceleration along y
    Acceleration z{ au::ZERO }; ///< the acceleration along z, upward

    /// @brief Gives the acceleration's size: the square root of the sum of the squared components.
    /// @return the size
    Acceleration magnitude() const {
        const auto xx = x * x;
        const auto yy = y * y;
        const auto zz = z * z;
        const auto sum = xx + yy + zz;
        const Acceleration size{ au::sqrt(sum) };
        return size;
    }

    /// @brief Adds two accelerations, component by component.
    /// @param a the first acceleration
    /// @param b the second acceleration
    /// @return the sum
    friend AccelerationVector operator+(const AccelerationVector& a, const AccelerationVector& b) {
        const AccelerationVector sum{ a.x + b.x, a.y + b.y, a.z + b.z };
        return sum;
    }
    /// @brief Subtracts two accelerations, component by component.
    /// @param a the acceleration to subtract from
    /// @param b the acceleration to subtract
    /// @return a - b
    friend AccelerationVector operator-(const AccelerationVector& a, const AccelerationVector& b) {
        const AccelerationVector difference{ a.x - b.x, a.y - b.y, a.z - b.z };
        return difference;
    }
    /// @brief Negates each component.
    /// @param a the acceleration
    /// @return the acceleration in the opposite direction
    friend AccelerationVector operator-(const AccelerationVector& a) {
        const AccelerationVector opposite{ -a.x, -a.y, -a.z };
        return opposite;
    }
    /// @brief Scales each component by a plain number.
    /// @param a the acceleration
    /// @param factor the plain number that multiplies each component
    /// @return the scaled acceleration
    friend AccelerationVector operator*(const AccelerationVector& a, double factor) {
        const AccelerationVector scaled{ a.x * factor, a.y * factor, a.z * factor };
        return scaled;
    }
    /// @brief Scales each component by a plain number (the factor first).
    /// @param factor the plain number that multiplies each component
    /// @param a the acceleration
    /// @return the scaled acceleration
    friend AccelerationVector operator*(double factor, const AccelerationVector& a) {
        const AccelerationVector scaled{ factor * a.x, factor * a.y, factor * a.z };
        return scaled;
    }
    /// @brief Divides each component by a plain number.
    /// @param a the acceleration
    /// @param divisor the divisor, not 0
    /// @return the divided acceleration
    friend AccelerationVector operator/(const AccelerationVector& a, double divisor) {
        const AccelerationVector divided{ a.x / divisor, a.y / divisor, a.z / divisor };
        return divided;
    }
    /// @brief Compares two accelerations component by component (exactly).
    /// @return true when all three components are equal
    friend bool operator==(const AccelerationVector&, const AccelerationVector&) = default;
};

/// @brief A force along x, y and z, in N per component.
struct ForceVector {
    Force x{ au::ZERO }; ///< the force along x
    Force y{ au::ZERO }; ///< the force along y
    Force z{ au::ZERO }; ///< the force along z, upward

    /// @brief Gives the force's size: the square root of the sum of the squared components.
    /// @return the size
    Force magnitude() const {
        const auto xx = x * x;
        const auto yy = y * y;
        const auto zz = z * z;
        const auto sum = xx + yy + zz;
        const Force size{ au::sqrt(sum) };
        return size;
    }

    /// @brief Adds two forces, component by component.
    /// @param a the first force
    /// @param b the second force
    /// @return the sum
    friend ForceVector operator+(const ForceVector& a, const ForceVector& b) {
        const ForceVector sum{ a.x + b.x, a.y + b.y, a.z + b.z };
        return sum;
    }
    /// @brief Subtracts two forces, component by component.
    /// @param a the force to subtract from
    /// @param b the force to subtract
    /// @return a - b
    friend ForceVector operator-(const ForceVector& a, const ForceVector& b) {
        const ForceVector difference{ a.x - b.x, a.y - b.y, a.z - b.z };
        return difference;
    }
    /// @brief Negates each component.
    /// @param f the force
    /// @return the force in the opposite direction
    friend ForceVector operator-(const ForceVector& f) {
        const ForceVector opposite{ -f.x, -f.y, -f.z };
        return opposite;
    }
    /// @brief Scales each component by a plain number.
    /// @param f the force
    /// @param factor the plain number that multiplies each component
    /// @return the scaled force
    friend ForceVector operator*(const ForceVector& f, double factor) {
        const ForceVector scaled{ f.x * factor, f.y * factor, f.z * factor };
        return scaled;
    }
    /// @brief Scales each component by a plain number (the factor first).
    /// @param factor the plain number that multiplies each component
    /// @param f the force
    /// @return the scaled force
    friend ForceVector operator*(double factor, const ForceVector& f) {
        const ForceVector scaled{ factor * f.x, factor * f.y, factor * f.z };
        return scaled;
    }
    /// @brief Divides each component by a plain number.
    /// @param f the force
    /// @param divisor the divisor, not 0
    /// @return the divided force
    friend ForceVector operator/(const ForceVector& f, double divisor) {
        const ForceVector divided{ f.x / divisor, f.y / divisor, f.z / divisor };
        return divided;
    }
    /// @brief Compares two forces component by component (exactly).
    /// @return true when all three components are equal
    friend bool operator==(const ForceVector&, const ForceVector&) = default;
};

/// @brief A direction or an axis without a unit (a plane's normal, a rotation axis); its length is not 1 in general.
struct Direction {
    double x{}; ///< the component along x
    double y{}; ///< the component along y
    double z{}; ///< the component along z

    /// @brief Gives the vector's length: the square root of the sum of the squared components.
    /// @return the length (a plain number)
    double magnitude() const {
        const double xx{ x * x };
        const double yy{ y * y };
        const double zz{ z * z };
        const double sum{ xx + yy + zz };
        const double length{ std::sqrt(sum) };
        return length;
    }

    /// @brief Adds two directions, component by component.
    /// @param a the first direction
    /// @param b the second direction
    /// @return the sum
    friend Direction operator+(const Direction& a, const Direction& b) {
        const Direction sum{ a.x + b.x, a.y + b.y, a.z + b.z };
        return sum;
    }
    /// @brief Subtracts two directions, component by component.
    /// @param a the direction to subtract from
    /// @param b the direction to subtract
    /// @return a - b
    friend Direction operator-(const Direction& a, const Direction& b) {
        const Direction difference{ a.x - b.x, a.y - b.y, a.z - b.z };
        return difference;
    }
    /// @brief Negates each component.
    /// @param d the direction
    /// @return the opposite direction
    friend Direction operator-(const Direction& d) {
        const Direction opposite{ -d.x, -d.y, -d.z };
        return opposite;
    }
    /// @brief Scales each component by a plain number.
    /// @param d the direction
    /// @param factor the plain number that multiplies each component
    /// @return the scaled direction
    friend Direction operator*(const Direction& d, double factor) {
        const Direction scaled{ d.x * factor, d.y * factor, d.z * factor };
        return scaled;
    }
    /// @brief Scales each component by a plain number (the factor first).
    /// @param factor the plain number that multiplies each component
    /// @param d the direction
    /// @return the scaled direction
    friend Direction operator*(double factor, const Direction& d) {
        const Direction scaled{ factor * d.x, factor * d.y, factor * d.z };
        return scaled;
    }
    /// @brief Divides each component by a plain number, for example by its magnitude() to make a unit vector.
    /// @param d the direction
    /// @param divisor the divisor, not 0
    /// @return the divided direction
    friend Direction operator/(const Direction& d, double divisor) {
        const Direction divided{ d.x / divisor, d.y / divisor, d.z / divisor };
        return divided;
    }
    /// @brief Compares two directions component by component (exactly).
    /// @return true when all three components are equal
    friend bool operator==(const Direction&, const Direction&) = default;
};

/// @brief Gives the position that a velocity reaches in a duration: each component times the duration.
/// @param u the velocity
/// @param t the duration
/// @return the displacement
inline Position operator*(const Velocity& u, Duration t) {
    const Position moved{ u.x * t, u.y * t, u.z * t };
    return moved;
}

/// @brief Gives the velocity that covers a displacement in a duration: each component over the duration.
/// @param p the displacement
/// @param t the duration, not 0
/// @return the velocity
inline Velocity operator/(const Position& p, Duration t) {
    const Velocity velocity{ p.x / t, p.y / t, p.z / t };
    return velocity;
}

/// @brief Gives the acceleration that changes a velocity in a duration: each component over the duration.
/// @param u the change of velocity
/// @param t the duration, not 0
/// @return the acceleration
inline AccelerationVector operator/(const Velocity& u, Duration t) {
    const AccelerationVector acceleration{ u.x / t, u.y / t, u.z / t };
    return acceleration;
}

} // namespace si
