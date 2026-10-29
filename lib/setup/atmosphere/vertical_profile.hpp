#pragma once

#include "setup/core/setup_error.hpp"
#include "setup/units/units.hpp"

#include <cstddef>
#include <utility>
#include <vector>

/// @file vertical_profile.hpp
/// @brief Values tabulated against height: the table in plain numbers (ProfileTable) and the three profiles of the
/// atmosphere (TemperatureProfile, WaterContentProfile, SpeedProfile).

/// @brief One sample of a table: a height and a value in the table's fixed unit.
struct ProfileSample {
    si::Length height{ au::ZERO }; ///< the height above the floor's top
    double value{};                ///< the value at that height, in the table's unit (K, kg/kg or m/s)
};

/// @brief Values tabulated against height in one fixed unit: linear between the samples, held at the end values
/// outside them (a + f (b - a), the convective boundary layer example's profile_at() and qt_at()).
///
/// The profiles below hold their values here as plain numbers in their unit and convert only at their boundary, so a
/// profile's value at a height is the same double as an interpolation of the quantities.
class ProfileTable {
public:
    /// @brief Makes a table from its samples.
    /// @param samples at least two samples, in increasing height
    /// @return the table
    /// @throws SetupError when there are fewer than two samples, or the heights do not increase strictly
    static ProfileTable from_samples(std::vector<ProfileSample> samples) {
        if(samples.size() < 2u) {
            throw SetupError("ProfileTable: at least two points");
        }
        for(std::size_t i = 1u; i < samples.size(); i++) {
            const si::Length below{ samples[i - 1u].height };
            const si::Length above{ samples[i].height };
            if(!(above > below)) {
                throw SetupError("ProfileTable: the heights must increase");
            }
        }
        const ProfileTable table(std::move(samples));
        return table;
    }

    /// @brief Gives the value at a height.
    /// @param h a height above the floor's top
    /// @return the value at h, in the table's unit: linear between the two samples around h; the lowest sample's value
    /// at or below it, the highest sample's value above it
    double at(si::Length h) const {
        const std::size_t i{ upper_index(h) };
        double value{ samples_.back().value };
        if(i == 0u) {
            value = samples_.front().value;
        } else if(i < samples_.size()) {
            value = interpolate(samples_[i - 1u], samples_[i], h);
        }
        return value;
    }

    /// @return the lowest sample's height
    si::Length bottom() const { return samples_.front().height; }
    /// @return the highest sample's height
    si::Length top() const { return samples_.back().height; }
    /// @return the samples, in increasing height
    const std::vector<ProfileSample>& samples() const { return samples_; }

private:
    /// @brief Makes the table from checked samples. Use from_samples(), which checks them.
    /// @param samples at least two samples, in increasing height
    explicit ProfileTable(std::vector<ProfileSample> samples) : samples_(std::move(samples)) {}

    /// @brief Finds the first sample at or above a height.
    /// @param h a height above the floor's top
    /// @return the index of the first sample at or above h, counted from 0; the number of samples when h is above all
    std::size_t upper_index(si::Length h) const {
        std::size_t i{ 0u };
        while(i < samples_.size() && !(h <= samples_[i].height)) {
            i++;
        }
        return i;
    }

    /// @brief Interpolates linearly between two samples: a + f (b - a), with f the part of the height from a to b.
    /// @param a the sample below h
    /// @param b the sample at or above h, higher than a
    /// @param h a height between the two samples
    /// @return the value at h, in the table's unit
    static double interpolate(const ProfileSample& a, const ProfileSample& b, si::Length h) {
        const si::Length rise{ h - a.height };
        const si::Length span{ b.height - a.height };
        const auto ratio = rise / span;
        const double f{ au::as_raw_number(ratio) };
        const double change{ b.value - a.value };
        const double part{ change * f };
        const double value{ a.value + part };
        return value;
    }

    std::vector<ProfileSample> samples_{}; ///< the samples, in increasing height
};

/// @brief One point of a temperature profile: a height and the temperature reading there.
struct TemperaturePoint {
    si::Length height{ au::ZERO };      ///< the height above the floor's top
    si::Temperature value{};            ///< the temperature reading at that height
};

/// @brief A temperature reading (for example theta_l) tabulated against height, linear between the points and held
/// outside them. The values are held in K.
/// @code
/// using namespace si::symbols;
/// const TemperatureProfile thl = TemperatureProfile::from_points({ { 0.0 * m, si::kelvin_point(300.0) }, { 100.0 * m, si::kelvin_point(302.0) } });
/// const si::Temperature at_50 = thl.at(50.0 * m); // 301 K
/// @endcode
class TemperatureProfile {
public:
    /// @brief Makes a profile from its points.
    /// @param points at least two points, in increasing height
    /// @return the profile
    /// @throws SetupError when there are fewer than two points, or the heights do not increase strictly
    static TemperatureProfile from_points(const std::vector<TemperaturePoint>& points) {
        std::vector<ProfileSample> samples{};
        for(const TemperaturePoint& p : points) {
            const double kelvin{ si::in_kelvin(p.value) };
            const ProfileSample sample{ .height = p.height, .value = kelvin };
            samples.push_back(sample);
        }
        const TemperatureProfile profile(ProfileTable::from_samples(std::move(samples)));
        return profile;
    }

    /// @brief Gives the temperature at a height.
    /// @param h a height above the floor's top
    /// @return the reading at h: linear between the points, the end values outside them
    si::Temperature at(si::Length h) const {
        const double kelvin{ table_.at(h) };
        const si::Temperature reading{ si::kelvin_point(kelvin) };
        return reading;
    }

    /// @return the lowest point's height
    si::Length bottom() const { return table_.bottom(); }
    /// @return the highest point's height
    si::Length top() const { return table_.top(); }
    /// @return the number of points, at least 2
    std::size_t size() const { return table_.samples().size(); }

    /// @brief Gives the points, in increasing height.
    /// @return a copy of the points
    std::vector<TemperaturePoint> points() const {
        std::vector<TemperaturePoint> points{};
        for(const ProfileSample& sample : table_.samples()) {
            const TemperaturePoint p{ .height = sample.height, .value = si::kelvin_point(sample.value) };
            points.push_back(p);
        }
        return points;
    }

private:
    /// @brief Makes the profile from its table in K.
    /// @param table the checked table
    explicit TemperatureProfile(ProfileTable table) : table_(std::move(table)) {}

    ProfileTable table_; ///< the readings in K against height
};

/// @brief One point of a water-content profile: a height and the total water there.
struct WaterContentPoint {
    si::Length height{ au::ZERO };              ///< the height above the floor's top
    si::WaterContent value{ au::ZERO };         ///< the water content at that height
};

/// @brief A water content (for example qt) tabulated against height, linear between the points and held outside them.
/// The values are held in kg of water per kg of air.
class WaterContentProfile {
public:
    /// @brief Makes a profile from its points.
    /// @param points at least two points, in increasing height
    /// @return the profile
    /// @throws SetupError when there are fewer than two points, or the heights do not increase strictly
    static WaterContentProfile from_points(const std::vector<WaterContentPoint>& points) {
        std::vector<ProfileSample> samples{};
        for(const WaterContentPoint& p : points) {
            const double kg_per_kg{ p.value.in(si::kilograms_of_water / si::kilograms) };
            const ProfileSample sample{ .height = p.height, .value = kg_per_kg };
            samples.push_back(sample);
        }
        const WaterContentProfile profile(ProfileTable::from_samples(std::move(samples)));
        return profile;
    }

    /// @brief Gives the water content at a height.
    /// @param h a height above the floor's top
    /// @return the water content at h: linear between the points, the end values outside them
    si::WaterContent at(si::Length h) const {
        const double kg_per_kg{ table_.at(h) };
        const si::WaterContent water{ (si::kilograms_of_water / si::kilograms)(kg_per_kg) };
        return water;
    }

    /// @return the lowest point's height
    si::Length bottom() const { return table_.bottom(); }
    /// @return the highest point's height
    si::Length top() const { return table_.top(); }
    /// @return the number of points, at least 2
    std::size_t size() const { return table_.samples().size(); }

private:
    /// @brief Makes the profile from its table in kg/kg.
    /// @param table the checked table
    explicit WaterContentProfile(ProfileTable table) : table_(std::move(table)) {}

    ProfileTable table_; ///< the water contents in kg/kg against height
};

/// @brief One point of a speed profile: a height and the speed there.
struct SpeedPoint {
    si::Length height{ au::ZERO };      ///< the height above the floor's top
    si::Speed value{ au::ZERO };        ///< the speed at that height
};

/// @brief A speed (for example a wind speed) tabulated against height, linear between the points and held outside
/// them. The values are held in m/s.
/// @code
/// using namespace si::symbols;
/// const SpeedProfile wind = SpeedProfile::from_points({ { 0.0 * m, 0.0 * m / s }, { 100.0 * m, 5.0 * m / s } });
/// const si::Speed at_50 = wind.at(50.0 * m); // 2.5 m/s
/// @endcode
class SpeedProfile {
public:
    /// @brief Makes a profile from its points.
    /// @param points at least two points, in increasing height
    /// @return the profile
    /// @throws SetupError when there are fewer than two points, or the heights do not increase strictly
    static SpeedProfile from_points(const std::vector<SpeedPoint>& points) {
        std::vector<ProfileSample> samples{};
        for(const SpeedPoint& p : points) {
            const double metres_per_second{ p.value.in(au::meters / au::second) };
            const ProfileSample sample{ .height = p.height, .value = metres_per_second };
            samples.push_back(sample);
        }
        const SpeedProfile profile(ProfileTable::from_samples(std::move(samples)));
        return profile;
    }

    /// @brief Gives the speed at a height.
    /// @param h a height above the floor's top
    /// @return the speed at h: linear between the points, the end values outside them
    si::Speed at(si::Length h) const {
        const double metres_per_second{ table_.at(h) };
        const si::Speed speed{ (au::meters / au::second)(metres_per_second) };
        return speed;
    }

    /// @return the lowest point's height
    si::Length bottom() const { return table_.bottom(); }
    /// @return the highest point's height
    si::Length top() const { return table_.top(); }
    /// @return the number of points, at least 2
    std::size_t size() const { return table_.samples().size(); }

private:
    /// @brief Makes the profile from its table in m/s.
    /// @param table the checked table
    explicit SpeedProfile(ProfileTable table) : table_(std::move(table)) {}

    ProfileTable table_; ///< the speeds in m/s against height
};
