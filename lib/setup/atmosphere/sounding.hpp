#pragma once

#include "setup/atmosphere/vertical_profile.hpp"
#include "setup/units/units.hpp"

#include <filesystem>
#include <iosfwd>
#include <optional>
#include <string>
#include <vector>

/// @file sounding.hpp
/// @brief The start state of an atmosphere's column: Sounding and IdealizedProfile.
///
/// Terms: theta_l is the liquid water potential temperature, qt the total water (vapour and liquid, kg per kg of
/// air), theta_0 the reference potential temperature that is the lattice temperature 1, and zi the inversion height.
/// Heights are above the floor's top.

/// @brief An idealized start profile above a mixed layer at theta_0: the inversion's base, the jump spread over a
/// depth (0: a step), and a lapse rate above the jump.
///
/// Sullivan and Patton 2011's dry convective boundary layer: the base at 974 m, a jump of 8 K over 100 m, and
/// 3 K/km above.
struct IdealizedProfile {
    si::Length inversion_base{ au::ZERO };      ///< the inversion's base, zi at the start, 0 or more
    si::TemperatureDifference jump{ au::ZERO }; ///< the rise of theta across the inversion
    si::Length jump_depth{ au::ZERO };          ///< the depth over which the jump rises linearly, 0 or more (0: a step)
    si::LapseRate lapse{ au::ZERO };            ///< the gradient of theta above the jump
};

/// @brief The start state of an atmosphere's column: theta_l(h) and qt(h), and theta_0.
///
/// Idealized (an IdealizedProfile above a mixed layer at theta_0, dry) or tabulated (points of theta_l and, for a
/// moist run, qt; from points or from a CSV file). theta_excess(h) = theta_l(h) - theta_0 is what the start state and
/// the sponge use. A tabulated sounding gets its theta_0 later from with_reference_from_mixed_layer().
/// @code
/// using namespace si::symbols;
/// const Sounding air = Sounding::idealized(si::kelvin_point(300.0), { 974.0 * m, 8.0 * K, 100.0 * m, 3.0 * K / km });
/// const Sounding measured = Sounding::from_csv("profile.csv").with_reference_from_mixed_layer(1864.0 * m);
/// @endcode
class Sounding {
public:
    /// @brief Makes an idealized, dry sounding.
    /// @param reference theta_0: the mixed layer's potential temperature, more than 0 K
    /// @param profile the inversion and the lapse rate above it
    /// @return the sounding, with theta_0
    /// @throws SetupError when theta_0 is not more than 0 K, or the inversion base or the jump depth is negative
    static Sounding idealized(si::Temperature reference, IdealizedProfile profile);

    /// @brief Makes a tabulated sounding from profiles.
    /// @param theta_l the liquid water potential temperature against height
    /// @param total_water the total water qt against height, or none for a dry sounding
    /// @param reference theta_0, more than 0 K; or none, to set it later with with_reference_from_mixed_layer()
    /// @return the sounding
    /// @throws SetupError when theta_0 is given and not more than 0 K
    static Sounding tabulated(TemperatureProfile theta_l, std::optional<WaterContentProfile> total_water,
                              std::optional<si::Temperature> reference);

    /// @brief Reads a tabulated sounding from a CSV file.
    ///
    /// The first line is the header, exactly z_m,thl_K or z_m,thl_K,qt_kgkg (spaces and a carriage return at its end
    /// are ignored): the height in m, theta_l in K and qt in kg/kg. Each further line with a comma is a row, with
    /// increasing heights; a line without a comma is skipped. Each value is a number with nothing after it but
    /// spaces: "300 K" and a third value in a file without qt_kgkg are refused.
    /// @param path the file
    /// @return the tabulated sounding, without theta_0 (dry when the header has no qt_kgkg)
    /// @throws SetupError that names the file (and the line) when the file does not open, the header is not one of
    /// the two, a value is not a number or has text after it, a row lacks its qt_kgkg value, or the file holds fewer
    /// than two rows; and SetupError from TemperatureProfile or WaterContentProfile when the heights do not
    /// increase
    static Sounding from_csv(const std::filesystem::path& path);

    /// @brief Gives a copy with theta_0 from the tabulated theta_l: the median of the points strictly between 0.2 and
    /// 0.8 of the mixed layer's depth (the upper middle value for an even count; the example's rule).
    /// @param mixed_layer_top the mixed layer's depth, zi
    /// @return the copy with theta_0
    /// @throws SetupError for an idealized sounding (it has its theta_0), or when no point is in the window
    Sounding with_reference_from_mixed_layer(si::Length mixed_layer_top) const;

    /// @brief Gives the reference potential temperature.
    /// @return theta_0
    /// @throws SetupError when a tabulated sounding has no theta_0 yet
    si::Temperature reference() const;

    /// @brief Checks that a lattice temperature scale's theta_0 is this sounding's theta_0: the start writes the
    /// sounding's excess over its own theta_0, so the lattice temperature 1 must mean the same temperature.
    /// @param theta_0 the temperature that the lattice temperature 1 stands for (the converter's theta_0)
    /// @throws SetupError when theta_0 differs from reference(), or a tabulated sounding has no theta_0 yet
    void require_reference(si::Temperature theta_0) const;
    /// @brief Gives theta_l above theta_0 at a height: the idealized profile, or the tabulated theta_l interpolated
    /// linearly (held at the end values outside the points).
    /// @param h a height above the floor's top
    /// @return theta_l(h) - theta_0
    /// @throws SetupError when a tabulated sounding has no theta_0 yet
    si::TemperatureDifference theta_excess(si::Length h) const;
    /// @return true when the sounding has total water (a moist sounding)
    bool has_water() const { return total_water_.has_value(); }
    /// @brief Gives the total water at a height, interpolated linearly (held at the end values outside the points).
    /// @param h a height above the floor's top
    /// @return qt(h)
    /// @throws SetupError for a dry sounding
    si::WaterContent total_water(si::Length h) const;
    /// @return true for an idealized sounding, false for a tabulated one
    bool is_idealized() const { return idealized_.has_value(); }
    /// @return the idealized profile, or none for a tabulated sounding
    const std::optional<IdealizedProfile>& idealized_profile() const { return idealized_; }
    /// @return the tabulated theta_l, or none for an idealized sounding
    const std::optional<TemperatureProfile>& tabulated_theta_l() const { return theta_l_; }
    /// @brief Describes the sounding for the run's log.
    /// @return one line: the kind, theta_0 (or "not set") and the water
    /// @throws SetupError for an idealized sounding without theta_0 (not possible through idealized())
    std::string describe() const;

private:
    /// The bottom of the window of with_reference_from_mixed_layer(), as a ratio of zi.
    static constexpr double reference_window_bottom = 0.2;
    /// The top of the window of with_reference_from_mixed_layer(), as a ratio of zi.
    static constexpr double reference_window_top = 0.8;

    /// @brief The values of one row of a sounding's CSV file.
    struct CsvRow {
        si::Length height{ au::ZERO };                    ///< the height, from z_m
        si::Temperature theta_l{ si::kelvin_point(0.0) }; ///< theta_l, from thl_K
        std::optional<si::WaterContent> total_water{};    ///< qt, from qt_kgkg, or none for a file without that column
    };

    /// @brief Makes an empty sounding. Use idealized(), tabulated() or from_csv().
    Sounding() = default;

    /// @brief Checks the header of a CSV file and tells whether the file has the qt_kgkg column.
    /// @param header the file's first line
    /// @param path the file, for the message
    /// @return true for z_m,thl_K,qt_kgkg, false for z_m,thl_K
    /// @throws SetupError that names the file and the header when it is neither (after spaces and a carriage return
    /// at its end are removed)
    static bool csv_header_has_qt(const std::string& header, const std::filesystem::path& path);

    /// @brief Reads one value of a CSV row: a number, then nothing but spaces or a carriage return.
    /// @param text the value's text
    /// @param where the file and the line number, for the message
    /// @return the number
    /// @throws SetupError that names the place and the text when text follows the number
    /// @throws std::logic_error from std::stod (std::invalid_argument, std::out_of_range) when it is not a number
    static double parse_csv_number(const std::string& text, const std::string& where);

    /// @brief Reads the values of one row of a CSV file.
    /// @param line the row's text
    /// @param comma the position of the row's first comma, counted from 0
    /// @param with_qt true when the file has the qt_kgkg column
    /// @param where the file and the line number, for the message
    /// @return the row's values
    /// @throws SetupError when the row lacks its qt_kgkg value, or a value has text after it (parse_csv_number())
    /// @throws std::logic_error from std::stod (std::invalid_argument, std::out_of_range) when a value is not a number
    static CsvRow parse_csv_row(const std::string& line, std::size_t comma, bool with_qt, const std::string& where);

    /// @brief The points of a CSV file's rows, in the file's order.
    struct CsvColumns {
        std::vector<TemperaturePoint> theta{};  ///< theta_l at each row's height
        std::vector<WaterContentPoint> water{}; ///< qt at each row's height; empty for a file without the qt_kgkg column
    };

    /// @brief Reads the rows of a CSV file after its header. A line without a comma is not a row.
    /// @param in the file, after its header line
    /// @param path the file, for the messages
    /// @param with_qt true when the file has the qt_kgkg column
    /// @return the rows' points
    /// @throws SetupError that names the file and the line when a row lacks its qt_kgkg value or a value is not a
    /// number or has text after it
    static CsvColumns read_csv_rows(std::istream& in, const std::filesystem::path& path, bool with_qt);

    /// @brief Adds the points of one row to the columns: theta_l always, qt when the row has it.
    /// @param columns the points so far
    /// @param row the row's values
    static void add_csv_row(CsvColumns& columns, const CsvRow& row);

    /// @brief Describes an idealized sounding.
    /// @return one line: theta_0, the inversion's base, the jump and its depth, and the lapse rate
    /// @throws SetupError without theta_0 (not possible through idealized()), when the sounding is not idealized, or
    /// when the text does not fit its buffer
    std::string describe_idealized() const;
    /// @brief Describes a tabulated sounding.
    /// @return one line: the number of points, the top, theta_0 (or "not set") and the water
    /// @throws SetupError when the sounding is not tabulated, or the text does not fit its buffer
    std::string describe_tabulated() const;

    std::optional<IdealizedProfile> idealized_{};                    ///< the idealized profile, or none for a tabulated sounding
    std::optional<TemperatureProfile> theta_l_{};      ///< the tabulated theta_l, or none for an idealized sounding
    std::optional<WaterContentProfile> total_water_{}; ///< the tabulated qt, or none for a dry sounding
    std::optional<si::Temperature> reference_{};                     ///< theta_0, or none until it is set
};
