#include "setup/atmosphere/sounding.hpp"
#include "setup/core/setup_error.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <fstream>
#include <istream>
#include <string>
#include <vector>

namespace {

/// @brief Gives the idealized profile's theta above theta_0 at a height (the example's start_theta()): 0 in the
/// mixed layer, rising linearly over the jump's depth, then the jump plus the lapse rate times the height above it.
/// @param p the idealized profile
/// @param h a height above the floor's top
/// @return theta_l(h) - theta_0
si::TemperatureDifference idealized_excess(const IdealizedProfile& p, si::Length h) {
    const si::Length jump_top{ p.inversion_base + p.jump_depth };
    si::TemperatureDifference excess{ au::kelvins(0.0) };
    if(h < p.inversion_base) {
        excess = au::kelvins(0.0);
    } else if(h < jump_top) {
        const si::Length into_jump{ h - p.inversion_base };
        const auto ratio = into_jump / p.jump_depth;
        const double fraction{ au::as_raw_number(ratio) };
        excess = p.jump * fraction;
    } else {
        const si::Length above_base{ h - p.inversion_base };
        const si::Length above_jump{ above_base - p.jump_depth };
        const auto rise = p.lapse * above_jump;
        excess = p.jump + rise;
    }
    return excess;
}

} // namespace

Sounding Sounding::idealized(si::Temperature reference, IdealizedProfile profile) {
    const double reference_k{ si::in_kelvin(reference) };
    if(!(reference_k > 0.0)) {
        throw SetupError("Sounding::idealized(): theta_0 must be more than 0 K");
    }
    const bool base_negative{ profile.inversion_base < au::ZERO };
    const bool depth_negative{ profile.jump_depth < au::ZERO };
    if(base_negative || depth_negative) {
        throw SetupError("Sounding::idealized(): the inversion base and the jump depth must be at least 0");
    }
    Sounding sounding;
    sounding.idealized_ = profile;
    sounding.reference_ = reference;
    return sounding;
}

Sounding Sounding::tabulated(TemperatureProfile theta_l, std::optional<WaterContentProfile> total_water,
                             std::optional<si::Temperature> reference) {
    if(reference) {
        const double reference_k{ si::in_kelvin(*reference) };
        if(!(reference_k > 0.0)) {
            throw SetupError("Sounding::tabulated(): theta_0 must be more than 0 K");
        }
    }
    Sounding sounding;
    sounding.theta_l_ = std::move(theta_l);
    sounding.total_water_ = std::move(total_water);
    sounding.reference_ = reference;
    return sounding;
}

bool Sounding::csv_header_has_qt(const std::string& header, const std::filesystem::path& path) {
    const std::size_t last{ header.find_last_not_of(" \t\r") };
    std::string names{};
    if(last != std::string::npos) {
        names = header.substr(0u, last + 1u);
    }
    const bool dry{ names == "z_m,thl_K" };
    const bool moist{ names == "z_m,thl_K,qt_kgkg" };
    if(!dry && !moist) {
        const std::string message{ "Sounding::from_csv(): " + path.string() + " has the header '" + names + "', not z_m,thl_K or z_m,thl_K,qt_kgkg" };
        throw SetupError(message);
    }
    return moist;
}

double Sounding::parse_csv_number(const std::string& text, const std::string& where) {
    std::size_t used{ 0u };
    const double value{ std::stod(text, &used) };
    const std::string rest{ text.substr(used) };
    const bool only_space{ rest.find_first_not_of(" \t\r") == std::string::npos };
    if(!only_space) {
        const std::string message{ where + ": text after a number: " + text };
        throw SetupError(message);
    }
    return value;
}

Sounding::CsvRow Sounding::parse_csv_row(const std::string& line, std::size_t comma, bool with_qt, const std::string& where) {
    const std::string z_text{ line.substr(0u, comma) };
    const double z_m{ parse_csv_number(z_text, where) };
    const si::Length z{ au::meters(z_m) };
    std::size_t second_comma{ std::string::npos };
    std::size_t theta_length{ std::string::npos }; // a dry row: theta_l is the rest of the line
    if(with_qt) {
        second_comma = line.find(',', comma + 1u);
        if(second_comma == std::string::npos) {
            const std::string message{ where + ": a row without its qt_kgkg value" };
            throw SetupError(message);
        }
        theta_length = second_comma - comma - 1u;
    }
    const std::string theta_text{ line.substr(comma + 1u, theta_length) };
    const double theta_k{ parse_csv_number(theta_text, where) };
    const si::Temperature theta{ si::kelvin_point(theta_k) };
    std::optional<si::WaterContent> water{};
    if(with_qt) {
        const std::string qt_text{ line.substr(second_comma + 1u) };
        const double qt{ parse_csv_number(qt_text, where) };
        water = (si::kilograms_of_water / si::kilograms)(qt);
    }
    const CsvRow row{ .height = z, .theta_l = theta, .total_water = water };
    return row;
}

void Sounding::add_csv_row(CsvColumns& columns, const CsvRow& row) {
    columns.theta.push_back({ .height = row.height, .value = row.theta_l });
    if(row.total_water.has_value()) {
        columns.water.push_back({ .height = row.height, .value = *row.total_water });
    }
}

Sounding::CsvColumns Sounding::read_csv_rows(std::istream& in, const std::filesystem::path& path, bool with_qt) {
    CsvColumns columns{};
    std::string line{};
    for(std::size_t row = 2u; std::getline(in, line); row++) {
        const std::size_t c{ line.find(',') };
        if(c == std::string::npos) {
            continue; // as the example: a line without a comma is not a row
        }
        const std::string row_text{ std::to_string(row) };
        std::string where{ path.string() };
        where += " line ";
        where += row_text;
        try {
            const CsvRow parsed{ parse_csv_row(line, c, with_qt, where) };
            add_csv_row(columns, parsed);
        } catch(const std::logic_error&) {
            std::string message{ where };
            message += ": not a number: ";
            message += line;
            throw SetupError(message);
        }
    }
    return columns;
}

Sounding Sounding::from_csv(const std::filesystem::path& path) {
    std::ifstream in(path);
    if(!in) {
        const std::string message{ "Sounding::from_csv(): no file " + path.string() };
        throw SetupError(message);
    }
    std::string header{};
    std::getline(in, header); // the header z_m,thl_K[,qt_kgkg]: the file's columns and units
    const bool with_qt{ csv_header_has_qt(header, path) };
    const CsvColumns columns{ read_csv_rows(in, path, with_qt) };
    if(columns.theta.size() < 2u) {
        const std::string message{ "Sounding::from_csv(): " + path.string() + " holds fewer than 2 levels" };
        throw SetupError(message);
    }
    std::optional<WaterContentProfile> qt{};
    if(with_qt) {
        qt.emplace(WaterContentProfile::from_points(columns.water));
    }
    TemperatureProfile theta_l{ TemperatureProfile::from_points(columns.theta) };
    Sounding sounding{ tabulated(std::move(theta_l), std::move(qt), std::nullopt) };
    return sounding;
}

Sounding Sounding::with_reference_from_mixed_layer(si::Length mixed_layer_top) const {
    if(!theta_l_) {
        throw SetupError("Sounding::with_reference_from_mixed_layer(): an idealized sounding has its theta_0");
    }
    const si::Length window_bottom{ mixed_layer_top * reference_window_bottom };
    const si::Length window_top{ mixed_layer_top * reference_window_top };
    std::vector<si::Temperature> mixed{};
    for(const TemperaturePoint& p : theta_l_->points()) {
        const bool above_bottom{ p.height > window_bottom };
        const bool below_top{ p.height < window_top };
        if(above_bottom && below_top) {
            mixed.push_back(p.value);
        }
    }
    if(mixed.empty()) {
        throw SetupError("Sounding::with_reference_from_mixed_layer(): no level between 0.2 and 0.8 zi");
    }
    std::sort(mixed.begin(), mixed.end());
    const std::size_t middle{ mixed.size() / 2u };
    Sounding sounding = *this;
    sounding.reference_ = mixed[middle]; // the upper middle value for an even count (the example)
    return sounding;
}

si::Temperature Sounding::reference() const {
    if(!reference_) {
        throw SetupError("Sounding: no theta_0 yet (with_reference_from_mixed_layer())");
    }
    const si::Temperature theta_0{ *reference_ };
    return theta_0;
}

void Sounding::require_reference(si::Temperature theta_0) const {
    const si::Temperature own{ reference() };
    if(theta_0 != own) {
        const std::string given_text{ std::to_string(si::in_kelvin(theta_0)) };
        const std::string own_text{ std::to_string(si::in_kelvin(own)) };
        throw SetupError("Sounding: the lattice temperature 1 is " + given_text + " K, the sounding's theta_0 is " + own_text + " K: they must be the same");
    }
}

si::TemperatureDifference Sounding::theta_excess(si::Length h) const {
    if(idealized_.has_value()) {
        const si::TemperatureDifference ideal{ idealized_excess(*idealized_, h) }; // the example's start_theta()
        return ideal;
    }
    if(!theta_l_.has_value()) {
        throw SetupError("Sounding::theta_excess(): neither idealized nor tabulated");
    }
    const si::Temperature theta_l{ theta_l_->at(h) };
    const si::Temperature theta_0{ reference() };
    const si::TemperatureDifference excess{ theta_l - theta_0 }; // the example's profile_at(): a + f (b - a) - theta_0
    return excess;
}

si::WaterContent Sounding::total_water(si::Length h) const {
    if(!total_water_.has_value()) {
        throw SetupError("Sounding::total_water(): a dry sounding");
    }
    const si::WaterContent qt{ total_water_->at(h) };
    return qt;
}

std::string Sounding::describe_idealized() const {
    if(!idealized_.has_value()) {
        throw SetupError("Sounding::describe_idealized(): not an idealized sounding");
    }
    const IdealizedProfile& ideal = *idealized_;
    const double theta_0_k{ si::in_kelvin(reference()) };
    const double base_m{ ideal.inversion_base.in(au::meters) };
    const double jump_k{ ideal.jump.in(au::kelvins) };
    const double depth_m{ ideal.jump_depth.in(au::meters) };
    const double lapse_k_m{ ideal.lapse.in(au::kelvins / au::meter) };
    constexpr std::size_t text_capacity{ 200u };
    char text[text_capacity]{};
    const int length = std::snprintf(text, sizeof(text), "idealized: theta_0 %.2f K, inversion at %.0f m, +%.2f K over %.0f m, %.4f K/m above", theta_0_k, base_m, jump_k,
                  depth_m, lapse_k_m);
    const bool cut{ length < 0 || static_cast<std::size_t>(length) >= text_capacity };
    if(cut) {
        throw SetupError("Sounding::describe_idealized(): the text does not fit its buffer");
    }
    const std::string description{ text };
    return description;
}

std::string Sounding::describe_tabulated() const {
    if(!theta_l_.has_value()) {
        throw SetupError("Sounding::describe_tabulated(): not a tabulated sounding");
    }
    const TemperatureProfile& table = *theta_l_;
    const std::size_t level_count{ table.points().size() };
    const double top_m{ table.top().in(au::meters) };
    std::string theta_0_text{ "not set" };
    if(reference_) {
        const double theta_0_k{ si::in_kelvin(*reference_) };
        const std::string kelvin_text{ std::to_string(theta_0_k) };
        theta_0_text = kelvin_text + " K";
    }
    const bool wet{ total_water_.has_value() };
    const char* const water_text{ wet ? "with total water" : "dry" };
    constexpr std::size_t text_capacity{ 200u };
    char text[text_capacity]{};
    const int length = std::snprintf(text, sizeof(text), "tabulated: %zu levels to %.0f m, theta_0 %s, %s", level_count, top_m, theta_0_text.c_str(), water_text);
    const bool cut{ length < 0 || static_cast<std::size_t>(length) >= text_capacity };
    if(cut) {
        throw SetupError("Sounding::describe_tabulated(): the text does not fit its buffer");
    }
    const std::string description{ text };
    return description;
}

std::string Sounding::describe() const {
    std::string description{};
    if(idealized_) {
        description = describe_idealized();
    } else {
        description = describe_tabulated();
    }
    return description;
}
