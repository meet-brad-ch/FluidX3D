#pragma once

#include <cstdlib>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

/// @brief A frozen capture of the convective boundary layer example (tests/golden/cbl/<case>): its settings and per-level values, read back exactly from their hex floats. The unit
/// tests compare the API's values against them: where the API keeps the example's float order, bit for bit.
class GoldenCapture {
public:
    /// @param name the case's folder under tests/golden/cbl (quick, quick_surface, sp2011)
    /// @throws std::runtime_error when a file is missing or a line cannot be read
    explicit GoldenCapture(const std::string& name) : GoldenCapture(Folder{ .path = golden_folder(name) }) {}

    /// @brief A capture outside the repository (an extra case, named by an environment variable).
    /// @param folder the capture's folder
    /// @return the capture
    /// @throws std::runtime_error when a file is missing or a line cannot be read
    static GoldenCapture at(const std::string& folder) {
        const Folder full = Folder{ .path = folder };
        return GoldenCapture(full);
    }

    /// @brief An environment variable that names an extra case (MSVC deprecates std::getenv, so _dupenv_s there).
    /// @param name the variable
    /// @return its value, or none when it is not set
    static std::optional<std::string> environment(const char* name) {
#ifdef _MSC_VER
        char* value = nullptr;
        std::size_t length = 0u;
        const int error = _dupenv_s(&value, &length, name);
        if(error != 0 || value == nullptr) {
            return std::nullopt;
        }
        const std::unique_ptr<char, decltype(&std::free)> owned(value, &std::free);
        const std::string text = std::string(owned.get());
        return text;
#else
        const char* value = std::getenv(name);
        if(value == nullptr) {
            return std::nullopt;
        }
        const std::string text = std::string(value);
        return text;
#endif
    }

private:
    /// @brief The full path of a capture's folder (keeps the two constructors apart).
    struct Folder {
        std::string path{}; ///< the folder
    };

    /// @param name the case's folder under tests/golden/cbl
    /// @return the folder's full path
    static std::string golden_folder(const std::string& name) {
        const std::string root = std::string(FLUIDX3D_GOLDEN_DIR);
        const std::string folder = root + "/cbl/" + name;
        return folder;
    }

    explicit GoldenCapture(const Folder& folder) : folder_(folder.path) {
        read_settings();
        read_levels();
    }

    /// @throws std::runtime_error for a missing settings.txt or a line without a key and a value
    void read_settings() {
        const std::string file = folder_ + "/settings.txt";
        std::ifstream settings(file);
        if(!settings) {
            throw std::runtime_error("GoldenCapture: no " + folder_ + "/settings.txt");
        }
        for(std::string line; std::getline(settings, line);) {
            std::istringstream in(line);
            std::string key{};
            std::string value{};
            if(!(in >> key >> value)) {
                throw std::runtime_error("GoldenCapture: settings line '" + line + "'");
            }
            const double number = std::strtod(value.c_str(), nullptr); // hex floats and integers alike
            values_[key] = number;
        }
    }

    /// @throws std::runtime_error for a missing levels.csv or a row whose length is not the header's
    void read_levels() {
        const std::string file = folder_ + "/levels.csv";
        std::ifstream levels(file);
        if(!levels) {
            throw std::runtime_error("GoldenCapture: no " + folder_ + "/levels.csv");
        }
        std::string header{};
        std::getline(levels, header);
        columns_ = split(header);
        for(std::string line; std::getline(levels, line);) {
            if(line.empty() || line == "\r") {
                continue;
            }
            const std::vector<double> row = numbers_of(line);
            if(row.size() != columns_.size()) {
                const std::string count = std::to_string(row.size());
                throw std::runtime_error("GoldenCapture: levels.csv row of " + count + " values");
            }
            rows_.push_back(row);
        }
    }

    /// @param line a levels.csv row
    /// @return its values, read back exactly
    static std::vector<double> numbers_of(const std::string& line) {
        std::vector<double> row{};
        const std::vector<std::string> cells = split(line);
        for(const std::string& cell : cells) {
            const double number = std::strtod(cell.c_str(), nullptr);
            row.push_back(number);
        }
        return row;
    }

public:
    /// @param key a settings name (dx, dt, heat_rate, ...)
    /// @return its value, exact for a float
    double setting(const std::string& key) const {
        const std::map<std::string, double>::const_iterator it = values_.find(key);
        if(it == values_.end()) {
            throw std::runtime_error("GoldenCapture: no setting " + key);
        }
        const double value = it->second;
        return value;
    }

    /// @return the number of levels
    std::size_t levels() const { return rows_.size(); }

    /// @param z a level
    /// @param column a levels.csv column (height_m, start_theta_K, T_ref, sponge_rate, ...)
    /// @return its value, exact
    double level(std::size_t z, const std::string& column) const {
        for(std::size_t c = 0u; c < columns_.size(); c++) {
            if(columns_[c] == column) {
                const double value = rows_.at(z).at(c);
                return value;
            }
        }
        throw std::runtime_error("GoldenCapture: no column " + column);
    }

    /// @param file a file of the case
    /// @return its full path
    std::string path(const std::string& file) const {
        const std::string full = folder_ + "/" + file;
        return full;
    }

private:
    /// @param line a CSV line (a trailing carriage return is removed)
    /// @return its cells
    static std::vector<std::string> split(std::string line) {
        if(!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        std::vector<std::string> cells{};
        std::stringstream in(line);
        for(std::string cell; std::getline(in, cell, ',');) {
            cells.push_back(cell);
        }
        return cells;
    }

    std::string folder_{};                     ///< the capture's folder
    std::map<std::string, double> values_{};   ///< the settings by name
    std::vector<std::string> columns_{};       ///< the levels.csv header
    std::vector<std::vector<double>> rows_{};  ///< the levels.csv rows
};
