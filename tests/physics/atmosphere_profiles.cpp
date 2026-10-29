// The atmosphere API (ConvectiveBoundaryLayer over AtmosphereSimulation) runs the verified CBL example's cases to their end: the interval pipeline (the fields,
// the layer means, the inversion height, the observers, the nudging, the checks) gives the example's profiles.csv byte
// for byte (but the line ends) on the golden cases quick, quick surface and sp2011 (tests/golden/cbl), written by an observer in the
// example's format (ProfileCsv). With FLUIDX3D_EXTRA_CBL, FLUIDX3D_EXTRA_CBL_WIND and FLUIDX3D_EXTRA_SOUNDING set, the two
// moist cases too (the cloud columns, the RESULT CLOUD line).
// Passes when every case's profiles.csv is its capture's.

#include "atmosphere_cases.hpp"
#include "setup/atmosphere/profile_csv.hpp"
#include "physics_check.hpp"

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace {

/// @brief Runs each golden case through the API and compares its profiles.csv and RESULT CLOUD line with the capture.
class ProfileComparison {
public:
    /// @brief Runs one case and prints its verdict line.
    /// @param c the case
    /// @param index the case's place in the list, for its file name
    /// @return true when the profiles.csv and the RESULT CLOUD line are the capture's
    /// @throws SetupError or std::runtime_error from the run or from reading the capture
    static bool same_as_capture(const AtmosphereCase& c, std::size_t index) {
        const std::string file_name = "fluidx3d_atmosphere_profiles_" + std::to_string(index) + ".csv";
        const std::filesystem::path path = std::filesystem::temp_directory_path() / file_name;
        const RunSummary summary = run_case(path, c.description);
        const std::string run_text = text_of(path);
        const std::string captured_text = text_of(c.capture.path("profiles.csv"));
        const bool same_profiles{ run_text == captured_text };
        const std::string cloud_line = cloud_line_of(summary);
        const std::string captured_cloud = result_cloud(c.capture);
        const bool same_cloud{ cloud_line == captured_cloud };
        const char* profiles_verdict{ same_profiles ? "byte-identical" : "DIFFERENT" };
        std::string cloud_verdict = "the same";
        if(!same_cloud) {
            cloud_verdict = "DIFFERENT: '" + cloud_line + "'";
        }
        std::printf("\n%s: %u intervals (%u recorded), profiles.csv %s, RESULT CLOUD %s\n", c.name.c_str(), summary.intervals, summary.recorded, profiles_verdict,
                    cloud_verdict.c_str());
        keep_or_remove(path, same_profiles, c.name);
        const bool same{ same_profiles && same_cloud };
        return same;
    }

private:
    /// @brief Reads a text file without its carriage returns.
    /// @param path a text file
    /// @return its bytes without carriage returns: the example writes in text mode (CRLF on Windows, LF elsewhere), and
    /// git checks the golden files out with the platform's line ends
    static std::string text_of(const std::filesystem::path& path) {
        const std::ifstream in(path, std::ios::binary);
        std::stringstream s;
        s << in.rdbuf();
        std::string t = s.str();
        const std::string::iterator kept_end = std::remove(t.begin(), t.end(), '\r');
        t.erase(kept_end, t.end());
        return t;
    }

    /// @brief Gives a capture's RESULT CLOUD line.
    /// @param g a capture
    /// @return its RESULT CLOUD line, or empty
    static std::string result_cloud(const GoldenCapture& g) {
        const std::string file = g.path("run_lines.txt");
        std::ifstream in(file);
        for(std::string line; std::getline(in, line);) {
            if(!line.empty() && line.back() == '\r') {
                line.pop_back();
            }
            if(line.rfind("RESULT CLOUD", 0u) == 0u) {
                return line;
            }
        }
        return "";
    }

    /// @brief Gives a run's RESULT CLOUD line.
    /// @param summary a run's summary
    /// @return the last of its check lines that starts with RESULT CLOUD, or empty
    static std::string cloud_line_of(const RunSummary& summary) {
        std::string cloud_line{};
        for(const CheckResult& r : summary.results) {
            if(r.line.rfind("RESULT CLOUD", 0u) == 0u) {
                cloud_line = r.line;
            }
        }
        return cloud_line;
    }

    /// @brief Runs a case with a ProfileCsv observer.
    /// @param path the profiles.csv to write
    /// @param d the case
    /// @return the summary of the case's run (the run and its profiles.csv closed at the return)
    static RunSummary run_case(const std::filesystem::path& path, const CblDescription& d) {
        ConvectiveBoundaryLayer run(d);
        const bool moist = d.moist();
        ProfileCsv csv(path, moist);
        run.observe(csv);
        RunSummary summary = run.run();
        return summary;
    }

    /// @brief Removes a run's profiles.csv that equals its capture, or keeps it for the diff and prints where it is.
    /// @param path the run's profiles.csv
    /// @param same true when it equals the capture's
    /// @param name the case's name, for the message
    static void keep_or_remove(const std::filesystem::path& path, bool same, const std::string& name) {
        if(same) {
            std::filesystem::remove(path);
            return;
        }
        const std::string kept = path.string();
        std::printf("%s: the run's profiles.csv is kept for the diff: %s\n", name.c_str(), kept.c_str());
    }
};

} // namespace

void main_setup() { // extensions: VOLUME_FORCE TEMPERATURE SUBGRID THERMAL_DIFFUSIVITY_FIX LEVEL_FORCING MOISTURE SCALAR_FV
    const std::vector<AtmosphereCase> cases = AtmosphereCases::all();
    double failed{ 0.0 };
    const std::size_t case_count{ cases.size() };
    for(std::size_t index{ 0u }; index < case_count; ++index) {
        const bool same{ ProfileComparison::same_as_capture(cases[index], index) };
        if(!same) {
            failed += 1.0;
        }
    }
    const std::string count = std::to_string(case_count);
    const std::string check = "atmosphere profiles: the API's runs give the verified example's profiles (" + count + " cases)";
    const double limit{ 0.5 }; // less than one failed case
    PhysicsCheck::report(check.c_str(), failed, limit);
}
