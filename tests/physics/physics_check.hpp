#pragma once

#include <cmath>
#include <cstdio>

/// @brief The verdict of a physics test, for CTest: the line "PHYSICS <check>: ... PASSED" or "... FAILED"
/// (tests/physics/CMakeLists.txt matches it).
class PhysicsCheck {
public:
    /// @brief Prints the verdict line of a relative error against its limit. A NaN or infinite error fails (checked
    /// with std::isnan/isinf, which /fp:fast does not fold away, unlike error < limit).
    /// @param check the name of the check, printed after "PHYSICS"
    /// @param error the relative error, a fraction (printed in %)
    /// @param limit the largest error that passes, a fraction (printed in %)
    /// @return true when the error is finite and less than the limit
    static bool report(const char* check, double error, double limit) {
        const bool finite{ !std::isnan(error) && !std::isinf(error) };
        const bool passed{ finite && error < limit };
        const double error_percent{ percent * error };
        const double limit_percent{ percent * limit };
        const char* verdict{ passed ? "PASSED" : "FAILED" };
        std::printf("\nPHYSICS %s: error %.3f %% (limit %.3g %%) %s\n", check, error_percent, limit_percent, verdict);
        std::fflush(stdout);
        return passed;
    }

private:
    static constexpr double percent = 100.0; ///< the percent of a fraction of 1, for the printed error and limit
};
