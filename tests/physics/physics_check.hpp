#pragma once

#include <cstdio>

/// @brief The verdict of a physics test, for CTest: the line "PHYSICS <check>: ... PASSED" or "... FAILED"
/// (tests/physics/CMakeLists.txt matches it).
class PhysicsCheck {
public:
    /// Reports a relative error against its limit (both as fractions, printed in %); true if it passed.
    static bool report(const char* check, double error, double limit) {
        const bool passed = error < limit; // false for NaN
        std::printf("\nPHYSICS %s: error %.3f %% (limit %.3g %%) %s\n", check, 100.0 * error, 100.0 * limit, passed ? "PASSED" : "FAILED");
        std::fflush(stdout);
        return passed;
    }
};
