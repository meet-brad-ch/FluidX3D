#pragma once

#include <stdexcept>

// A setup that cannot be simulated (missing file, grid larger than the memory limit, ...). The library throws it
// with a complete message; the simulation layer reports it (print_error) before the LBM is created.
class SetupError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};
