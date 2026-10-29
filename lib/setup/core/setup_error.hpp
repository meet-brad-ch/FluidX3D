#pragma once

#include <stdexcept>

/// @brief The error of a set-up that cannot be simulated: a missing file, a value out of its range, a grid larger
/// than the memory limit, and so on.
///
/// The library throws it with a message that names the fault. Simulation, ConvectiveBoundaryLayer and the graphics
/// and moving-part components catch it at their entry points and end the program with print_error(). Simulation and
/// ConvectiveBoundaryLayer check their descriptions before the LBM is created; the graphics (a camera's pose) and
/// the moving parts (the model's placement) catch it later, when the LBM exists. Code that calls the library's value
/// classes directly gets the exception and must handle it.
class SetupError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};
