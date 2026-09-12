#pragma once
#include "utilities.hpp"
#include "setup/core/quantity.hpp"
#include "setup/config/simulation_config.hpp"
#include <optional>
#include <string>

// A geometry file (in resources/) and its orientation. Its STL coordinates are metres.
class Model {
public:
    explicit Model(std::string file) : file_(std::move(file)) {}

    Model& rotation(Angle x, Angle y, Angle z) { // applied in the order X, Y, Z (right-hand rule)
        rotation_x_ = x;
        rotation_y_ = y;
        rotation_z_ = z;
        return *this;
    }

    Model& angle_of_attack(Angle pitch) { // additional pitch after the rotation, positive: nose up
        angle_of_attack_ = pitch;
        return *this;
    }

    Model& repair_mesh() { // fill holes of a non-watertight mesh before the SDF is generated
        repair_mesh_ = true;
        return *this;
    }

private:
    std::string file_;
    Angle rotation_x_{}, rotation_y_{}, rotation_z_{};
    Angle angle_of_attack_{};
    bool repair_mesh_ = false;

    friend class Domain;
};

// The simulation box in physical units:
//   Domain::box(1.0_m, 5.0_m, 0.75_m).vram(2000_mb)
//   Domain::around(Model("hill.stl")).clearances(2_m, 500_m, 100_m).cell_size(8_m).max_vram(20_gb)
// Conflicting or unused settings are reported with a SetupError when the domain is used.
class Domain {
public:
    static Domain box(Length x, Length y, Length z) { // no model
        Domain domain;
        domain.size_ = { x, y, z };
        return domain;
    }

    static Domain around(Model model) { // the model plus clearances
        Domain domain;
        domain.model_ = std::move(model);
        return domain;
    }

    Domain& clearances(Length bottom, Length top, Length sides) { // around(): below, above and on each of the four sides
        clearances_ = { bottom, top, sides };
        return *this;
    }

    Domain& vram(MemorySize budget) { // resolution: the largest grid that fits (default 2000 MB)
        vram_ = budget;
        return *this;
    }

    Domain& cell_size(Length size) { // around(): resolution from the cell size ...
        cell_size_ = size;
        return *this;
    }

    Domain& max_vram(MemorySize limit) { // ... up to this memory (default 24000 MB)
        max_vram_ = limit;
        return *this;
    }

    // the planner's input; throws SetupError for conflicting or unused settings
    SimulationConfig config() const;

private:
    struct Box { Length x, y, z; };
    struct Clearances { Length bottom, top, sides; };

    std::optional<Model> model_;
    std::optional<Box> size_;
    std::optional<Clearances> clearances_;
    std::optional<MemorySize> vram_;
    std::optional<Length> cell_size_;
    std::optional<MemorySize> max_vram_;
};
