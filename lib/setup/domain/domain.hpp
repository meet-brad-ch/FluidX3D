#pragma once
#include "utilities.hpp"
#include "setup/core/quantity.hpp"
#include "setup/boundaries/boundary_flags.hpp"
#include "setup/config/simulation_config.hpp"
#include <optional>
#include <string>

// A geometry file (in resources/) and its orientation. Its STL coordinates are metres, unless length() gives the
// model's real size; then the STL's own units do not matter.
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

    /// The model's real extent along an axis, measured after rotation() and before angle_of_attack() (for Domain::size()).
    /// It scales the whole model and is the reference length of the units and the Reynolds number.
    Model& length(Length length, Axis axis = Axis::Y) {
        length_ = length;
        length_axis_ = axis;
        return *this;
    }

    Model& repair_mesh() { // fill holes of a non-watertight mesh before the SDF is generated
        repair_mesh_ = true;
        return *this;
    }

    // half model: completed by its mirror image across the plane through the mesh origin normal to axis
    Model& mirrored(Axis axis) {
        mirror_ = axis;
        return *this;
    }

private:
    std::string file_;
    Angle rotation_x_{}, rotation_y_{}, rotation_z_{};
    Angle angle_of_attack_{};
    std::optional<Length> length_;
    Axis length_axis_ = Axis::Y;
    bool repair_mesh_ = false;
    std::optional<Axis> mirror_;

    friend class Domain;
};

// The simulation box in physical units:
//   Domain::box(1.0_m, 5.0_m, 0.75_m).vram(2000_mb)
//   Domain::around(Model("hill.stl")).clearances(2_m, 500_m, 100_m).cell_size(8_m).max_vram(20_gb)
//   Domain::around(Model("Cow_t.stl").length(2.4_m)).size(1.85_m, 3.7_m, 1.85_m).gap_to_inlet(0.24_m).on_floor().vram(1000_mb)
//   Domain::box(0.5_m, 1.0_m, 1.0_m).cell_size(1.0_m / 256.0f)   // 128 x 256 x 256 cells
// Conflicting or unused settings are reported with a SetupError when the domain is used.
class Domain {
public:
    static Domain box(Length x, Length y, Length z) { // no model
        Domain domain;
        domain.size_ = { x, y, z };
        return domain;
    }

    static Domain around(Model model) { // the model plus clearances, or a size() around it
        Domain domain;
        domain.model_ = std::move(model);
        return domain;
    }

    Domain& clearances(Length bottom, Length top, Length sides) { // around(): below, above and on each of the four sides
        clearances_ = { bottom, top, sides };
        return *this;
    }

    // around(): the domain's size (the model needs a length()); the model is centered unless placed
    Domain& size(Length x, Length y, Length z) {
        size_ = { x, y, z };
        return *this;
    }

    // size(): the model's center, this far from the domain's center
    Domain& model_offset(Length x, Length y, Length z) {
        model_offset_ = { x, y, z };
        return *this;
    }

    // size(): the model's front (bounding box minimum in Y) this far from the inlet at y = 0; X stays centered
    Domain& gap_to_inlet(Length gap) {
        gap_to_inlet_ = gap;
        return *this;
    }

    // size(): the model's bottom this far above the floor at z = 0
    Domain& gap_to_floor(Length gap) {
        gap_to_floor_ = gap;
        return *this;
    }

    // size(): the model's bottom one cell above z = 0, resting on a solid floor there, at any resolution
    Domain& on_floor() {
        on_floor_ = true;
        return *this;
    }

    Domain& vram(MemorySize budget) { // resolution: the largest grid that fits (default 2000 MB)
        vram_ = budget;
        return *this;
    }

    Domain& cell_size(Length size) { // resolution from the cell size (box() and size(): whole cells, rounded) ...
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

    void set_resolution(SimulationConfig& config) const; // cell_size() and max_vram(), or vram()

    std::optional<Model> model_;
    std::optional<Box> size_;
    std::optional<Clearances> clearances_;
    std::optional<Box> model_offset_;
    std::optional<Length> gap_to_inlet_;
    std::optional<Length> gap_to_floor_;
    bool on_floor_ = false;
    std::optional<MemorySize> vram_;
    std::optional<Length> cell_size_;
    std::optional<MemorySize> max_vram_;
};
