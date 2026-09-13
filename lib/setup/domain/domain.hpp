#pragma once
#include "utilities.hpp"
#include "setup/core/quantity.hpp"
#include "setup/boundaries/boundary_flags.hpp"
#include <optional>
#include <string>
#include <utility>
#include <vector>

/// @brief A geometry file (in resources/) and its orientation.
///
/// Its STL coordinates are metres, unless length() gives the model's real size; then the STL's own units do not
/// matter.
/// @code
/// Model("X-Wing.stl").rotation(0_deg, 0_deg, 180_deg).length(13.4_m)
/// Model("crm.stl").rotation(0_deg, 0_deg, 90_deg).angle_of_attack(-10_deg).length(62_m).mirrored(Axis::X)
/// @endcode
class Model {
public:
    /// @param file the STL or SDF file, in resources/
    explicit Model(std::string file) : file_(std::move(file)) {}

    /// @brief The rotation of the model, applied in the order X, Y, Z (right-hand rule).
    /// @param x the angle about X
    /// @param y the angle about Y
    /// @param z the angle about Z
    /// @return this model
    Model& rotation(Angle x, Angle y, Angle z) {
        rotation_x_ = x;
        rotation_y_ = y;
        rotation_z_ = z;
        return *this;
    }

    /// @brief An additional pitch after the rotation.
    /// @param pitch the pitch, positive nose up
    /// @return this model
    Model& angle_of_attack(Angle pitch) {
        angle_of_attack_ = pitch;
        return *this;
    }

    /// @brief The model's real extent along an axis, measured after rotation() and before angle_of_attack() (for
    /// Domain::size()).
    ///
    /// It scales the whole model and is the reference length of the units and the Reynolds number.
    /// @param length the extent
    /// @param axis the axis it is measured along
    /// @return this model
    Model& length(Length length, Axis axis = Axis::Y) {
        length_ = length;
        length_axis_ = axis;
        return *this;
    }

    /// @brief Fill the holes of a non-watertight mesh before the SDF is generated.
    /// @return this model
    Model& repair_mesh() {
        repair_mesh_ = true;
        return *this;
    }

    /// @brief A half model: completed by its mirror image across the plane through the mesh origin normal to an axis.
    /// @param axis the axis normal to the symmetry plane
    /// @return this model
    Model& mirrored(Axis axis) {
        mirror_ = axis;
        return *this;
    }

    /// @brief Other files of the same assembly the simulation needs (its moving parts), checked with the model's file.
    /// @param files the file names, in resources/
    /// @return this model
    Model& needs(std::vector<std::string> files) {
        required_files_ = std::move(files);
        return *this;
    }

    /// @brief How to get the files, printed line by line when one is missing.
    /// @param lines the instructions, one line each
    /// @return this model
    Model& instructions(std::vector<std::string> lines) {
        instructions_ = std::move(lines);
        return *this;
    }

    /// @return the file, in resources/
    const std::string& file() const { return file_; }

    /// @return whether the file is a binary SDF (".sdf"), not an STL
    bool is_sdf() const;

    /// @return the files of needs()
    const std::vector<std::string>& required_files() const { return required_files_; }

    /// @return the lines of instructions()
    const std::vector<std::string>& instruction_lines() const { return instructions_; }

    /// @brief The rotation matrix of rotation() and then angle_of_attack(), as the core's voxelizer turns the model.
    /// @param with_angle_of_attack whether the angle of attack is included
    /// @return the matrix
    float3x3 rotation_matrix(bool with_angle_of_attack = true) const;

private:
    std::string file_;                             ///< the file, in resources/
    Angle rotation_x_{};                           ///< rotation() about X
    Angle rotation_y_{};                           ///< rotation() about Y
    Angle rotation_z_{};                           ///< rotation() about Z
    Angle angle_of_attack_{};                      ///< angle_of_attack()
    std::optional<Length> length_;                 ///< length(): the real extent along length_axis_
    Axis length_axis_ = Axis::Y;                   ///< the axis of length_
    bool repair_mesh_ = false;                     ///< repair_mesh()
    std::optional<Axis> mirror_;                   ///< mirrored(): the axis normal to the symmetry plane
    std::vector<std::string> required_files_;      ///< needs()
    std::vector<std::string> instructions_;        ///< instructions()

    friend class Domain;
    friend class DomainPlanner;
};

/// @brief The simulation box in physical units.
///
/// @code
/// Domain::box(1.0_m, 5.0_m, 0.75_m).vram(2000_mb)
/// Domain::around(Model("hill.stl")).clearances(2_m, 500_m, 100_m).cell_size(8_m).vram_limit(20_gb)
/// Domain::around(Model("Cow_t.stl").length(2.4_m)).size(1.85_m, 3.7_m, 1.85_m).gap_to_inlet(0.24_m).on_floor().vram(1000_mb)
/// Domain::box(0.5_m, 1.0_m, 1.0_m).cell_size(1.0_m / 256.0f)   // 128 x 256 x 256 cells
/// @endcode
/// DomainPlanner computes the grid and the model's place from it. Conflicting or unused settings are reported with a
/// SetupError when the domain is used.
class Domain {
public:
    /// @brief A box of fluid without a model.
    /// @param x, y, z the box's sides
    /// @return the domain
    static Domain box(Length x, Length y, Length z) {
        Domain domain;
        domain.size_ = { x, y, z };
        return domain;
    }

    /// @brief A domain around a model: the model plus clearances(), or a size() around it.
    /// @param model the model
    /// @return the domain
    static Domain around(Model model) {
        Domain domain;
        domain.model_ = std::move(model);
        return domain;
    }

    /// @brief around(): the fluid below, above and on each of the four sides of the model, whose STL is in metres.
    /// @param bottom the clearance below the model
    /// @param top the clearance above it
    /// @param sides the clearance on each of the four sides
    /// @return this domain
    Domain& clearances(Length bottom, Length top, Length sides) {
        clearances_ = { bottom, top, sides };
        return *this;
    }

    /// @brief around(): the domain's size; the model (which needs a length()) is centered unless placed.
    /// @param x, y, z the domain's sides
    /// @return this domain
    Domain& size(Length x, Length y, Length z) {
        size_ = { x, y, z };
        return *this;
    }

    /// @brief size(): the model's center, this far from the domain's center.
    /// @param x, y, z the offset along each axis
    /// @return this domain
    Domain& model_offset(Length x, Length y, Length z) {
        model_offset_ = { x, y, z };
        return *this;
    }

    /// @brief size(): the model's front (bounding box minimum in Y) this far from the inlet at y = 0; X stays centered.
    /// @param gap the gap
    /// @return this domain
    Domain& gap_to_inlet(Length gap) {
        gap_to_inlet_ = gap;
        return *this;
    }

    /// @brief size(): the model's bottom this far above the floor at z = 0.
    /// @param gap the gap
    /// @return this domain
    Domain& gap_to_floor(Length gap) {
        gap_to_floor_ = gap;
        return *this;
    }

    /// @brief size(): the model's bottom one cell above z = 0, resting on a solid floor there, at any resolution.
    /// @return this domain
    Domain& on_floor() {
        on_floor_ = true;
        return *this;
    }

    /// @brief The resolution from a device memory budget: the largest grid that fits (default 2000 MB).
    /// @param budget the budget
    /// @return this domain
    Domain& vram(MemorySize budget) {
        vram_ = budget;
        return *this;
    }

    /// @brief The resolution from the cell size (box() and size(): whole cells, rounded), within vram_limit().
    /// @param size the cell size
    /// @return this domain
    Domain& cell_size(Length size) {
        cell_size_ = size;
        return *this;
    }

    /// @brief cell_size(): the device memory the grid may use (default 24000 MB).
    /// @param limit the limit
    /// @return this domain
    Domain& vram_limit(MemorySize limit) {
        vram_limit_ = limit;
        return *this;
    }

    /// @brief The grid split among this many GPUs along each axis (default one GPU); the core rounds the grid down to
    /// whole parts.
    /// @param x, y, z the GPUs along each axis
    /// @return this domain
    Domain& gpus(uint32_t x, uint32_t y, uint32_t z) {
        gpus_ = uint3(x, y, z);
        return *this;
    }

    /// @brief Checks the settings.
    /// @throws SetupError for conflicting or unused settings (DomainPlanner::plan() checks them too)
    void validate() const;

    /// @return the model, or none for box()
    const std::optional<Model>& model() const { return model_; }

private:
    /// Three lengths along the axes.
    struct Box {
        Length x; ///< along X
        Length y; ///< along Y
        Length z; ///< along Z
    };
    /// The fluid around a model.
    struct Clearances {
        Length bottom; ///< below the model
        Length top;    ///< above it
        Length sides;  ///< on each of the four sides
    };

    std::optional<Model> model_;           ///< around(): the model
    std::optional<Box> size_;              ///< box() or size()
    std::optional<Clearances> clearances_; ///< clearances()
    std::optional<Box> model_offset_;      ///< model_offset()
    std::optional<Length> gap_to_inlet_;   ///< gap_to_inlet()
    std::optional<Length> gap_to_floor_;   ///< gap_to_floor()
    bool on_floor_ = false;                ///< on_floor()
    std::optional<MemorySize> vram_;       ///< vram()
    std::optional<Length> cell_size_;      ///< cell_size()
    std::optional<MemorySize> vram_limit_; ///< vram_limit()
    uint3 gpus_ = uint3(1u, 1u, 1u);       ///< gpus()

    /// @return whether the model is placed with gap_to_inlet(), gap_to_floor() or on_floor()
    bool has_gaps() const { return gap_to_inlet_ || gap_to_floor_ || on_floor_; }

    /// @return the vram() budget in MB, or the default
    uint32_t vram_mb() const { return vram_ ? vram_->mb() : 2000u; }

    /// @return the vram_limit() in MB, or the default
    uint32_t max_vram_mb() const { return vram_limit_ ? vram_limit_->mb() : 24000u; }

    friend class DomainPlanner;
};
