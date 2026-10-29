#pragma once
#include "utilities.hpp"
#include "setup/boundaries/boundary_flags.hpp"
#include "setup/units/units.hpp"
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

/// @file domain.hpp
/// @brief The simulation box in physical units (Domain) and the geometry file in it (Model).

/// @brief A geometry file (in resources/) and how it is turned and sized in the box.
///
/// Its STL coordinates are metres, unless length() gives the model's real size; then the STL's own units do not
/// matter. The rotation is applied about the axes in the order X, Y, Z, then the angle of attack about X.
/// @code
/// using namespace si::symbols;
/// Model("X-Wing.stl").rotation(0.0 * deg, 0.0 * deg, 180.0 * deg).length(13.4 * m)
/// Model("crm.stl").rotation(0.0 * deg, 0.0 * deg, 90.0 * deg).angle_of_attack(-10.0 * deg).length(62.0 * m).mirrored(Axis::X)
/// @endcode
class Model {
public:
    /// @brief Names the geometry file. The file is checked when the simulation is planned.
    /// @param file the STL or SDF file name, in resources/
    explicit Model(std::string file) : file_(std::move(file)) {}

    /// @brief Sets the rotation of the model, applied about X, then Y, then Z (right-hand rule).
    /// @param x the angle about X
    /// @param y the angle about Y
    /// @param z the angle about Z
    /// @return this model
    Model& rotation(si::Angle x, si::Angle y, si::Angle z) {
        rotation_x_ = x;
        rotation_y_ = y;
        rotation_z_ = z;
        return *this;
    }

    /// @brief Sets a pitch about X after the rotation: the angle of attack of a wing or a vehicle.
    /// @param pitch the pitch, positive nose up
    /// @return this model
    Model& angle_of_attack(si::Angle pitch) {
        angle_of_attack_ = pitch;
        return *this;
    }

    /// @brief Sets the model's real extent along an axis, measured after rotation() and before angle_of_attack()
    /// (for Domain::size()).
    ///
    /// It scales the whole model, and it is the reference length of the units and of the Reynolds number.
    /// @param length the model's extent along the axis, more than 0
    /// @param axis the axis along which the extent is measured
    /// @return this model
    Model& length(si::Length length, Axis axis = Axis::Y) {
        length_ = length;
        length_axis_ = axis;
        return *this;
    }

    /// @brief Fills the holes of a mesh that is not watertight before the SDF is made (Domain::clearances() only).
    /// @return this model
    Model& repair_mesh() {
        repair_mesh_ = true;
        return *this;
    }

    /// @brief Makes a half model whole: the mesh is completed by its mirror image across the plane through the mesh
    /// origin, normal to an axis.
    /// @param axis the axis normal to the symmetry plane
    /// @return this model
    Model& mirrored(Axis axis) {
        mirror_ = axis;
        return *this;
    }

    /// @brief Names the other files of the same assembly that the simulation needs (its moving parts). They are
    /// checked together with the model's file.
    /// @param files the file names, in resources/
    /// @return this model
    Model& needs(std::vector<std::string> files) {
        required_files_ = std::move(files);
        return *this;
    }

    /// @brief Sets the text that tells the user how to get the files. It is printed line by line when a file is
    /// missing.
    /// @param lines the instructions, one line each
    /// @return this model
    Model& instructions(std::vector<std::string> lines) {
        instructions_ = std::move(lines);
        return *this;
    }

    /// @return the file name, in resources/
    const std::string& file() const { return file_; }

    /// @brief Tells whether the file is a binary SDF (its extension is ".sdf", in any case), not an STL.
    /// @return true for an SDF file
    bool is_sdf() const;

    /// @return the file names given to needs()
    const std::vector<std::string>& required_files() const { return required_files_; }

    /// @return the lines given to instructions()
    const std::vector<std::string>& instruction_lines() const { return instructions_; }

    /// @brief Gives the rotation matrix of rotation() and then angle_of_attack(), as the core's voxelizer turns the
    /// model. The angles go to the core as float degrees (the core's radians()), so the voxels are the same as with
    /// the core's own rotation.
    /// @param with_angle_of_attack true to include the angle of attack
    /// @return the matrix
    float3x3 rotation_matrix(bool with_angle_of_attack = true) const;

private:
    /// @brief Gives the core's rotation about one axis.
    /// @param axis the axis as a unit vector
    /// @param angle the angle (right-hand rule)
    /// @return the matrix
    static float3x3 rotation_about(const float3& axis, si::Angle angle);

    std::string file_{};                           ///< the file name, in resources/
    si::Angle rotation_x_{ au::ZERO };             ///< the rotation() angle about X
    si::Angle rotation_y_{ au::ZERO };             ///< the rotation() angle about Y
    si::Angle rotation_z_{ au::ZERO };             ///< the rotation() angle about Z
    si::Angle angle_of_attack_{ au::ZERO };        ///< the angle_of_attack() pitch about X
    std::optional<si::Length> length_{};           ///< length(): the real extent along length_axis_, or none
    Axis length_axis_ = Axis::Y;                   ///< the axis along which length_ is measured
    bool repair_mesh_ = false;                     ///< true after repair_mesh()
    std::optional<Axis> mirror_{};                 ///< mirrored(): the axis normal to the symmetry plane, or none
    std::vector<std::string> required_files_{};    ///< the file names given to needs()
    std::vector<std::string> instructions_{};      ///< the lines given to instructions()

    friend class Domain;
    friend class DomainPlanner;
};

/// @brief The simulation box in physical units: its size, the model in it, and how fine the grid is.
///
/// A box is either a box of fluid (box()) or a box around a model (around(), with clearances() or a size()). The
/// grid comes from a device memory budget (vram()) or from a cell size (cell_size()). DomainPlanner computes the grid
/// and the model's place from it. A setting that conflicts with another or is not used is reported with a SetupError
/// when the domain is used (validate()).
/// @code
/// using namespace si::symbols;
/// Domain::box(1.0 * m, 5.0 * m, 0.75 * m).vram(2000.0 * MiB)
/// Domain::around(Model("hill.stl")).clearances(2.0 * m, 500.0 * m, 100.0 * m).cell_size(8.0 * m).vram_limit(20.0 * GiB)
/// Domain::around(Model("Cow_t.stl").length(2.4 * m)).size(1.85 * m, 3.7 * m, 1.85 * m).gap_to_inlet(0.24 * m).on_floor().vram(1000.0 * MiB)
/// @endcode
class Domain {
public:
    /// The device memory budget of vram() when the domain sets no resolution.
    static constexpr si::MemoryAmount default_vram{ si::mebibytes(2000.0) };
    /// The device memory limit of cell_size() when vram_limit() is not set.
    static constexpr si::MemoryAmount default_vram_limit{ si::mebibytes(24000.0) };

    /// @brief Makes a box of fluid without a model.
    /// @param x the box's extent along X
    /// @param y the box's extent along Y
    /// @param z the box's extent along Z, the height
    /// @return the domain
    static Domain box(si::Length x, si::Length y, si::Length z) {
        Domain domain{};
        domain.size_ = Box{ .x = x, .y = y, .z = z };
        return domain;
    }

    /// @brief Makes a domain around a model: the model plus clearances(), or a size() around it.
    /// @param model the geometry file with its rotation and, for size(), its real length
    /// @return the domain
    static Domain around(Model model) {
        Domain domain{};
        domain.model_ = std::move(model);
        return domain;
    }

    /// @brief Sets the fluid around the model below, above and on each of the four sides (around() with an STL in
    /// metres, without Model::length()).
    /// @param bottom the clearance below the model
    /// @param top the clearance above the model
    /// @param sides the clearance on each of the four sides
    /// @return this domain
    Domain& clearances(si::Length bottom, si::Length top, si::Length sides) {
        clearances_ = Clearances{ .bottom = bottom, .top = top, .sides = sides };
        return *this;
    }

    /// @brief Sets the domain's size around the model (around(); the model needs a Model::length()). The model is
    /// centred unless it is placed with model_offset() or the gaps.
    /// @param x the domain's extent along X
    /// @param y the domain's extent along Y
    /// @param z the domain's extent along Z, the height
    /// @return this domain
    Domain& size(si::Length x, si::Length y, si::Length z) {
        size_ = Box{ .x = x, .y = y, .z = z };
        return *this;
    }

    /// @brief Moves the model's centre this far from the domain's centre (with size()).
    /// @param x the offset along X
    /// @param y the offset along Y
    /// @param z the offset along Z
    /// @return this domain
    Domain& model_offset(si::Length x, si::Length y, si::Length z) {
        model_offset_ = Box{ .x = x, .y = y, .z = z };
        return *this;
    }

    /// @brief Places the model's front (its bounding box minimum in Y) this far from the inlet at y = 0 (with size());
    /// X stays centred.
    /// @param gap the distance from the inlet to the model's front
    /// @return this domain
    Domain& gap_to_inlet(si::Length gap) {
        gap_to_inlet_ = gap;
        return *this;
    }

    /// @brief Places the model's bottom this far above the floor at z = 0 (with size()).
    /// @param gap the distance from the floor to the model's bottom
    /// @return this domain
    Domain& gap_to_floor(si::Length gap) {
        gap_to_floor_ = gap;
        return *this;
    }

    /// @brief Places the model's bottom one cell above z = 0, resting on a solid floor there, at any resolution
    /// (with size()).
    /// @return this domain
    Domain& on_floor() {
        on_floor_ = true;
        return *this;
    }

    /// @brief Sets the resolution from a device memory budget: the largest grid that fits it. Without vram() and
    /// cell_size() the budget is default_vram.
    /// @param budget the device memory the grid may use, a whole number of MiB (2000.0 * MiB)
    /// @return this domain
    Domain& vram(si::MemoryAmount budget) {
        vram_ = budget;
        return *this;
    }

    /// @brief Sets the resolution from the cell size: box() and size() get whole cells along each side (rounded to
    /// the nearest), within vram_limit().
    /// @param size the edge of one cubic cell, more than 0
    /// @return this domain
    Domain& cell_size(si::Length size) {
        cell_size_ = size;
        return *this;
    }

    /// @brief Sets the device memory that a cell_size() grid may use. Without it, default_vram_limit.
    /// @param limit the memory limit, a whole number of MiB
    /// @return this domain
    Domain& vram_limit(si::MemoryAmount limit) {
        vram_limit_ = limit;
        return *this;
    }

    /// @brief Splits the grid among this many GPUs along each axis (default one GPU). The core rounds the grid down
    /// to whole parts.
    /// @param x the GPUs along X, at least 1
    /// @param y the GPUs along Y, at least 1
    /// @param z the GPUs along Z, at least 1
    /// @return this domain
    Domain& gpus(std::uint32_t x, std::uint32_t y, std::uint32_t z) {
        gpus_ = uint3(x, y, z);
        return *this;
    }

    /// @brief Keeps every horizontal level of cells in one domain: a split along z is refused. This is for the
    /// per-level forcing (LEVEL_FORCING), whose level means are sums over one domain. The planner's own splits are
    /// along x or y.
    /// @return this domain
    Domain& keep_levels_whole() {
        keep_levels_whole_ = true;
        return *this;
    }

    /// @return the cell size given to cell_size(), or none when the resolution comes from vram()
    const std::optional<si::Length>& cell() const { return cell_size_; }

    /// @brief Checks that the settings fit together.
    /// @throws SetupError for vram() together with cell_size(); vram_limit() without cell_size(); a GPU count of 0;
    /// keep_levels_whole() with more than one GPU along z; on_floor() together with gap_to_floor(); model_offset()
    /// together with a gap; clearances(), model_offset() or a gap without a model; an SDF model without size() and
    /// Model::length(), or mirrored; size() around a model without Model::length(); size() together with
    /// clearances(); Model::length() or a placement together with clearances()
    void validate() const;

    /// @return the model, or none for box()
    const std::optional<Model>& model() const { return model_; }

private:
    /// Three lengths along the axes.
    struct Box {
        si::Length x{ au::ZERO }; ///< the length along X
        si::Length y{ au::ZERO }; ///< the length along Y
        si::Length z{ au::ZERO }; ///< the length along Z
    };
    /// The fluid around a model.
    struct Clearances {
        si::Length bottom{ au::ZERO }; ///< the clearance below the model
        si::Length top{ au::ZERO };    ///< the clearance above the model
        si::Length sides{ au::ZERO };  ///< the clearance on each of the four sides
    };

    std::optional<Model> model_{};             ///< around(): the model, or none for box()
    std::optional<Box> size_{};                ///< box() or size(): the domain's extent, or none
    std::optional<Clearances> clearances_{};   ///< clearances(), or none
    std::optional<Box> model_offset_{};        ///< model_offset(), or none
    std::optional<si::Length> gap_to_inlet_{}; ///< gap_to_inlet(), or none
    std::optional<si::Length> gap_to_floor_{}; ///< gap_to_floor(), or none
    bool on_floor_ = false;                    ///< true after on_floor()
    std::optional<si::MemoryAmount> vram_{};   ///< vram(), or none
    std::optional<si::Length> cell_size_{};    ///< cell_size(), or none
    std::optional<si::MemoryAmount> vram_limit_{}; ///< vram_limit(), or none
    uint3 gpus_ = uint3(1u, 1u, 1u);           ///< gpus(): the GPUs along each axis
    bool keep_levels_whole_ = false;           ///< true after keep_levels_whole()

    /// @brief Checks the resolution and the GPU settings.
    /// @throws SetupError for vram() together with cell_size(), vram_limit() without cell_size(), or a GPU count of 0
    void validate_resolution() const;

    /// @brief Checks that the model is placed in one way only.
    /// @throws SetupError for on_floor() together with gap_to_floor(), or model_offset() together with a gap
    void validate_placement() const;

    /// @brief Checks keep_levels_whole() against gpus().
    /// @throws SetupError for keep_levels_whole() with more than one GPU along z
    void validate_levels() const;

    /// @brief Checks the settings of a box without a model: it has no clearances and no placement.
    /// @throws SetupError for clearances(), model_offset() or a gap without a model
    void validate_box() const;

    /// @brief Checks a model in a size(): it needs Model::length() and refuses clearances().
    /// @param model the domain's model
    /// @throws SetupError for size() without Model::length(), or size() together with clearances()
    void validate_sized_model(const Model& model) const;

    /// @brief Checks a model with clearances(): its STL is in metres, so it refuses Model::length() and a placement.
    /// @param model the domain's model
    /// @throws SetupError for Model::length() or a placement together with clearances()
    void validate_cleared_model(const Model& model) const;

    /// @brief Checks the settings of a model: an SDF needs size(); size() needs Model::length() and no clearances();
    /// clearances() refuse Model::length() and a placement.
    /// @param model the domain's model
    /// @throws SetupError (see validate())
    void validate_model(const Model& model) const;

    /// @return true when the model is placed with gap_to_inlet(), gap_to_floor() or on_floor()
    bool has_gaps() const { return gap_to_inlet_ || gap_to_floor_ || on_floor_; }

    /// @brief Checks that a memory amount is a whole number of MiB, the unit that the core's memory figures count.
    /// @param amount a memory amount
    /// @param name the setting's name, for the message
    /// @return the amount, unchanged
    /// @throws SetupError when the amount is negative, not a whole number of MiB, or more than 2^32 - 1 MiB
    static si::MemoryAmount whole_mebibytes(si::MemoryAmount amount, const char* name);

    /// @brief Gives the vram() budget.
    /// @return the budget, or default_vram
    /// @throws SetupError when the budget is not a whole number of MiB
    si::MemoryAmount memory_budget() const;

    /// @brief Gives the vram_limit().
    /// @return the limit, or default_vram_limit
    /// @throws SetupError when the limit is not a whole number of MiB
    si::MemoryAmount memory_limit() const;

    friend class DomainPlanner;
};
