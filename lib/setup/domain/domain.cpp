#include "setup/domain/domain.hpp"
#include "setup/core/setup_error.hpp"

#include <cmath>
#include <limits>
#include <string>

bool Model::is_sdf() const {
    const std::size_t dot{ file_.rfind('.') };
    if(dot == std::string::npos) {
        return false;
    }
    const std::string extension{ to_lower(file_.substr(dot)) };
    const bool sdf{ extension == ".sdf" };
    return sdf;
}

float3x3 Model::rotation_about(const float3& axis, si::Angle angle) {
    // the core's order: the angle as a float in degrees, then the core's radians() in float
    const double angle_deg{ angle.in(au::degrees) };
    const float angle_deg_float{ static_cast<float>(angle_deg) };
    const float angle_rad{ radians(angle_deg_float) };
    const float3x3 matrix{ float3x3(axis, angle_rad) };
    return matrix;
}

float3x3 Model::rotation_matrix(bool with_angle_of_attack) const {
    const float3 x_axis{ 1.0f, 0.0f, 0.0f };
    const float3 y_axis{ 0.0f, 1.0f, 0.0f };
    const float3 z_axis{ 0.0f, 0.0f, 1.0f };
    const float3x3 rx{ rotation_about(x_axis, rotation_x_) };
    const float3x3 ry{ rotation_about(y_axis, rotation_y_) };
    const float3x3 rz{ rotation_about(z_axis, rotation_z_) };
    const float3x3 rotation{ rz * ry * rx };
    const bool pitched{ angle_of_attack_ != au::ZERO };
    if(!with_angle_of_attack || !pitched) {
        return rotation;
    }
    const float3x3 pitch{ rotation_about(x_axis, angle_of_attack_) }; // a pitch about X after the rotation
    const float3x3 pitched_rotation{ pitch * rotation };
    return pitched_rotation;
}

void Domain::validate() const {
    validate_resolution();
    validate_levels();
    validate_placement();
    if(model_.has_value()) {
        const Model& model{ model_.value() };
        validate_model(model);
    } else {
        validate_box();
    }
}

void Domain::validate_resolution() const {
    if(vram_ && cell_size_) {
        throw SetupError("Domain: set the resolution with vram() or with cell_size(), not both");
    }
    if(vram_limit_ && !cell_size_) {
        throw SetupError("Domain: vram_limit() limits cell_size(); with vram() the budget is the limit");
    }
    const bool no_gpu{ gpus_.x == 0u || gpus_.y == 0u || gpus_.z == 0u };
    if(no_gpu) {
        throw SetupError("Domain: gpus() needs at least one GPU along each axis");
    }
}

void Domain::validate_placement() const {
    if(on_floor_ && gap_to_floor_) {
        throw SetupError("Domain: place the model on_floor() or at a gap_to_floor(), not both");
    }
    if(model_offset_ && has_gaps()) {
        throw SetupError("Domain: place the model with model_offset() or with the gaps, not both");
    }
}

void Domain::validate_box() const {
    if(clearances_) {
        throw SetupError("Domain::box() has no model to keep clearances around");
    }
    const bool placed{ model_offset_.has_value() || has_gaps() };
    if(placed) {
        throw SetupError("Domain::box() has no model to place");
    }
}

void Domain::validate_model(const Model& model) const {
    const bool sdf{ model.is_sdf() };
    const bool sdf_without_size{ sdf && (!size_.has_value() || model.mirror_.has_value()) };
    if(sdf_without_size) {
        throw SetupError("Domain: an SDF model needs a size() and its length(), and cannot be mirrored");
    }
    if(size_.has_value()) {
        validate_sized_model(model);
    } else {
        validate_cleared_model(model);
    }
}

void Domain::validate_sized_model(const Model& model) const {
    // a size in metres around the model, scaled to its real length
    if(!model.length_.has_value()) {
        throw SetupError("Domain::size() around a model needs the model's real size: Model::length()");
    }
    if(clearances_) {
        throw SetupError("Domain: give the size() or the clearances(), not both");
    }
}

void Domain::validate_cleared_model(const Model& model) const {
    // clearances around the model; its STL is in metres
    const bool placed{ model_offset_.has_value() || has_gaps() };
    if(model.length_.has_value()) {
        throw SetupError("Model::length() is for Domain::size(); with clearances() the STL is in metres");
    }
    if(placed) {
        throw SetupError("Domain: placing the model needs size(); with clearances() it sits centered on the bottom clearance");
    }
}

void Domain::validate_levels() const {
    if(keep_levels_whole_ && gpus_.z > 1u) {
        throw SetupError("Domain: keep_levels_whole() refuses a split along z: gpus() along z must be 1");
    }
}

si::MemoryAmount Domain::whole_mebibytes(si::MemoryAmount amount, const char* name) {
    const double mebibytes{ amount.in(si::symbols::MiB) };
    const double whole{ std::floor(mebibytes) };
    const double largest{ static_cast<double>(std::numeric_limits<std::uint32_t>::max()) };
    const bool valid{ mebibytes >= 0.0 && whole == mebibytes && mebibytes <= largest };
    if(!valid) {
        const std::string text{ std::to_string(mebibytes) };
        const std::string message{ std::string("Domain: ") + name + " must be a whole number of MiB from 0 to 2^32 - 1, not " + text + " MiB" };
        throw SetupError(message);
    }
    return amount;
}

si::MemoryAmount Domain::memory_budget() const {
    if(!vram_.has_value()) {
        return default_vram;
    }
    const si::MemoryAmount budget{ whole_mebibytes(vram_.value(), "vram()") };
    return budget;
}

si::MemoryAmount Domain::memory_limit() const {
    if(!vram_limit_.has_value()) {
        return default_vram_limit;
    }
    const si::MemoryAmount limit{ whole_mebibytes(vram_limit_.value(), "vram_limit()") };
    return limit;
}
