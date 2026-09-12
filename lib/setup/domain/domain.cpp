#include "setup/domain/domain.hpp"
#include "setup/core/setup_error.hpp"

bool Model::is_sdf() const {
    const size_t dot = file_.rfind('.');
    return dot != std::string::npos && to_lower(file_.substr(dot)) == ".sdf";
}

float3x3 Model::rotation_matrix(bool with_angle_of_attack) const {
    const float3x3 Rx = float3x3(float3(1, 0, 0), radians(rotation_x_.deg()));
    const float3x3 Ry = float3x3(float3(0, 1, 0), radians(rotation_y_.deg()));
    const float3x3 Rz = float3x3(float3(0, 0, 1), radians(rotation_z_.deg()));
    const float3x3 rotation = Rz * Ry * Rx;
    if(with_angle_of_attack && angle_of_attack_.deg() != 0.0f) { // a pitch about X after the rotation
        return float3x3(float3(1, 0, 0), radians(angle_of_attack_.deg())) * rotation;
    }
    return rotation;
}

void Domain::validate() const {
    if(vram_ && cell_size_) throw SetupError("Domain: set the resolution with vram() or with cell_size(), not both");
    if(max_vram_ && !cell_size_) throw SetupError("Domain: max_vram() limits cell_size(); with vram() the budget is the limit");
    if(on_floor_ && gap_to_floor_) throw SetupError("Domain: place the model on_floor() or at a gap_to_floor(), not both");
    if(model_offset_ && has_gaps()) throw SetupError("Domain: place the model with model_offset() or with the gaps, not both");

    if(!model_) {
        if(clearances_) throw SetupError("Domain::box() has no model to keep clearances around");
        if(model_offset_ || has_gaps()) throw SetupError("Domain::box() has no model to place");
        return;
    }
    if(size_) { // a size in metres around the model, scaled to its real length
        if(!model_->length_) throw SetupError("Domain::size() around a model needs the model's real size: Model::length()");
        if(clearances_) throw SetupError("Domain: give the size() or the clearances(), not both");
        return;
    }
    // clearances around the model; its STL is in metres
    if(model_->length_) throw SetupError("Model::length() is for Domain::size(); with clearances() the STL is in metres");
    if(model_offset_ || has_gaps()) throw SetupError("Domain: placing the model needs size(); with clearances() it sits centered on the bottom clearance");
}
