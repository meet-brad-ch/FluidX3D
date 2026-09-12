#include "setup/domain/domain.hpp"
#include "setup/core/setup_error.hpp"

namespace {
SimulationConfig::ReferenceAxis reference_axis(Axis axis) {
    switch(axis) {
        case Axis::X: return SimulationConfig::ReferenceAxis::X;
        case Axis::Z: return SimulationConfig::ReferenceAxis::Z;
        default: return SimulationConfig::ReferenceAxis::Y;
    }
}
} // namespace

SimulationConfig Domain::config() const {
    if(vram_ && cell_size_) throw SetupError("Domain: set the resolution with vram() or with cell_size(), not both");
    if(max_vram_ && !cell_size_) throw SetupError("Domain: max_vram() limits cell_size(); with vram() the budget is the limit");
    const bool gaps = gap_to_inlet_ || gap_to_floor_;
    if(model_offset_ && gaps) throw SetupError("Domain: place the model with model_offset() or with the gaps, not both");

    if(!model_) {
        if(clearances_) throw SetupError("Domain::box() has no model to keep clearances around");
        if(cell_size_) throw SetupError("Domain::box() is sized by vram(), not cell_size()");
        if(model_offset_ || gaps) throw SetupError("Domain::box() has no model to place");
        SimulationConfig config;
        config.set_domain_size_m(size_->x.si(), size_->y.si(), size_->z.si());
        if(vram_) config.set_vram_mb(vram_->mb());
        return config;
    }

    SimulationConfig config(model_->file_);
    config.set_rotation_deg(model_->rotation_x_.deg(), model_->rotation_y_.deg(), model_->rotation_z_.deg());
    if(model_->angle_of_attack_ != Angle{}) config.set_angle_of_attack_deg(model_->angle_of_attack_.deg());
    if(model_->repair_mesh_) config.set_fix_mesh(true);
    if(model_->mirror_) {
        config.set_mirror_plane(*model_->mirror_ == Axis::X ? SimulationConfig::MirrorPlane::X
                              : *model_->mirror_ == Axis::Y ? SimulationConfig::MirrorPlane::Y : SimulationConfig::MirrorPlane::Z);
    }

    if(size_) { // a size in metres around the model, scaled to its real length
        if(!model_->length_) throw SetupError("Domain::size() around a model needs the model's real size: Model::length()");
        if(clearances_) throw SetupError("Domain: give the size() or the clearances(), not both");
        if(cell_size_) throw SetupError("Domain::size() is resolved with vram(), not cell_size()");
        const float length = model_->length_->si();
        const Axis axis = model_->length_axis_;
        const float size_along_axis = axis == Axis::X ? size_->x.si() : axis == Axis::Y ? size_->y.si() : size_->z.si();
        config.set_domain_aspect_ratio(size_->x.si(), size_->y.si(), size_->z.si()); // metres: the same grid as the ratio
        config.set_geometry_scale(length / size_along_axis);
        config.set_reference_axis(reference_axis(axis));
        config.set_reference_length_m(length);
        if(model_offset_) {
            config.set_center_offset_ratio(model_offset_->x.si() / length, model_offset_->y.si() / length, model_offset_->z.si() / length);
        }
        if(gaps) {
            config.set_pmin_offset_ratio(0.0f, (gap_to_inlet_ ? gap_to_inlet_->si() : 0.0f) / length,
                                               (gap_to_floor_ ? gap_to_floor_->si() : 0.0f) / length);
        }
        if(vram_) config.set_vram_mb(vram_->mb());
        return config;
    }

    // clearances around the model; its STL is in metres
    if(model_->length_) throw SetupError("Model::length() is for Domain::size(); with clearances() the STL is in metres");
    if(model_offset_ || gaps) throw SetupError("Domain: placing the model needs size(); with clearances() it sits centered on the bottom clearance");
    if(clearances_) config.set_clearances_m(clearances_->bottom.si(), clearances_->top.si(), clearances_->sides.si());
    if(cell_size_) {
        config.set_voxel_size_m(cell_size_->si());
        if(max_vram_) config.set_max_vram_mb(max_vram_->mb());
    } else if(vram_) {
        config.set_vram_mb(vram_->mb());
    }
    return config;
}
