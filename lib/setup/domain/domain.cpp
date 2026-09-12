#include "setup/domain/domain.hpp"
#include "setup/core/setup_error.hpp"

SimulationConfig Domain::config() const {
    if(vram_ && cell_size_) throw SetupError("Domain: set the resolution with vram() or with cell_size(), not both");
    if(max_vram_ && !cell_size_) throw SetupError("Domain: max_vram() limits cell_size(); with vram() the budget is the limit");

    if(!model_) {
        if(clearances_) throw SetupError("Domain::box() has no model to keep clearances around");
        if(cell_size_) throw SetupError("Domain::box() is sized by vram(), not cell_size()");
        SimulationConfig config;
        config.set_domain_size_m(size_->x.si(), size_->y.si(), size_->z.si());
        if(vram_) config.set_vram_mb(vram_->mb());
        return config;
    }

    SimulationConfig config(model_->file_);
    config.set_rotation_deg(model_->rotation_x_.deg(), model_->rotation_y_.deg(), model_->rotation_z_.deg());
    if(model_->angle_of_attack_ != Angle{}) config.set_angle_of_attack_deg(model_->angle_of_attack_.deg());
    if(model_->repair_mesh_) config.set_fix_mesh(true);
    if(clearances_) config.set_clearances_m(clearances_->bottom.si(), clearances_->top.si(), clearances_->sides.si());
    if(cell_size_) {
        config.set_voxel_size_m(cell_size_->si());
        if(max_vram_) config.set_max_vram_mb(max_vram_->mb());
    } else if(vram_) {
        config.set_vram_mb(vram_->mb());
    }
    return config;
}
