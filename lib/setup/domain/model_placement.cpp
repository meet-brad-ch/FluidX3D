#include "setup/domain/model_placement.hpp"
#include "setup/core/setup_error.hpp"

#include <filesystem>

ModelPlacement ModelPlacement::of(const DomainPlan& plan) {
    const bool has_model{ !plan.stl_path.empty() && std::filesystem::exists(plan.stl_path) };
    if(!has_model) {
        throw SetupError("ModelPlacement: the plan has no model");
    }
    const std::unique_ptr<const Mesh> model(read_stl(plan.stl_path, 1.0f, plan.rotation_matrix));
    const ModelPlacement placement(*model, plan.rotation_matrix, plan.voxel_size, plan.center_lbm);
    return placement;
}

ModelPlacement::ModelPlacement(const Mesh& rotated_model, const float3x3& rotation, float voxel_size, const float3& center)
    : rotation_(rotation),
      cells_per_unit_(voxel_size / rotated_model.get_max_size()),
      center_(center),
      translation_(center - cells_per_unit_ * rotated_model.get_bounding_box_center()) {}

std::unique_ptr<Mesh> ModelPlacement::load(const std::string& path) const {
    std::unique_ptr<Mesh> mesh(read_stl(path, cells_per_unit_, rotation_, translation_)); // the core's read_stl() returns a new Mesh
    return mesh;
}

std::unique_ptr<Mesh> ModelPlacement::load_centered(const std::string& path, const float3& offset) const {
    std::unique_ptr<Mesh> mesh{ load(path) };
    const float3 own_centre{ mesh->get_bounding_box_center() };
    const float3 move{ center_ + offset - own_centre };
    mesh->translate(move);
    return mesh;
}
