#include "setup/domain/model_placement.hpp"
#include "setup/core/setup_error.hpp"

#include <filesystem>

ModelPlacement ModelPlacement::of(const DomainPlan& plan) {
    if(plan.stl_path.empty() || !std::filesystem::exists(plan.stl_path)) throw SetupError("ModelPlacement: the plan has no model");
    const std::unique_ptr<Mesh> model(read_stl(plan.stl_path, 1.0f, plan.rotation_matrix));
    return ModelPlacement(*model, plan.rotation_matrix, plan.voxel_size, plan.center_lbm);
}

ModelPlacement::ModelPlacement(const Mesh& rotated_model, const float3x3& rotation, float voxel_size, const float3& center)
    : rotation_(rotation),
      cells_per_unit_(voxel_size / rotated_model.get_max_size()),
      center_(center),
      translation_(center - cells_per_unit_ * rotated_model.get_bounding_box_center()) {}

std::unique_ptr<Mesh> ModelPlacement::load(const std::string& path) const {
    return std::unique_ptr<Mesh>(read_stl(path, cells_per_unit_, rotation_, translation_));
}

std::unique_ptr<Mesh> ModelPlacement::load_centered(const std::string& path, const float3& offset) const {
    std::unique_ptr<Mesh> mesh = load(path);
    mesh->translate(center_ + offset - mesh->get_bounding_box_center());
    return mesh;
}
