#pragma once

#include "setup/core/types.hpp"

// Geometry, domain and resolution settings for SimulationSetup (fluent setters).
// Three domain modes, set by the constructor and the last mode setter:
// - GEOMETRY_BASED (SimulationConfig("file.stl")): domain = geometry + clearances in m
// - ASPECT_RATIO (set_domain_aspect_ratio): domain from aspect ratio + VRAM, geometry scaled to a fraction of the reference axis
// - DOMAIN_ONLY (SimulationConfig() or set_domain_size_m): no geometry, domain size in m
// Geometry files are loaded from resources/. Rotations are in degrees, lengths in m.
class SimulationConfig {
public:
    enum class ReferenceAxis { X, Y, Z, MAX, MIN }; // geometry dimension used for scaling and units (default Y)
    enum class ResolutionMode { VRAM_BUDGET, VOXEL_SIZE };
    enum class DomainMode { GEOMETRY_BASED, ASPECT_RATIO, DOMAIN_ONLY };
    enum class MirrorPlane { NONE, X, Y, Z }; // half models are mirrored across this plane

private:
    string geometry_filename;
    bool use_sdf { false }; // geometry file is a binary SDF

    float32_t rotation_x{}; // degrees
    float32_t rotation_y{};
    float32_t rotation_z{};

    ReferenceAxis reference_axis { ReferenceAxis::Y };

    float32_t bottom_clearance_m{};
    float32_t top_clearance_m{};
    float32_t side_clearance_m{};

    ResolutionMode resolution_mode_ { ResolutionMode::VRAM_BUDGET };
    uint32_t vram_mb { 2000u };           // VRAM_BUDGET mode
    float32_t voxel_size_m_ { 0.0f };     // VOXEL_SIZE mode
    uint32_t max_vram_mb_ { 24000u };     // VOXEL_SIZE mode: limit

    DomainMode domain_mode_ { DomainMode::GEOMETRY_BASED };
    float32_t aspect_x_ { 1.0f };         // ASPECT_RATIO mode
    float32_t aspect_y_ { 1.0f };
    float32_t aspect_z_ { 1.0f };
    float32_t geometry_scale_ { 1.0f };   // ASPECT_RATIO mode: geometry size as fraction of the reference axis

    float32_t domain_size_x_m_ { 1.0f };  // DOMAIN_ONLY mode
    float32_t domain_size_y_m_ { 1.0f };
    float32_t domain_size_z_m_ { 1.0f };

    float32_t angle_of_attack_deg_ { 0.0f };

    float32_t center_offset_x_ { 0.0f };  // ASPECT_RATIO mode: ratio of the geometry reference length
    float32_t center_offset_y_ { 0.0f };
    float32_t center_offset_z_ { 0.0f };

    bool has_pmin_offset_ { false };      // ASPECT_RATIO mode: place by bounding box minimum instead of center
    float3 pmin_offset_ratio_ { 0.0f, 0.0f, 0.0f };

    bool fix_mesh_ { false };
    MirrorPlane mirror_plane_ { MirrorPlane::NONE };

    friend class SimulationSetup;

public:
    // geometry file in resources/; ".sdf" files are loaded as SDF, everything else as STL
    SimulationConfig(const string& geometry_file) : geometry_filename(geometry_file) {
        const size_t dot_pos = geometry_file.rfind('.');
        if(dot_pos != string::npos) {
            string extension = to_lower(geometry_file.substr(dot_pos));
            use_sdf = (extension == ".sdf");
        }
    }

    // no geometry; set the domain size with set_domain_size_m()
    SimulationConfig() : geometry_filename(""), domain_mode_(DomainMode::DOMAIN_ONLY) {}

    SimulationConfig& set_domain_size_m(float32_t x_m, float32_t y_m, float32_t z_m) {
        domain_mode_ = DomainMode::DOMAIN_ONLY;
        domain_size_x_m_ = x_m;
        domain_size_y_m_ = y_m;
        domain_size_z_m_ = z_m;
        geometry_filename = "";
        return *this;
    }

    // applied in order X, Y, Z (right-hand rule)
    SimulationConfig& set_rotation_deg(float32_t x, float32_t y, float32_t z) {
        rotation_x = x; rotation_y = y; rotation_z = z;
        return *this;
    }

    SimulationConfig& set_reference_axis(ReferenceAxis axis) {
        reference_axis = axis;
        return *this;
    }

    // space below, above and on each of the four sides of the geometry (GEOMETRY_BASED mode)
    SimulationConfig& set_clearances_m(float32_t bottom_m, float32_t top_m, float32_t side_m = 0.0f) {
        bottom_clearance_m = bottom_m;
        top_clearance_m = top_m;
        side_clearance_m = side_m;
        return *this;
    }

    // resolution from a VRAM budget
    SimulationConfig& set_vram_mb(uint32_t mb) {
        resolution_mode_ = ResolutionMode::VRAM_BUDGET;
        vram_mb = mb;
        return *this;
    }

    // resolution from the cell size; exits if the grid needs more than set_max_vram_mb() (default 24 GB)
    SimulationConfig& set_voxel_size_m(float32_t size_meters) {
        resolution_mode_ = ResolutionMode::VOXEL_SIZE;
        voxel_size_m_ = size_meters;
        return *this;
    }

    SimulationConfig& set_max_vram_mb(uint32_t mb) {
        max_vram_mb_ = mb;
        return *this;
    }

    // repair holes in non-watertight meshes before SDF generation
    SimulationConfig& set_fix_mesh(bool fix) {
        fix_mesh_ = fix;
        return *this;
    }

    // ASPECT_RATIO mode; the half model's symmetry plane must be at the mesh origin
    SimulationConfig& set_mirror_plane(MirrorPlane plane) {
        mirror_plane_ = plane;
        return *this;
    }

    // domain from this aspect ratio and the VRAM budget; set the geometry size with set_geometry_scale()
    SimulationConfig& set_domain_aspect_ratio(float32_t x, float32_t y, float32_t z) {
        domain_mode_ = DomainMode::ASPECT_RATIO;
        aspect_x_ = x;
        aspect_y_ = y;
        aspect_z_ = z;
        return *this;
    }

    // geometry length as fraction of the domain's reference axis (ASPECT_RATIO mode)
    SimulationConfig& set_geometry_scale(float32_t scale) {
        geometry_scale_ = scale;
        return *this;
    }

    // additional pitch after the rotation (positive: nose up)
    SimulationConfig& set_angle_of_attack_deg(float32_t pitch_deg) {
        angle_of_attack_deg_ = pitch_deg;
        return *this;
    }

    // geometry center offset from the domain center, as ratio of the geometry reference length (ASPECT_RATIO mode)
    SimulationConfig& set_center_offset_ratio(float32_t x, float32_t y, float32_t z) {
        center_offset_x_ = x;
        center_offset_y_ = y;
        center_offset_z_ = z;
        return *this;
    }

    // bounding box minimum at this offset from the domain origin, as ratio of the reference length;
    // X stays centered (ASPECT_RATIO mode, replaces set_center_offset_ratio)
    SimulationConfig& set_pmin_offset_ratio(float32_t x, float32_t y, float32_t z) {
        has_pmin_offset_ = true;
        pmin_offset_ratio_ = float3(x, y, z);
        return *this;
    }
};
