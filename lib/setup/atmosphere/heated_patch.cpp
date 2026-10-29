#include "setup/atmosphere/heated_patch.hpp"
#include "setup/core/setup_error.hpp"

#include <cmath>
#include <cstdio>

HeatedPatch HeatedPatch::disc(si::Length diameter, si::Length north_of_middle, PatchMode mode, std::uint32_t layers) {
    if(!(diameter > au::ZERO)) {
        throw SetupError("HeatedPatch: the diameter must be more than 0 m (no patch: HeatedPatch::none())");
    }
    const bool heated_layers{ mode == PatchMode::HEATED_LAYERS };
    if(heated_layers && layers == 0u) {
        throw SetupError("HeatedPatch: HEATED_LAYERS needs at least 1 layer");
    }
    const bool surface_layer{ mode == PatchMode::SURFACE_LAYER };
    const std::uint32_t disc_layers{ surface_layer ? 1u : layers };
    HeatedPatch patch;
    patch.disc_ = Disc{ .diameter = diameter, .north = north_of_middle, .mode = mode, .layers = disc_layers };
    return patch;
}

const HeatedPatch::Disc& HeatedPatch::disc() const {
    if(!disc_) {
        throw SetupError("HeatedPatch: no patch");
    }
    return *disc_;
}

PatchMode HeatedPatch::mode() const {
    const PatchMode patch_mode{ disc().mode };
    return patch_mode;
}

std::uint32_t HeatedPatch::layers() const {
    const std::uint32_t layer_count{ disc().layers };
    return layer_count;
}

std::uint32_t HeatedPatch::flux_factor() const {
    const Disc& d{ disc() };
    std::uint32_t factor{ d.layers };
    if(d.mode == PatchMode::SURFACE_LAYER) {
        factor = surface_layer_factor;
    }
    return factor;
}

bool HeatedPatch::covers(std::uint32_t x, std::uint32_t y, std::uint32_t nx, std::uint32_t ny, si::Length cell) const {
    bool inside{ false };
    if(disc_) {
        constexpr double half = 0.5; // the middle of the box, a cell's centre at i + 1/2, the radius from the diameter
        const double nx_double{ static_cast<double>(nx) };
        const double ny_double{ static_cast<double>(ny) };
        const double cx{ half * nx_double };
        const double middle_y{ half * ny_double };
        const auto north_ratio = disc_->north / cell;
        const double north_cells{ au::as_raw_number(north_ratio) };
        const double cy{ middle_y + north_cells };
        const auto diameter_ratio = disc_->diameter / cell;
        const double diameter_cells{ au::as_raw_number(diameter_ratio) };
        const double r{ half * diameter_cells }; // in cells
        const double x_double{ static_cast<double>(x) };
        const double y_double{ static_cast<double>(y) };
        const double x_centre{ x_double + half };
        const double y_centre{ y_double + half };
        const double dx{ x_centre - cx };
        const double dy{ y_centre - cy };
        const double distance{ std::hypot(dx, dy) };
        inside = distance <= r;
    }
    return inside;
}

std::string HeatedPatch::describe() const {
    std::string description{ "no heated patch" };
    if(disc_) {
        const double diameter_m{ disc_->diameter.in(au::meters) };
        const double north_m{ disc_->north.in(au::meters) };
        const std::uint32_t layer_count{ disc_->layers };
        const std::uint32_t factor{ surface_layer_factor };
        constexpr std::size_t text_capacity{ 160u };
        char text[text_capacity]{};
        int length{ 0 };
        if(disc_->mode == PatchMode::SURFACE_LAYER) {
            length = std::snprintf(text, sizeof(text), "heated patch %.0f m wide, %.0f m north of the middle, %ux flux in the surface layer", diameter_m, north_m,
                                   factor);
        } else {
            length = std::snprintf(text, sizeof(text), "heated patch %.0f m wide, %.0f m north of the middle, %ux flux over %u heated layers", diameter_m, north_m,
                                   layer_count, layer_count);
        }
        const bool cut{ length < 0 || static_cast<std::size_t>(length) >= text_capacity };
        if(cut) {
            throw SetupError("HeatedPatch::describe(): the text does not fit its buffer");
        }
        description = text;
    }
    return description;
}
