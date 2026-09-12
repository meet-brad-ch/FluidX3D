#include "setup/graphics/camera_view.hpp"
#include "setup/core/setup_error.hpp"

#include <cmath>

CameraPose CameraView::pose(const uint3& cells, float cell_size) const {
    CameraPose pose;
    pose.azimuth = azimuth_.deg();
    pose.elevation = elevation_.deg();
    pose.field_of_view = field_of_view_.deg();
    const float3 N((float)cells.x, (float)cells.y, (float)cells.z);
    if(!position_) {
        if(target_) throw SetupError("CameraView::look_at() is for CameraView::at(); an orbit() camera looks at the domain's center");
        // the core's zoom shows 0.5*max(N)/zoom cells on each side of the center across the frame's smaller side
        if(view_height_) pose.zoom = fmax(fmax(N.x, N.y), N.z) * cell_size / view_height_->si();
        return pose;
    }
    if(view_height_) throw SetupError("CameraView::view_height() is for CameraView::orbit(); move an at() camera or narrow its field_of_view()");
    const float3 position(position_->x.si(), position_->y.si(), position_->z.si());
    pose.free = true;
    pose.position = position / cell_size - 0.5f * N;
    if(target_) {
        const float3 d = position - float3(target_->x.si(), target_->y.si(), target_->z.si()); // from the target to the camera
        pose.azimuth = degrees(std::atan2(d.y, d.x));
        pose.elevation = degrees(std::atan2(d.z, std::sqrt(d.x * d.x + d.y * d.y)));
    }
    return pose;
}
