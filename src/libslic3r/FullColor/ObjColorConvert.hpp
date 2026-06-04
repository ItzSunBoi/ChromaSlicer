#pragma once

#include <memory>
#include <string>

namespace Slic3r {

struct ObjInfo;

namespace FullColor {

struct SurfaceData;

std::shared_ptr<SurfaceData> build_surface_data_from_obj_info(
    const std::string &obj_path,
    const ObjInfo     &obj_info);

} // namespace FullColor
} // namespace Slic3r