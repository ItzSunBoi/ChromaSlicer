#include "FullColorSurfaceData.hpp"

namespace Slic3r::FullColor {

bool SurfaceData::empty() const
{
    return vertex_colors.empty() && triangles.empty() && textures.empty();
}

} // namespace Slic3r::FullColor