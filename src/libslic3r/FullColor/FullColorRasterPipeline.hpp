#ifndef slic3r_FullColorRasterPipeline_hpp_
#define slic3r_FullColorRasterPipeline_hpp_

#include <cstddef>

namespace Slic3r {

class DynamicPrintConfig;
class Model;
class PrintConfig;

namespace FullColor {

struct RasterPipelineSummary
{
    bool   enabled               = false;
    size_t object_count          = 0;
    size_t volume_count          = 0;
    size_t textured_volume_count = 0;
    size_t total_color_triangles = 0;
    size_t total_textures        = 0;
};

RasterPipelineSummary collect_full_color_raster_summary(
    const Model &model,
    const DynamicPrintConfig &config);

RasterPipelineSummary collect_full_color_raster_summary(
    const Model &model,
    const PrintConfig &config);

} // namespace FullColor
} // namespace Slic3r

#endif // slic3r_FullColorRasterPipeline_hpp_
