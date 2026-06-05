#ifndef slic3r_FullColorRasterPipeline_hpp_
#define slic3r_FullColorRasterPipeline_hpp_

#include <cstddef>
#include <string>

namespace Slic3r {

class DynamicPrintConfig;
class Model;
class Print;
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

struct RasterPipelineOutput
{
    RasterPipelineSummary summary;
    std::string           output_dir;
    std::string           manifest_path;
    size_t                layer_count = 0;
    bool                  generated   = false;
};

RasterPipelineSummary collect_full_color_raster_summary(
    const Model &model,
    const DynamicPrintConfig &config);

RasterPipelineSummary collect_full_color_raster_summary(
    const Model &model,
    const PrintConfig &config);

RasterPipelineOutput generate_full_color_rasters(
    const Print &print,
    const std::string &gcode_path);

} // namespace FullColor
} // namespace Slic3r

#endif // slic3r_FullColorRasterPipeline_hpp_
