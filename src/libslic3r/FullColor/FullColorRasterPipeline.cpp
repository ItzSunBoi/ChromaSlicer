#include "FullColorRasterPipeline.hpp"

#include "FullColorSurfaceData.hpp"
#include "../Model.hpp"
#include "../PrintConfig.hpp"

#include <boost/log/trivial.hpp>
#include <memory>

namespace Slic3r::FullColor {

static RasterPipelineSummary collect_full_color_raster_summary(const Model &model, bool enabled)
{
    RasterPipelineSummary summary;
    summary.enabled = enabled;

    if (!summary.enabled) {
        BOOST_LOG_TRIVIAL(info) << "FullColor Raster Pipeline: disabled";
        return summary;
    }

    BOOST_LOG_TRIVIAL(info) << "FullColor Raster Pipeline: enabled";

    for (const ModelObject *object : model.objects) {
        if (object == nullptr)
            continue;

        ++summary.object_count;

        for (const ModelVolume *volume : object->volumes) {
            if (volume == nullptr)
                continue;

            ++summary.volume_count;

            const std::shared_ptr<SurfaceData> &surface_data = volume->full_color_data;
            if (!surface_data || surface_data->empty())
                continue;

            if (surface_data->has_textures || !surface_data->textures.empty())
                ++summary.textured_volume_count;

            summary.total_color_triangles += surface_data->triangles.size();
            summary.total_textures += surface_data->textures.size();
        }
    }

    BOOST_LOG_TRIVIAL(info) << "FullColor Raster Pipeline: textured_volumes=" << summary.textured_volume_count
                            << ", color_triangles=" << summary.total_color_triangles
                            << ", textures=" << summary.total_textures;
    BOOST_LOG_TRIVIAL(info) << "FullColor Raster Pipeline: raster generation not implemented in this stage";

    return summary;
}

RasterPipelineSummary collect_full_color_raster_summary(const Model &model, const DynamicPrintConfig &config)
{
    const bool enabled = config.has("enable_full_color_printing") && config.opt_bool("enable_full_color_printing");
    return collect_full_color_raster_summary(model, enabled);
}

RasterPipelineSummary collect_full_color_raster_summary(const Model &model, const PrintConfig &config)
{
    return collect_full_color_raster_summary(model, config.enable_full_color_printing.value);
}

} // namespace Slic3r::FullColor
