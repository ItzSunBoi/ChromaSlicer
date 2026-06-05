#include "FullColorRasterPipeline.hpp"

#include "FullColorSurfaceData.hpp"
#include "../Layer.hpp"
#include "../Model.hpp"
#include "../PNGReadWrite.hpp"
#include "../Print.hpp"
#include "../PrintConfig.hpp"
#include "../TriangleMesh.hpp"

#include <boost/filesystem.hpp>
#include <boost/log/trivial.hpp>
#include <boost/nowide/fstream.hpp>
#include "nlohmann/json.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <memory>
#include <sstream>
#include <unordered_map>

namespace Slic3r::FullColor {

namespace {

using json = nlohmann::json;

static constexpr double FULL_COLOR_PIXEL_SIZE_MM = 0.0846667;
static constexpr double FULL_COLOR_NORMAL_TOLERANCE_MM = 0.02;

struct TextureBitmap
{
    size_t width = 0;
    size_t height = 0;
    int bytes_per_pixel = 0;
    std::vector<std::uint8_t> pixels;
};

struct SurfaceSample
{
    Vec3d pos;
    Vec3d normal;
    RGBA color;
};

struct LayerRasterInfo
{
    size_t index = 0;
    double z0 = 0.;
    double z1 = 0.;
    double sample_z = 0.;
};

struct GridKey
{
    int x = 0;
    int y = 0;
    int z = 0;

    bool operator==(const GridKey &other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct GridKeyHash
{
    size_t operator()(const GridKey &key) const
    {
        size_t seed = 0;
        auto combine = [&seed](int value) {
            seed ^= std::hash<int>{}(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        };
        combine(key.x);
        combine(key.y);
        combine(key.z);
        return seed;
    }
};

struct SurfaceSampleGrid
{
    double cell_size = 1.0;
    std::unordered_map<GridKey, std::vector<const SurfaceSample*>, GridKeyHash> cells;
};

static std::uint8_t to_u8(float value)
{
    return static_cast<std::uint8_t>(std::clamp(value, 0.0f, 1.0f) * 255.0f + 0.5f);
}

static bool load_texture_bitmap(const TextureImage &texture, TextureBitmap &bitmap)
{
    if (texture.source_path.empty())
        return false;

    boost::nowide::ifstream in(texture.source_path.string(), std::ios::binary);
    if (!in) {
        BOOST_LOG_TRIVIAL(warning) << "FullColor Raster: failed to open texture '" << texture.source_path.string() << "'";
        return false;
    }

    std::vector<char> encoded((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    if (encoded.empty())
        return false;

    png::ImageColorscale image;
    png::ReadBuf read_buf{encoded.data(), encoded.size()};
    if (!png::decode_colored_png(read_buf, image)) {
        BOOST_LOG_TRIVIAL(warning) << "FullColor Raster: failed to decode texture '" << texture.source_path.string() << "'";
        return false;
    }

    bitmap.width = image.cols;
    bitmap.height = image.rows;
    bitmap.bytes_per_pixel = image.bytes_per_pixel;
    bitmap.pixels = std::move(image.buf);
    return bitmap.width > 0 && bitmap.height > 0 && bitmap.bytes_per_pixel >= 3;
}

static RGBA sample_texture(const TextureBitmap &bitmap, const Vec2f &uv, const RGBA &fallback)
{
    if (bitmap.pixels.empty() || bitmap.width == 0 || bitmap.height == 0 || bitmap.bytes_per_pixel < 3)
        return fallback;

    const float u = uv.x() - std::floor(uv.x());
    const float v = uv.y() - std::floor(uv.y());
    const size_t x = std::min(bitmap.width - 1, static_cast<size_t>(std::round(u * (bitmap.width - 1))));
    const size_t y = std::min(bitmap.height - 1, static_cast<size_t>(std::round(v * (bitmap.height - 1))));
    const size_t idx = (y * bitmap.width + x) * static_cast<size_t>(bitmap.bytes_per_pixel);

    if (idx + 2 >= bitmap.pixels.size())
        return fallback;

    return RGBA{
        bitmap.pixels[idx + 0] / 255.0f,
        bitmap.pixels[idx + 1] / 255.0f,
        bitmap.pixels[idx + 2] / 255.0f,
        bitmap.bytes_per_pixel >= 4 && idx + 3 < bitmap.pixels.size() ? bitmap.pixels[idx + 3] / 255.0f : fallback[3]};
}

static Vec3d transform_vertex(const Transform3d &transform, const Vec3f &v)
{
    return transform * v.cast<double>();
}

static void add_surface_samples_for_triangle(
    const Vec3d &p0,
    const Vec3d &p1,
    const Vec3d &p2,
    const TriangleColorData &triangle_data,
    const std::vector<TextureBitmap> &textures,
    std::vector<SurfaceSample> &samples)
{
    const Vec3d n_raw = (p1 - p0).cross(p2 - p0);
    if (n_raw.squaredNorm() <= 1e-12)
        return;

    const Vec3d normal = n_raw.normalized();
    const double max_edge = std::max({(p1 - p0).norm(), (p2 - p1).norm(), (p0 - p2).norm()});
    const int divisions = std::clamp(static_cast<int>(std::ceil(max_edge / (FULL_COLOR_PIXEL_SIZE_MM * 2.0))), 1, 16);

    const TextureBitmap *texture = nullptr;
    if (triangle_data.has_uv && triangle_data.texture_index >= 0 &&
        static_cast<size_t>(triangle_data.texture_index) < textures.size() &&
        !textures[triangle_data.texture_index].pixels.empty())
        texture = &textures[triangle_data.texture_index];

    for (int a = 0; a <= divisions; ++a) {
        for (int b = 0; b <= divisions - a; ++b) {
            const double w0 = static_cast<double>(a) / divisions;
            const double w1 = static_cast<double>(b) / divisions;
            const double w2 = 1.0 - w0 - w1;
            const Vec3d pos = p0 * w0 + p1 * w1 + p2 * w2;

            RGBA color = triangle_data.fallback_color;
            if (texture != nullptr) {
                const Vec2f uv = triangle_data.uv[0] * static_cast<float>(w0) +
                                 triangle_data.uv[1] * static_cast<float>(w1) +
                                 triangle_data.uv[2] * static_cast<float>(w2);
                color = sample_texture(*texture, uv, color);
            }

            samples.push_back({pos, normal, color});
        }
    }
}

static std::vector<LayerRasterInfo> collect_layers(const Print &print)
{
    std::vector<LayerRasterInfo> layers;
    for (const PrintObject *object : print.objects()) {
        if (object == nullptr)
            continue;

        for (const Layer *layer : object->layers()) {
            if (layer == nullptr)
                continue;

            const double z1 = layer->print_z;
            auto it = std::find_if(layers.begin(), layers.end(), [z1](const LayerRasterInfo &other) {
                return std::abs(other.z1 - z1) < 1e-5;
            });
            if (it != layers.end())
                continue;

            layers.push_back({0, layer->bottom_z(), z1, layer->slice_z});
        }
    }

    std::sort(layers.begin(), layers.end(), [](const LayerRasterInfo &a, const LayerRasterInfo &b) {
        return a.z1 < b.z1;
    });
    for (size_t idx = 0; idx < layers.size(); ++idx)
        layers[idx].index = idx;
    return layers;
}

static GridKey grid_key(const Vec3d &pos, double cell_size)
{
    return {
        static_cast<int>(std::floor(pos.x() / cell_size)),
        static_cast<int>(std::floor(pos.y() / cell_size)),
        static_cast<int>(std::floor(pos.z() / cell_size))
    };
}

static SurfaceSampleGrid build_surface_sample_grid(const std::vector<SurfaceSample> &samples, double shell_thickness)
{
    SurfaceSampleGrid grid;
    grid.cell_size = std::max(FULL_COLOR_PIXEL_SIZE_MM * 4.0, shell_thickness);
    grid.cells.reserve(samples.size());
    for (const SurfaceSample &sample : samples)
        grid.cells[grid_key(sample.pos, grid.cell_size)].push_back(&sample);
    return grid;
}

static const SurfaceSample *find_best_sample(
    const SurfaceSampleGrid &grid,
    const Vec3d &query,
    double shell_thickness)
{
    const GridKey center = grid_key(query, grid.cell_size);
    const int search_radius = std::max(1, static_cast<int>(std::ceil(shell_thickness / grid.cell_size)) + 1);
    const double shell_sq = shell_thickness * shell_thickness;

    const SurfaceSample *best = nullptr;
    double best_dist_sq = shell_sq;
    for (int dz = -search_radius; dz <= search_radius; ++dz) {
        for (int dy = -search_radius; dy <= search_radius; ++dy) {
            for (int dx = -search_radius; dx <= search_radius; ++dx) {
                const auto cell_it = grid.cells.find({center.x + dx, center.y + dy, center.z + dz});
                if (cell_it == grid.cells.end())
                    continue;

                for (const SurfaceSample *sample : cell_it->second) {
                    const Vec3d delta = query - sample->pos;
                    const double normal_distance = delta.dot(sample->normal);
                    if (normal_distance > FULL_COLOR_NORMAL_TOLERANCE_MM)
                        continue;

                    const double dist_sq = delta.squaredNorm();
                    if (dist_sq <= best_dist_sq) {
                        best_dist_sq = dist_sq;
                        best = sample;
                    }
                }
            }
        }
    }

    return best;
}

static boost::filesystem::path full_color_output_dir(const std::string &gcode_path)
{
    const boost::filesystem::path gcode(gcode_path);
    if (!gcode_path.empty() && !gcode.stem().empty())
        return gcode.parent_path() / (gcode.stem().string() + "_full_color");

    const boost::filesystem::path fallback =
        boost::filesystem::temp_directory_path() / boost::filesystem::unique_path("orcaslicer_full_color_%%%%-%%%%-%%%%");
    BOOST_LOG_TRIVIAL(warning) << "FullColor Raster: no G-code output path available, using debug output=" << fallback.string();
    return fallback;
}

static void write_json_file(const boost::filesystem::path &path, const json &data)
{
    boost::nowide::ofstream out(path.string());
    out << data.dump(2);
}

static std::string layer_basename(size_t layer_index)
{
    std::ostringstream name;
    name << "L" << std::setw(6) << std::setfill('0') << layer_index;
    return name.str();
}

} // namespace

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

RasterPipelineOutput generate_full_color_rasters(const Print &print, const std::string &gcode_path)
{
    RasterPipelineOutput output;
    output.summary = collect_full_color_raster_summary(print.model(), print.config());

    if (!output.summary.enabled) {
        BOOST_LOG_TRIVIAL(info) << "FullColor Raster: disabled";
        return output;
    }

    if (output.summary.textured_volume_count == 0 && output.summary.total_color_triangles == 0) {
        BOOST_LOG_TRIVIAL(info) << "FullColor Raster: enabled, but no full-color volumes found";
        return output;
    }

    const double shell_thickness = std::max(0.01, print.config().full_color_shell_thickness.value);
    BOOST_LOG_TRIVIAL(info) << "FullColor Raster: generating layer rasters";
    BOOST_LOG_TRIVIAL(info) << "FullColor Raster: pixel_size=" << FULL_COLOR_PIXEL_SIZE_MM << " mm";
    BOOST_LOG_TRIVIAL(info) << "FullColor Raster: shell_thickness=" << shell_thickness << " mm";
    BOOST_LOG_TRIVIAL(info) << "FullColor Raster: shell_mode=normal-inward";

    std::vector<SurfaceSample> samples;
    samples.reserve(output.summary.total_color_triangles * 16);
    size_t loaded_texture_count = 0;

    for (const PrintObject *print_object : print.objects()) {
        if (print_object == nullptr || print_object->model_object() == nullptr)
            continue;

        const ModelObject *model_object = print_object->model_object();
        for (const PrintInstance &instance : print_object->instances()) {
            const Point instance_shift = instance.shift_without_plate_offset();
            Transform3d instance_transform = print_object->trafo_centered();
            instance_transform.pretranslate(Vec3d(unscale<double>(instance_shift.x()), unscale<double>(instance_shift.y()), 0.0));

            for (const ModelVolume *volume : model_object->volumes) {
                if (volume == nullptr || !volume->full_color_data || volume->full_color_data->empty())
                    continue;

                const SurfaceData &surface = *volume->full_color_data;
                std::vector<TextureBitmap> texture_bitmaps(surface.textures.size());
                for (size_t texture_idx = 0; texture_idx < surface.textures.size(); ++texture_idx)
                    if (load_texture_bitmap(surface.textures[texture_idx], texture_bitmaps[texture_idx]))
                        ++loaded_texture_count;

                const Transform3d transform = instance_transform * volume->get_matrix();
                const indexed_triangle_set &its = volume->mesh().its;
                const size_t triangle_count = std::min(surface.triangles.size(), its.indices.size());
                for (size_t triangle_idx = 0; triangle_idx < triangle_count; ++triangle_idx) {
                    const stl_triangle_vertex_indices face = its.indices[triangle_idx];
                    add_surface_samples_for_triangle(
                        transform_vertex(transform, its.vertices[face[0]]),
                        transform_vertex(transform, its.vertices[face[1]]),
                        transform_vertex(transform, its.vertices[face[2]]),
                        surface.triangles[triangle_idx],
                        texture_bitmaps,
                        samples);
                }
            }
        }
    }

    if (samples.empty()) {
        BOOST_LOG_TRIVIAL(info) << "FullColor Raster: enabled, but no CPU surface samples could be generated";
        return output;
    }

    Vec3d min_pt(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
    Vec3d max_pt(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest());
    for (const SurfaceSample &sample : samples) {
        min_pt = min_pt.cwiseMin(sample.pos);
        max_pt = max_pt.cwiseMax(sample.pos);
    }

    const int width = std::max(1, static_cast<int>(std::ceil((max_pt.x() - min_pt.x()) / FULL_COLOR_PIXEL_SIZE_MM)));
    const int height = std::max(1, static_cast<int>(std::ceil((max_pt.y() - min_pt.y()) / FULL_COLOR_PIXEL_SIZE_MM)));
    const std::vector<LayerRasterInfo> layers = collect_layers(print);
    if (layers.empty()) {
        BOOST_LOG_TRIVIAL(info) << "FullColor Raster: enabled, but no print layers are available";
        return output;
    }

    const SurfaceSampleGrid sample_grid = build_surface_sample_grid(samples, shell_thickness);
    BOOST_LOG_TRIVIAL(info) << "FullColor Raster: sample_count=" << samples.size()
                            << ", grid_cells=" << sample_grid.cells.size()
                            << ", grid_cell_size=" << sample_grid.cell_size << " mm";

    const boost::filesystem::path output_dir = full_color_output_dir(gcode_path);
    const boost::filesystem::path layer_dir = output_dir / "layers";
    boost::filesystem::create_directories(layer_dir);

    BOOST_LOG_TRIVIAL(info) << "FullColor Raster: output=" << output_dir.string();
    BOOST_LOG_TRIVIAL(info) << "FullColor Raster: layer_count=" << layers.size();

    json manifest;
    manifest["format"] = "OrcaSlicerFullColorRaster";
    manifest["version"] = 1;
    manifest["raster_strategy"] = "surface-cloud";
    manifest["shell_mode"] = "normal-inward";
    manifest["preview_scaffold"] = "Show full-color layers overlay is scaffolded; generated layer image paths are tracked in this manifest.";
    manifest["settings"] = {
        {"pixel_size_mm", FULL_COLOR_PIXEL_SIZE_MM},
        {"shell_thickness_mm", shell_thickness},
        {"normal_tolerance_mm", FULL_COLOR_NORMAL_TOLERANCE_MM}
    };
    manifest["model_bounds"] = {
        {"min", {min_pt.x(), min_pt.y(), min_pt.z()}},
        {"max", {max_pt.x(), max_pt.y(), max_pt.z()}},
        {"image_width", width},
        {"image_height", height},
        {"origin_xy_mm", {min_pt.x(), min_pt.y()}},
        {"coordinate_origin", "min model bounds in print-bed XY, units mm"},
        {"image_row_0_y_mm", min_pt.y()},
        {"image_y_flipped", false},
        {"background", "white"}
    };
    manifest["full_color_volumes"] = output.summary.textured_volume_count;
    manifest["textures"] = output.summary.total_textures;
    manifest["loaded_textures"] = loaded_texture_count;
    manifest["layers"] = json::array();

    for (const LayerRasterInfo &layer : layers) {
        std::vector<std::uint8_t> image(static_cast<size_t>(width) * static_cast<size_t>(height) * 3, 255);
        for (int y = 0; y < height; ++y) {
            const double py = min_pt.y() + (y + 0.5) * FULL_COLOR_PIXEL_SIZE_MM;
            for (int x = 0; x < width; ++x) {
                const Vec3d query(
                    min_pt.x() + (x + 0.5) * FULL_COLOR_PIXEL_SIZE_MM,
                    py,
                    layer.sample_z);

                const SurfaceSample *best = find_best_sample(sample_grid, query, shell_thickness);

                if (best != nullptr) {
                    const size_t idx = (static_cast<size_t>(y) * width + x) * 3;
                    image[idx + 0] = to_u8(best->color[0]);
                    image[idx + 1] = to_u8(best->color[1]);
                    image[idx + 2] = to_u8(best->color[2]);
                }
            }
        }

        const std::string base = layer_basename(layer.index);
        const boost::filesystem::path png_path = layer_dir / (base + ".png");
        const boost::filesystem::path json_path = layer_dir / (base + ".json");
        BOOST_LOG_TRIVIAL(info) << "FullColor Raster: writing " << png_path.filename().string();
        png::write_rgb_to_file(png_path.string(), width, height, image);

        json layer_json;
        layer_json["layer_index"] = layer.index;
        layer_json["z0"] = layer.z0;
        layer_json["z1"] = layer.z1;
        layer_json["sample_z"] = layer.sample_z;
        layer_json["shell_thickness_mm"] = shell_thickness;
        layer_json["pixel_size_mm"] = FULL_COLOR_PIXEL_SIZE_MM;
        layer_json["image_width"] = width;
        layer_json["image_height"] = height;
        layer_json["origin_xy_mm"] = {min_pt.x(), min_pt.y()};
        layer_json["image_row_0_y_mm"] = min_pt.y();
        layer_json["image_y_flipped"] = false;
        layer_json["background"] = "white";
        layer_json["units"] = "mm";
        layer_json["coordinate_convention"] = "print-bed XY in mm; sample_z uses actual PrintObject layer slice_z";
        layer_json["image"] = png_path.filename().string();
        write_json_file(json_path, layer_json);

        manifest["layers"].push_back({
            {"layer_index", layer.index},
            {"z0", layer.z0},
            {"z1", layer.z1},
            {"sample_z", layer.sample_z},
            {"png", (boost::filesystem::path("layers") / png_path.filename()).string()},
            {"json", (boost::filesystem::path("layers") / json_path.filename()).string()}
        });
    }

    const boost::filesystem::path manifest_path = output_dir / "manifest.json";
    manifest["layer_count"] = layers.size();
    manifest["future_format_support_todo"] = {
        "3MF texture/color support",
        "glTF/GLB textured mesh support",
        "PLY vertex colors",
        "VRML/X3D if useful",
        "OBJ+MTL+texture is the currently proven MVP path"
    };
    write_json_file(manifest_path, manifest);

    output.output_dir = output_dir.string();
    output.manifest_path = manifest_path.string();
    output.layer_count = layers.size();
    output.generated = true;

    BOOST_LOG_TRIVIAL(info) << "FullColor Preview: Show full-color layers scaffold manifest=" << output.manifest_path;
    if (!layers.empty())
        BOOST_LOG_TRIVIAL(info) << "FullColor Preview: current color layer image scaffold=" << (layer_dir / (layer_basename(0) + ".png")).string();

    return output;
}

} // namespace Slic3r::FullColor
