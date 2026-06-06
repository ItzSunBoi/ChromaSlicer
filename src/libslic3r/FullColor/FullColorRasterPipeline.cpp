#include "FullColorRasterPipeline.hpp"

#include "FullColorSurfaceData.hpp"
#include "../ClipperUtils.hpp"
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

#include <zlib.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <functional>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <cstdint>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <unordered_map>
#include <vector>

namespace Slic3r::FullColor {

namespace {

using json = nlohmann::json;
using RasterClock = std::chrono::steady_clock;

static constexpr double FULL_COLOR_PIXEL_SIZE_MM        = 0.0846667; // ~300 DPI
static constexpr double FULL_COLOR_NORMAL_TOLERANCE_MM = 0.02;
static constexpr double FULL_COLOR_FDM_MASK_BLEED_MM   = FULL_COLOR_PIXEL_SIZE_MM * 1.5;
static constexpr bool   FULL_COLOR_OBJ_FLIP_V          = true;
static constexpr bool   FULL_COLOR_CPU_TEXTURE_FLIP_V  = false;
static constexpr const char *FULL_COLOR_TEXTURE_ADDRESS_MODE = "clamp_to_edge";
static constexpr const char *FULL_COLOR_TEXTURE_BUFFER_ORIGIN = "bottom-left";

enum class RasterDebugMode
{
    Normal,
    UV,
    Triangle
};

struct TextureBitmap
{
    size_t width = 0;
    size_t height = 0;
    int bytes_per_pixel = 0;
    std::vector<std::uint8_t> pixels;
};

struct RasterTriangle
{
    Vec3d p0;
    Vec3d p1;
    Vec3d p2;
    Vec3d min_pt;
    Vec3d max_pt;
    Vec3d normal;
    std::array<Vec2f, 3> uv;
    RGBA fallback_color;
    int texture_index = -1;
    bool has_uv = false;
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

struct TriangleGrid
{
    double cell_size = 1.0;
    std::unordered_map<GridKey, std::vector<size_t>, GridKeyHash> cells;
};

struct LayerDebugCounters
{
    size_t total_pixels = 0;
    size_t colored_pixels = 0;
    size_t candidate_misses = 0;
    size_t candidate_triangle_tests = 0;
    size_t accepted_triangle_hits = 0;
    size_t subpixel_edge_hits = 0;
    size_t vertical_projected_samples = 0;
    size_t closest_point_samples = 0;
    size_t rejected_by_xy_projection = 0;
    size_t rejected_by_shell_distance = 0;
    size_t rejected_by_outward_shell = 0;
    size_t rejected_by_fdm_mask = 0;
    size_t rejected_by_normal = 0;
    size_t invalid_uv_texture = 0;
};

struct LayerPixelBounds
{
    int min_x = 0;
    int max_x = -1;
    int min_y = 0;
    int max_y = -1;

    bool empty() const { return max_x < min_x || max_y < min_y; }
};

struct LayerRasterMask
{
    ExPolygons printable_area;
    std::vector<BoundingBox> bboxes;

    bool empty() const { return printable_area.empty(); }
};

struct LayerMaskRowIndex
{
    std::vector<std::vector<size_t>> rows;
};

struct RasterTiming
{
    double total_ms = 0.0;
    double surface_prep_ms = 0.0;
    double grid_build_ms = 0.0;
    double layer_raster_total_ms = 0.0;
    double layer_average_ms = 0.0;
    double slowest_layer_ms = 0.0;
    double staging_ms = 0.0;
    double manifest_ms = 0.0;
    double packaging_ms = 0.0;
};

static double elapsed_ms(const RasterClock::time_point &start, const RasterClock::time_point &end = RasterClock::now())
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

static void add_counters(LayerDebugCounters &dst, const LayerDebugCounters &src)
{
    dst.total_pixels += src.total_pixels;
    dst.colored_pixels += src.colored_pixels;
    dst.candidate_misses += src.candidate_misses;
    dst.candidate_triangle_tests += src.candidate_triangle_tests;
    dst.accepted_triangle_hits += src.accepted_triangle_hits;
    dst.subpixel_edge_hits += src.subpixel_edge_hits;
    dst.vertical_projected_samples += src.vertical_projected_samples;
    dst.closest_point_samples += src.closest_point_samples;
    dst.rejected_by_xy_projection += src.rejected_by_xy_projection;
    dst.rejected_by_shell_distance += src.rejected_by_shell_distance;
    dst.rejected_by_outward_shell += src.rejected_by_outward_shell;
    dst.rejected_by_fdm_mask += src.rejected_by_fdm_mask;
    dst.rejected_by_normal += src.rejected_by_normal;
    dst.invalid_uv_texture += src.invalid_uv_texture;
}

static RasterDebugMode raster_debug_mode()
{
    const char *env = std::getenv("ORCA_FULL_COLOR_RASTER_DEBUG");
    if (env == nullptr)
        return RasterDebugMode::Normal;

    const std::string value(env);
    if (value == "uv" || value == "UV")
        return RasterDebugMode::UV;
    if (value == "triangle" || value == "triangles")
        return RasterDebugMode::Triangle;
    return RasterDebugMode::Normal;
}

static const char *raster_debug_mode_name(RasterDebugMode mode)
{
    switch (mode) {
    case RasterDebugMode::UV:       return "uv";
    case RasterDebugMode::Triangle: return "triangle";
    case RasterDebugMode::Normal:
    default:                        return "normal";
    }
}

static const char *raster_debug_mode_manifest_name(RasterDebugMode mode)
{
    return mode == RasterDebugMode::Normal ? "none" : raster_debug_mode_name(mode);
}

static std::uint8_t to_u8(float value)
{
    return static_cast<std::uint8_t>(std::clamp(value, 0.0f, 1.0f) * 255.0f + 0.5f);
}

static RGBA triangle_debug_color(size_t triangle_index)
{
    const uint32_t h = static_cast<uint32_t>(triangle_index * 2654435761u);
    return RGBA{
        ((h >> 16) & 0xff) / 255.0f,
        ((h >> 8) & 0xff) / 255.0f,
        (h & 0xff) / 255.0f,
        1.0f
    };
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
    if (encoded.empty()) {
        BOOST_LOG_TRIVIAL(warning) << "FullColor Raster: texture file is empty '" << texture.source_path.string() << "'";
        return false;
    }

    png::ImageColorscale image;
    png::ReadBuf read_buf{encoded.data(), encoded.size()};
    if (!png::decode_colored_png(read_buf, image)) {
        BOOST_LOG_TRIVIAL(warning) << "FullColor Raster: failed to decode PNG texture '" << texture.source_path.string() << "'";
        return false;
    }

    bitmap.width = image.cols;
    bitmap.height = image.rows;
    bitmap.bytes_per_pixel = image.bytes_per_pixel;
    bitmap.pixels = std::move(image.buf);

    const bool ok = bitmap.width > 0 && bitmap.height > 0 && bitmap.bytes_per_pixel >= 3 &&
                    bitmap.pixels.size() >= bitmap.width * bitmap.height * static_cast<size_t>(bitmap.bytes_per_pixel);

    if (ok) {
        BOOST_LOG_TRIVIAL(info) << "FullColor Raster: loaded texture '" << texture.source_path.string()
                                << "' " << bitmap.width << "x" << bitmap.height
                                << " bpp=" << bitmap.bytes_per_pixel;
    }

    return ok;
}

static float texel_channel(const TextureBitmap &bitmap, size_t x, size_t y, int channel, const RGBA &fallback)
{
    const size_t idx = (y * bitmap.width + x) * static_cast<size_t>(bitmap.bytes_per_pixel);
    if (channel < bitmap.bytes_per_pixel && idx + static_cast<size_t>(channel) < bitmap.pixels.size())
        return bitmap.pixels[idx + static_cast<size_t>(channel)] / 255.0f;
    return fallback[channel];
}

static RGBA sample_texture_bilinear(const TextureBitmap &bitmap, const Vec2f &uv, const RGBA &fallback)
{
    if (bitmap.pixels.empty() || bitmap.width == 0 || bitmap.height == 0 || bitmap.bytes_per_pixel < 3)
        return fallback;

    double u = static_cast<double>(uv.x());
    double v = static_cast<double>(uv.y());

    u = std::clamp(u, 0.0, 1.0);
    v = std::clamp(v, 0.0, 1.0);

    // PNGReadWrite stores colored PNG rows bottom-up for historical slicer preview code.
    // That already matches OBJ's bottom-left V convention, so the CPU sampler must not
    // apply the viewport's GL upload flip a second time.
    if (FULL_COLOR_CPU_TEXTURE_FLIP_V)
        v = 1.0 - v;

    const double fx = u * static_cast<double>(bitmap.width  - 1);
    const double fy = v * static_cast<double>(bitmap.height - 1);

    const size_t x0 = std::min(bitmap.width  - 1, static_cast<size_t>(std::floor(fx)));
    const size_t y0 = std::min(bitmap.height - 1, static_cast<size_t>(std::floor(fy)));
    const size_t x1 = std::min(bitmap.width  - 1, x0 + 1);
    const size_t y1 = std::min(bitmap.height - 1, y0 + 1);

    const float tx = static_cast<float>(fx - std::floor(fx));
    const float ty = static_cast<float>(fy - std::floor(fy));

    float out[4] = {0.f, 0.f, 0.f, fallback[3]};
    for (int c = 0; c < 4; ++c) {
        const float c00 = texel_channel(bitmap, x0, y0, c, fallback);
        const float c10 = texel_channel(bitmap, x1, y0, c, fallback);
        const float c01 = texel_channel(bitmap, x0, y1, c, fallback);
        const float c11 = texel_channel(bitmap, x1, y1, c, fallback);
        const float cx0 = c00 * (1.0f - tx) + c10 * tx;
        const float cx1 = c01 * (1.0f - tx) + c11 * tx;
        out[c] = cx0 * (1.0f - ty) + cx1 * ty;
    }

    return RGBA{out[0], out[1], out[2], out[3]};
}

static Vec3d transform_vertex(const Transform3d &transform, const Vec3f &v)
{
    return transform * v.cast<double>();
}

static Vec3d closest_point_on_triangle(const Vec3d &p, const Vec3d &a, const Vec3d &b, const Vec3d &c)
{
    // Real-Time Collision Detection, Christer Ericson, closest point on triangle.
    const Vec3d ab = b - a;
    const Vec3d ac = c - a;
    const Vec3d ap = p - a;
    const double d1 = ab.dot(ap);
    const double d2 = ac.dot(ap);
    if (d1 <= 0.0 && d2 <= 0.0)
        return a;

    const Vec3d bp = p - b;
    const double d3 = ab.dot(bp);
    const double d4 = ac.dot(bp);
    if (d3 >= 0.0 && d4 <= d3)
        return b;

    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        const double v = d1 / (d1 - d3);
        return a + v * ab;
    }

    const Vec3d cp = p - c;
    const double d5 = ab.dot(cp);
    const double d6 = ac.dot(cp);
    if (d6 >= 0.0 && d5 <= d6)
        return c;

    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        const double w = d2 / (d2 - d6);
        return a + w * ac;
    }

    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
        const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + w * (c - b);
    }

    const double denom = 1.0 / (va + vb + vc);
    const double v = vb * denom;
    const double w = vc * denom;
    return a + ab * v + ac * w;
}

static Vec3d barycentric_coords(const Vec3d &p, const Vec3d &a, const Vec3d &b, const Vec3d &c)
{
    const Vec3d v0 = b - a;
    const Vec3d v1 = c - a;
    const Vec3d v2 = p - a;
    const double d00 = v0.dot(v0);
    const double d01 = v0.dot(v1);
    const double d11 = v1.dot(v1);
    const double d20 = v2.dot(v0);
    const double d21 = v2.dot(v1);
    const double denom = d00 * d11 - d01 * d01;

    if (std::abs(denom) <= 1e-20)
        return Vec3d(1.0, 0.0, 0.0);

    const double v = (d11 * d20 - d01 * d21) / denom;
    const double w = (d00 * d21 - d01 * d20) / denom;
    const double u = 1.0 - v - w;
    return Vec3d(u, v, w);
}

static bool barycentric_coords_xy(const Vec3d &p, const Vec3d &a, const Vec3d &b, const Vec3d &c, Vec3d &out)
{
    const double v0x = b.x() - a.x();
    const double v0y = b.y() - a.y();
    const double v1x = c.x() - a.x();
    const double v1y = c.y() - a.y();
    const double v2x = p.x() - a.x();
    const double v2y = p.y() - a.y();
    const double denom = v0x * v1y - v1x * v0y;

    if (std::abs(denom) <= 1e-12)
        return false;

    const double v = (v2x * v1y - v1x * v2y) / denom;
    const double w = (v0x * v2y - v2x * v0y) / denom;
    const double u = 1.0 - v - w;
    out = Vec3d(u, v, w);
    return true;
}

static bool barycentric_inside_triangle(const Vec3d &bary)
{
    static constexpr double eps = 1e-6;
    return bary.x() >= -eps && bary.y() >= -eps && bary.z() >= -eps &&
           bary.x() <= 1.0 + eps && bary.y() <= 1.0 + eps && bary.z() <= 1.0 + eps;
}

static Vec3d point_from_barycentric(const Vec3d &bary, const Vec3d &a, const Vec3d &b, const Vec3d &c)
{
    return a * bary.x() + b * bary.y() + c * bary.z();
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

static TriangleGrid build_triangle_grid(const std::vector<RasterTriangle> &triangles, double shell_thickness)
{
    TriangleGrid grid;
    grid.cell_size = std::max({shell_thickness, FULL_COLOR_PIXEL_SIZE_MM * 16.0, 0.5});

    for (size_t i = 0; i < triangles.size(); ++i) {
        const RasterTriangle &tri = triangles[i];
        Vec3d min_pt = tri.p0.cwiseMin(tri.p1).cwiseMin(tri.p2) - Vec3d(shell_thickness, shell_thickness, shell_thickness);
        Vec3d max_pt = tri.p0.cwiseMax(tri.p1).cwiseMax(tri.p2) + Vec3d(shell_thickness, shell_thickness, shell_thickness);

        const GridKey min_key = grid_key(min_pt, grid.cell_size);
        const GridKey max_key = grid_key(max_pt, grid.cell_size);

        for (int z = min_key.z; z <= max_key.z; ++z)
            for (int y = min_key.y; y <= max_key.y; ++y)
                for (int x = min_key.x; x <= max_key.x; ++x)
                    grid.cells[{x, y, z}].push_back(i);
    }

    return grid;
}

static LayerPixelBounds layer_pixel_bounds(
    const std::vector<RasterTriangle> &triangles,
    const Vec3d &min_pt,
    int width,
    int height,
    double z,
    double shell_thickness)
{
    LayerPixelBounds bounds;
    bounds.min_x = width;
    bounds.min_y = height;
    bounds.max_x = -1;
    bounds.max_y = -1;

    for (const RasterTriangle &tri : triangles) {
        if (z < tri.min_pt.z() - shell_thickness || z > tri.max_pt.z() + shell_thickness)
            continue;

        const double min_x = tri.min_pt.x() - shell_thickness;
        const double max_x = tri.max_pt.x() + shell_thickness;
        const double min_y = tri.min_pt.y() - shell_thickness;
        const double max_y = tri.max_pt.y() + shell_thickness;

        const int x0 = std::clamp(static_cast<int>(std::floor((min_x - min_pt.x()) / FULL_COLOR_PIXEL_SIZE_MM)) - 1, 0, width - 1);
        const int x1 = std::clamp(static_cast<int>(std::ceil ((max_x - min_pt.x()) / FULL_COLOR_PIXEL_SIZE_MM)) + 1, 0, width - 1);
        const int y0 = std::clamp(static_cast<int>(std::floor((min_y - min_pt.y()) / FULL_COLOR_PIXEL_SIZE_MM)) - 1, 0, height - 1);
        const int y1 = std::clamp(static_cast<int>(std::ceil ((max_y - min_pt.y()) / FULL_COLOR_PIXEL_SIZE_MM)) + 1, 0, height - 1);

        bounds.min_x = std::min(bounds.min_x, x0);
        bounds.max_x = std::max(bounds.max_x, x1);
        bounds.min_y = std::min(bounds.min_y, y0);
        bounds.max_y = std::max(bounds.max_y, y1);
    }

    return bounds;
}

static bool contains_point(const LayerRasterMask &mask, const Point &point)
{
    for (size_t idx = 0; idx < mask.printable_area.size(); ++idx) {
        if (idx < mask.bboxes.size() && !mask.bboxes[idx].contains(point))
            continue;
        if (mask.printable_area[idx].contains(point, true))
            return true;
    }
    return false;
}

static bool contains_point_indexed(const LayerRasterMask &mask, const LayerMaskRowIndex &row_index, int y, const Point &point)
{
    if (y < 0 || static_cast<size_t>(y) >= row_index.rows.size())
        return contains_point(mask, point);

    for (const size_t idx : row_index.rows[static_cast<size_t>(y)]) {
        if (idx >= mask.printable_area.size())
            continue;
        if (idx < mask.bboxes.size() && !mask.bboxes[idx].contains(point))
            continue;
        if (mask.printable_area[idx].contains(point, true))
            return true;
    }
    return false;
}

static void append_shifted_polygons(Polygons &dst, const Polygons &src, const Point &shift)
{
    const size_t start = dst.size();
    append(dst, src);
    for (size_t idx = start; idx < dst.size(); ++idx)
        dst[idx].translate(shift);
}

static LayerRasterMask build_layer_fdm_mask(const Print &print, const LayerRasterInfo &layer)
{
    Polygons covered;
    const float scaled_bleed = static_cast<float>(scale_(FULL_COLOR_FDM_MASK_BLEED_MM));

    for (const PrintObject *object : print.objects()) {
        if (object == nullptr)
            continue;

        const Layer *object_layer = object->get_layer_at_printz(layer.z1, 1e-5);
        if (object_layer == nullptr)
            continue;

        Polygons object_polygons;
        for (const LayerRegion *region : object_layer->regions()) {
            if (region == nullptr)
                continue;

            region->perimeters.polygons_covered_by_width(object_polygons, scaled_bleed);
            region->thin_fills.polygons_covered_by_width(object_polygons, scaled_bleed);
            region->fills.polygons_covered_by_spacing(object_polygons, scaled_bleed);
        }

        if (object_polygons.empty())
            continue;

        for (const PrintInstance &instance : object->instances())
            append_shifted_polygons(covered, object_polygons, instance.shift_without_plate_offset());
    }

    LayerRasterMask mask;
    if (!covered.empty())
        mask.printable_area = union_ex(covered);
    mask.bboxes.reserve(mask.printable_area.size());
    for (const ExPolygon &expoly : mask.printable_area)
        mask.bboxes.emplace_back(get_extents(expoly));
    return mask;
}

static LayerMaskRowIndex build_layer_mask_row_index(
    const LayerRasterMask &mask,
    const Vec3d &min_pt,
    int height)
{
    LayerMaskRowIndex index;
    index.rows.resize(static_cast<size_t>(height));
    if (mask.empty() || height <= 0)
        return index;

    for (size_t idx = 0; idx < mask.bboxes.size(); ++idx) {
        const BoundingBox &bbox = mask.bboxes[idx];
        const double min_y = unscale<double>(bbox.min.y());
        const double max_y = unscale<double>(bbox.max.y());

        const int y0 = std::clamp(static_cast<int>(std::floor((min_y - min_pt.y()) / FULL_COLOR_PIXEL_SIZE_MM)) - 1, 0, height - 1);
        const int y1 = std::clamp(static_cast<int>(std::ceil ((max_y - min_pt.y()) / FULL_COLOR_PIXEL_SIZE_MM)) + 1, 0, height - 1);

        for (int y = y0; y <= y1; ++y)
            index.rows[static_cast<size_t>(y)].push_back(idx);
    }

    return index;
}

static LayerPixelBounds layer_mask_pixel_bounds(
    const LayerRasterMask &mask,
    const Vec3d &min_pt,
    int width,
    int height)
{
    LayerPixelBounds bounds;
    bounds.min_x = width;
    bounds.min_y = height;
    bounds.max_x = -1;
    bounds.max_y = -1;

    for (const BoundingBox &bbox : mask.bboxes) {
        const double min_x = unscale<double>(bbox.min.x());
        const double max_x = unscale<double>(bbox.max.x());
        const double min_y = unscale<double>(bbox.min.y());
        const double max_y = unscale<double>(bbox.max.y());

        const int x0 = std::clamp(static_cast<int>(std::floor((min_x - min_pt.x()) / FULL_COLOR_PIXEL_SIZE_MM)) - 1, 0, width - 1);
        const int x1 = std::clamp(static_cast<int>(std::ceil ((max_x - min_pt.x()) / FULL_COLOR_PIXEL_SIZE_MM)) + 1, 0, width - 1);
        const int y0 = std::clamp(static_cast<int>(std::floor((min_y - min_pt.y()) / FULL_COLOR_PIXEL_SIZE_MM)) - 1, 0, height - 1);
        const int y1 = std::clamp(static_cast<int>(std::ceil ((max_y - min_pt.y()) / FULL_COLOR_PIXEL_SIZE_MM)) + 1, 0, height - 1);

        bounds.min_x = std::min(bounds.min_x, x0);
        bounds.max_x = std::max(bounds.max_x, x1);
        bounds.min_y = std::min(bounds.min_y, y0);
        bounds.max_y = std::max(bounds.max_y, y1);
    }

    return bounds;
}

static LayerPixelBounds intersect_bounds(const LayerPixelBounds &a, const LayerPixelBounds &b)
{
    if (a.empty() || b.empty())
        return {};

    LayerPixelBounds out;
    out.min_x = std::max(a.min_x, b.min_x);
    out.max_x = std::min(a.max_x, b.max_x);
    out.min_y = std::max(a.min_y, b.min_y);
    out.max_y = std::min(a.max_y, b.max_y);
    return out;
}

static bool sample_best_triangle(
    const std::vector<RasterTriangle> &triangles,
    const std::vector<TextureBitmap> &textures,
    const TriangleGrid &grid,
    const Vec3d &query,
    double shell_thickness,
    RasterDebugMode debug_mode,
    LayerDebugCounters &counters,
    RGBA &out_color)
{
    const GridKey key = grid_key(query, grid.cell_size);
    const auto it = grid.cells.find(key);
    if (it == grid.cells.end()) {
        ++counters.candidate_misses;
        return false;
    }

    const double shell_sq = shell_thickness * shell_thickness;
    double best_dist_sq = shell_sq;
    bool found = false;
    RGBA best_color{1.0f, 1.0f, 1.0f, 1.0f};

    for (const size_t tri_idx : it->second) {
        if (tri_idx >= triangles.size())
            continue;
        ++counters.candidate_triangle_tests;

        const RasterTriangle &tri = triangles[tri_idx];
        Vec3d bary;
        Vec3d sample;
        bool used_vertical_projection = false;

        const bool has_xy_projection = barycentric_coords_xy(query, tri.p0, tri.p1, tri.p2, bary);
        if (has_xy_projection && barycentric_inside_triangle(bary)) {
            sample = point_from_barycentric(bary, tri.p0, tri.p1, tri.p2);
            used_vertical_projection = true;
        } else {
            if (has_xy_projection)
                ++counters.rejected_by_xy_projection;
            sample = closest_point_on_triangle(query, tri.p0, tri.p1, tri.p2);
            bary = barycentric_coords(sample, tri.p0, tri.p1, tri.p2);
        }

        const Vec3d delta = query - sample;
        const double normal_distance = delta.dot(tri.normal);
        if (normal_distance > FULL_COLOR_NORMAL_TOLERANCE_MM) {
            ++counters.rejected_by_normal;
            continue;
        }

        const double inward_depth = -normal_distance;
        if (inward_depth > shell_thickness) {
            ++counters.rejected_by_shell_distance;
            continue;
        }

        // The color shell is only the material band inside the model surface.
        // Closest-point hits on triangle edges can otherwise color pixels outside
        // sharp corners because their offset is tangential to the triangle normal.
        if (!used_vertical_projection && inward_depth < FULL_COLOR_NORMAL_TOLERANCE_MM) {
            ++counters.rejected_by_outward_shell;
            continue;
        }

        const double dist_sq = delta.squaredNorm();
        if (dist_sq > shell_sq) {
            ++counters.rejected_by_shell_distance;
            continue;
        }
        if (dist_sq > best_dist_sq)
            continue;

        RGBA color = tri.fallback_color;
        const Vec2f uv = tri.uv[0] * static_cast<float>(bary.x()) +
                         tri.uv[1] * static_cast<float>(bary.y()) +
                         tri.uv[2] * static_cast<float>(bary.z());

        if (debug_mode == RasterDebugMode::UV) {
            if (!tri.has_uv) {
                ++counters.invalid_uv_texture;
                continue;
            }
            const float u = std::clamp(uv.x(), 0.0f, 1.0f);
            const float v = std::clamp(uv.y(), 0.0f, 1.0f);
            color = RGBA{
                u,
                v,
                0.0f,
                1.0f
            };
        } else if (debug_mode == RasterDebugMode::Triangle) {
            color = triangle_debug_color(tri_idx);
        } else if (tri.has_uv && tri.texture_index >= 0 && static_cast<size_t>(tri.texture_index) < textures.size()) {
            const TextureBitmap &texture = textures[static_cast<size_t>(tri.texture_index)];
            if (texture.pixels.empty())
                ++counters.invalid_uv_texture;
            else
                color = sample_texture_bilinear(texture, uv, color);
        }

        best_dist_sq = dist_sq;
        best_color = color;
        if (used_vertical_projection)
            ++counters.vertical_projected_samples;
        else
            ++counters.closest_point_samples;
        found = true;
    }

    if (found)
        out_color = best_color;
    if (found)
        ++counters.accepted_triangle_hits;
    return found;
}

static bool sample_pixel_color(
    const std::vector<RasterTriangle> &triangles,
    const std::vector<TextureBitmap> &textures,
    const TriangleGrid &grid,
    const Vec3d &pixel_center,
    double shell_thickness,
    RasterDebugMode debug_mode,
    LayerDebugCounters &counters,
    RGBA &out_color)
{
    if (sample_best_triangle(triangles, textures, grid, pixel_center, shell_thickness, debug_mode, counters, out_color))
        return true;

    static constexpr double edge_offsets[][2] = {
        {-0.35, -0.35}, { 0.35, -0.35}, {-0.35,  0.35}, { 0.35,  0.35},
        {-0.35,  0.00}, { 0.35,  0.00}, { 0.00, -0.35}, { 0.00,  0.35}
    };

    for (const auto &offset : edge_offsets) {
        const Vec3d query(
            pixel_center.x() + offset[0] * FULL_COLOR_PIXEL_SIZE_MM,
            pixel_center.y() + offset[1] * FULL_COLOR_PIXEL_SIZE_MM,
            pixel_center.z());
        if (sample_best_triangle(triangles, textures, grid, query, shell_thickness, debug_mode, counters, out_color)) {
            ++counters.subpixel_edge_hits;
            return true;
        }
    }

    return false;
}

static boost::filesystem::path fallback_chroma_output_path()
{
    const char *home = std::getenv("HOME");
    boost::filesystem::path base = home != nullptr && *home != '\0' ?
        boost::filesystem::path(home) / "Desktop" : boost::filesystem::temp_directory_path();

    return base / (boost::filesystem::unique_path("orcaslicer_full_color_%%%%-%%%%-%%%%").string() + ".chroma");
}

static boost::filesystem::path chroma_output_path_for_gcode(const std::string &gcode_path, std::string &reason)
{
    const boost::filesystem::path gcode(gcode_path);

    if (!gcode_path.empty() && !gcode.stem().empty()) {
        reason = "gcode_sidecar_path";
        return gcode.parent_path() / (gcode.stem().string() + ".chroma");
    }

    reason = "missing_gcode_path";
    const boost::filesystem::path fallback = fallback_chroma_output_path();
    BOOST_LOG_TRIVIAL(warning) << "FullColor Raster: no user-facing G-code output path available, using debug chroma=" << fallback.string();
    return fallback;
}

static std::string safe_package_gcode_filename(const std::string &requested_name, const std::string &source_gcode_path)
{
    boost::filesystem::path filename = boost::filesystem::path(requested_name).filename();
    if (filename.empty())
        filename = boost::filesystem::path(source_gcode_path).filename();

    std::string out = filename.string();
    if (out.empty() || out.front() == '.')
        out = "full_color_print.gcode";

    boost::filesystem::path normalized(out);
    normalized.replace_extension(".gcode");
    return normalized.filename().string();
}

static void write_u16(std::ofstream &out, std::uint16_t value)
{
    out.put(static_cast<char>(value & 0xff));
    out.put(static_cast<char>((value >> 8) & 0xff));
}

static void write_u32(std::ofstream &out, std::uint32_t value)
{
    out.put(static_cast<char>(value & 0xff));
    out.put(static_cast<char>((value >> 8) & 0xff));
    out.put(static_cast<char>((value >> 16) & 0xff));
    out.put(static_cast<char>((value >> 24) & 0xff));
}

struct ZipEntryInfo
{
    std::string name;
    std::uint32_t crc = 0;
    std::uint32_t size = 0;
    std::uint32_t local_header_offset = 0;
    std::uint16_t mod_time = 0;
    std::uint16_t mod_date = 0;
};

struct ZipTimestamp
{
    std::uint16_t time = 0;
    std::uint16_t date = 0;
};

static ZipTimestamp current_zip_timestamp()
{
    const std::time_t now = std::time(nullptr);
    std::tm local_tm {};
#ifdef _WIN32
    localtime_s(&local_tm, &now);
#else
    localtime_r(&now, &local_tm);
#endif
    const int year = std::clamp(local_tm.tm_year + 1900, 1980, 2107);
    return {
        static_cast<std::uint16_t>((local_tm.tm_hour << 11) | (local_tm.tm_min << 5) | (local_tm.tm_sec / 2)),
        static_cast<std::uint16_t>(((year - 1980) << 9) | ((local_tm.tm_mon + 1) << 5) | local_tm.tm_mday)
    };
}

static std::string zip_relative_name(const boost::filesystem::path &root, const boost::filesystem::path &file)
{
    std::string rel = boost::filesystem::relative(file, root).generic_string();
    while (!rel.empty() && rel.front() == '/')
        rel.erase(rel.begin());
    return rel;
}

static bool collect_package_files(
    const boost::filesystem::path &root,
    std::vector<boost::filesystem::path> &files,
    std::string &error)
{
    try {
        if (!boost::filesystem::exists(root) || !boost::filesystem::is_directory(root)) {
            error = "staging directory does not exist: " + root.string();
            return false;
        }

        for (boost::filesystem::recursive_directory_iterator it(root), end; it != end; ++it) {
            if (boost::filesystem::is_regular_file(it->path()))
                files.push_back(it->path());
        }

        std::sort(files.begin(), files.end(), [&root](const auto &a, const auto &b) {
            return zip_relative_name(root, a) < zip_relative_name(root, b);
        });

        return true;
    } catch (const std::exception &e) {
        error = e.what();
        return false;
    }
}

// Minimal ZIP writer using stored/uncompressed entries.
// This keeps .chroma packaging dependency-light and avoids calling external zip tools.
static bool write_stored_zip_from_directory(
    const boost::filesystem::path &root,
    const boost::filesystem::path &zip_path,
    std::string &error)
{
    std::vector<boost::filesystem::path> files;
    if (!collect_package_files(root, files, error))
        return false;

    try {
        if (!zip_path.parent_path().empty())
            boost::filesystem::create_directories(zip_path.parent_path());

        std::ofstream out(zip_path.string(), std::ios::binary | std::ios::trunc);
        if (!out) {
            error = "failed to open chroma package for writing: " + zip_path.string();
            return false;
        }

        std::vector<ZipEntryInfo> entries;
        entries.reserve(files.size());
        const ZipTimestamp timestamp = current_zip_timestamp();

        for (const boost::filesystem::path &file : files) {
            boost::nowide::ifstream in(file.string(), std::ios::binary);
            if (!in) {
                error = "failed to read package file: " + file.string();
                return false;
            }

            std::vector<unsigned char> data((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
            const std::string name = zip_relative_name(root, file);

            if (name.empty())
                continue;
            if (data.size() > std::numeric_limits<std::uint32_t>::max()) {
                error = "file too large for minimal ZIP writer: " + file.string();
                return false;
            }
            if (name.size() > std::numeric_limits<std::uint16_t>::max()) {
                error = "zip entry name too long: " + name;
                return false;
            }

            ZipEntryInfo entry;
            entry.name = name;
            entry.size = static_cast<std::uint32_t>(data.size());
            entry.crc = static_cast<std::uint32_t>(crc32(0L, data.data(), static_cast<uInt>(data.size())));
            entry.local_header_offset = static_cast<std::uint32_t>(out.tellp());
            entry.mod_time = timestamp.time;
            entry.mod_date = timestamp.date;

            write_u32(out, 0x04034b50);
            write_u16(out, 20);
            write_u16(out, 0);
            write_u16(out, 0);
            write_u16(out, entry.mod_time);
            write_u16(out, entry.mod_date);
            write_u32(out, entry.crc);
            write_u32(out, entry.size);
            write_u32(out, entry.size);
            write_u16(out, static_cast<std::uint16_t>(entry.name.size()));
            write_u16(out, 0);
            out.write(entry.name.data(), static_cast<std::streamsize>(entry.name.size()));
            if (!data.empty())
                out.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size()));

            entries.emplace_back(std::move(entry));
        }

        const std::uint32_t central_dir_offset = static_cast<std::uint32_t>(out.tellp());

        for (const ZipEntryInfo &entry : entries) {
            write_u32(out, 0x02014b50);
            write_u16(out, 20);
            write_u16(out, 20);
            write_u16(out, 0);
            write_u16(out, 0);
            write_u16(out, entry.mod_time);
            write_u16(out, entry.mod_date);
            write_u32(out, entry.crc);
            write_u32(out, entry.size);
            write_u32(out, entry.size);
            write_u16(out, static_cast<std::uint16_t>(entry.name.size()));
            write_u16(out, 0);
            write_u16(out, 0);
            write_u16(out, 0);
            write_u16(out, 0);
            write_u32(out, 0);
            write_u32(out, entry.local_header_offset);
            out.write(entry.name.data(), static_cast<std::streamsize>(entry.name.size()));
        }

        const std::uint32_t central_dir_size =
            static_cast<std::uint32_t>(static_cast<std::uint32_t>(out.tellp()) - central_dir_offset);

        if (entries.size() > std::numeric_limits<std::uint16_t>::max()) {
            error = "too many files for minimal ZIP writer";
            return false;
        }

        write_u32(out, 0x06054b50);
        write_u16(out, 0);
        write_u16(out, 0);
        write_u16(out, static_cast<std::uint16_t>(entries.size()));
        write_u16(out, static_cast<std::uint16_t>(entries.size()));
        write_u32(out, central_dir_size);
        write_u32(out, central_dir_offset);
        write_u16(out, 0);

        out.close();
        return true;
    } catch (const std::exception &e) {
        error = e.what();
        return false;
    }
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

static void write_debug_log(
    const boost::filesystem::path &path,
    const std::string &output_reason,
    RasterDebugMode debug_mode,
    size_t triangle_count,
    size_t triangle_count_mesh,
    size_t triangle_count_full_color_metadata,
    bool triangle_mapping_validated,
    size_t grid_cells,
    size_t texture_count,
    size_t loaded_texture_count,
    size_t layer_count,
    const Vec3d &min_pt,
    const Vec3d &max_pt,
    int width,
    int height,
    double shell_thickness,
    const RasterTiming &timing)
{
    boost::nowide::ofstream out(path.string());
    out << "FullColor Raster Debug Log\n";
    out << "output_reason=" << output_reason << "\n";
    out << "raster_geometry_source=PrintObject::trafo_centered() * ModelVolume::get_matrix() * ModelVolume::mesh().its\n";
    out << "viewport_geometry_source=ModelVolume::mesh().its, rendered through GLVolume instance/volume world_matrix\n";
    out << "triangle_mapping_strategy=mesh triangle index matched to FullColor::SurfaceData::triangles index\n";
    out << "triangle_mapping_validated=" << (triangle_mapping_validated ? "true" : "false") << "\n";
    out << "coordinate_space=slicer object space with PrintInstance::shift_without_plate_offset applied in XY\n";
    out << "raster_strategy=vertical-layer-projection-with-closest-point-fallback-uniform-grid-fdm-mask\n";
    out << "raster_clip=actual-fdm-extrusion-mask\n";
    out << "fdm_mask_source=LayerRegion perimeters/fills/thin_fills polygons_covered_by_width_or_spacing\n";
    out << "debug_mode=" << raster_debug_mode_name(debug_mode) << "\n";
    out << "texture_sampling=bilinear\n";
    out << "uv_flip_v=" << (FULL_COLOR_OBJ_FLIP_V ? "true" : "false") << "\n";
    out << "cpu_texture_v_flip=" << (FULL_COLOR_CPU_TEXTURE_FLIP_V ? "true" : "false") << "\n";
    out << "texture_address_mode=" << FULL_COLOR_TEXTURE_ADDRESS_MODE << "\n";
    out << "texture_buffer_origin=" << FULL_COLOR_TEXTURE_BUFFER_ORIGIN << "\n";
    out << "pixel_size_mm=" << FULL_COLOR_PIXEL_SIZE_MM << "\n";
    out << "fdm_mask_bleed_mm=" << FULL_COLOR_FDM_MASK_BLEED_MM << "\n";
    out << "shell_thickness_mm=" << shell_thickness << "\n";
    out << "normal_tolerance_mm=" << FULL_COLOR_NORMAL_TOLERANCE_MM << "\n";
    out << "triangle_count=" << triangle_count << "\n";
    out << "triangle_count_mesh=" << triangle_count_mesh << "\n";
    out << "triangle_count_full_color_metadata=" << triangle_count_full_color_metadata << "\n";
    out << "grid_cells=" << grid_cells << "\n";
    out << "texture_count=" << texture_count << "\n";
    out << "loaded_texture_count=" << loaded_texture_count << "\n";
    out << "layer_count=" << layer_count << "\n";
    out << "image_width=" << width << "\n";
    out << "image_height=" << height << "\n";
    out << "bounds_min=" << min_pt.x() << "," << min_pt.y() << "," << min_pt.z() << "\n";
    out << "bounds_max=" << max_pt.x() << "," << max_pt.y() << "," << max_pt.z() << "\n";
    out << "timing_total_ms=" << timing.total_ms << "\n";
    out << "timing_surface_prep_ms=" << timing.surface_prep_ms << "\n";
    out << "timing_grid_build_ms=" << timing.grid_build_ms << "\n";
    out << "timing_layer_raster_total_ms=" << timing.layer_raster_total_ms << "\n";
    out << "timing_layer_average_ms=" << timing.layer_average_ms << "\n";
    out << "timing_slowest_layer_ms=" << timing.slowest_layer_ms << "\n";
    out << "timing_staging_ms=" << timing.staging_ms << "\n";
    out << "timing_manifest_ms=" << timing.manifest_ms << "\n";
    out << "timing_packaging_ms=" << timing.packaging_ms << "\n";
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

RasterPipelineOutput generate_full_color_rasters(
    const Print &print,
    const std::string &gcode_path,
    const std::string &package_gcode_filename,
    std::function<void(int, const std::string&)> status_callback)
{
    const auto total_start = RasterClock::now();
    RasterTiming timing;
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
    const RasterDebugMode debug_mode = raster_debug_mode();
    if (status_callback)
        status_callback(81, "Preparing full-color raster data");
    BOOST_LOG_TRIVIAL(info) << "FullColor Raster: generating layer rasters";
    BOOST_LOG_TRIVIAL(info) << "FullColor Raster: pixel_size=" << FULL_COLOR_PIXEL_SIZE_MM << " mm";
    BOOST_LOG_TRIVIAL(info) << "FullColor Raster: shell_thickness=" << shell_thickness << " mm";
    BOOST_LOG_TRIVIAL(info) << "FullColor Raster: shell_mode=normal-inward";
    BOOST_LOG_TRIVIAL(info) << "FullColor Raster: raster_strategy=vertical-layer-projection-with-closest-point-fallback-uniform-grid-fdm-mask";
    BOOST_LOG_TRIVIAL(info) << "FullColor Raster: fdm_mask_bleed=" << FULL_COLOR_FDM_MASK_BLEED_MM << " mm";
    BOOST_LOG_TRIVIAL(info) << "FullColor Raster: debug_mode=" << raster_debug_mode_name(debug_mode);

    std::vector<RasterTriangle> triangles;
    triangles.reserve(output.summary.total_color_triangles);
    std::vector<TextureBitmap> textures;
    textures.reserve(output.summary.total_textures);
    size_t loaded_texture_count = 0;
    size_t triangle_count_mesh = 0;
    size_t triangle_count_full_color_metadata = 0;
    size_t validated_volume_count = 0;
    size_t skipped_mapping_mismatch_volume_count = 0;

    const auto surface_prep_start = RasterClock::now();
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
                const size_t texture_base = textures.size();
                for (const TextureImage &texture : surface.textures) {
                    TextureBitmap bitmap;
                    if (load_texture_bitmap(texture, bitmap))
                        ++loaded_texture_count;
                    textures.emplace_back(std::move(bitmap));
                }

                const Transform3d transform = instance_transform * volume->get_matrix();
                const indexed_triangle_set &its = volume->mesh().its;
                triangle_count_mesh += its.indices.size();
                triangle_count_full_color_metadata += surface.triangles.size();
                if (surface.triangles.size() != its.indices.size()) {
                    BOOST_LOG_TRIVIAL(warning) << "FullColor Raster: triangle metadata mismatch for volume; metadata="
                                               << surface.triangles.size() << ", mesh=" << its.indices.size()
                                               << "; skipping volume to avoid incorrect UV remapping";
                    ++skipped_mapping_mismatch_volume_count;
                    continue;
                }
                ++validated_volume_count;
                const size_t triangle_count = std::min(surface.triangles.size(), its.indices.size());
                for (size_t triangle_idx = 0; triangle_idx < triangle_count; ++triangle_idx) {
                    const stl_triangle_vertex_indices face = its.indices[triangle_idx];
                    const Vec3d p0 = transform_vertex(transform, its.vertices[face[0]]);
                    const Vec3d p1 = transform_vertex(transform, its.vertices[face[1]]);
                    const Vec3d p2 = transform_vertex(transform, its.vertices[face[2]]);
                    const Vec3d n_raw = (p1 - p0).cross(p2 - p0);
                    if (n_raw.squaredNorm() <= 1e-12)
                        continue;

                    const TriangleColorData &triangle_data = surface.triangles[triangle_idx];
                    RasterTriangle tri;
                    tri.p0 = p0;
                    tri.p1 = p1;
                    tri.p2 = p2;
                    tri.min_pt = p0.cwiseMin(p1).cwiseMin(p2);
                    tri.max_pt = p0.cwiseMax(p1).cwiseMax(p2);
                    tri.normal = n_raw.normalized();
                    tri.uv = triangle_data.uv;
                    tri.fallback_color = triangle_data.fallback_color;
                    if (!triangle_data.has_color && surface.has_vertex_colors &&
                        static_cast<size_t>(face[0]) < surface.vertex_colors.size() &&
                        static_cast<size_t>(face[1]) < surface.vertex_colors.size() &&
                        static_cast<size_t>(face[2]) < surface.vertex_colors.size()) {
                        RGBA average_color{0.0f, 0.0f, 0.0f, 0.0f};
                        for (size_t corner = 0; corner < 3; ++corner)
                            for (size_t channel = 0; channel < 4; ++channel)
                                average_color[channel] += surface.vertex_colors[face[corner]][channel] / 3.0f;
                        tri.fallback_color = average_color;
                    }
                    tri.has_uv = triangle_data.has_uv;
                    tri.texture_index = triangle_data.texture_index >= 0 ?
                        static_cast<int>(texture_base + static_cast<size_t>(triangle_data.texture_index)) : -1;
                    triangles.emplace_back(std::move(tri));
                }
            }
        }
    }
    timing.surface_prep_ms = elapsed_ms(surface_prep_start);

    if (triangles.empty()) {
        BOOST_LOG_TRIVIAL(info) << "FullColor Raster: enabled, but no raster triangles could be generated";
        return output;
    }

    Vec3d min_pt(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
    Vec3d max_pt(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest());
    for (const RasterTriangle &tri : triangles) {
        min_pt = min_pt.cwiseMin(tri.p0).cwiseMin(tri.p1).cwiseMin(tri.p2);
        max_pt = max_pt.cwiseMax(tri.p0).cwiseMax(tri.p1).cwiseMax(tri.p2);
    }

    const int width = std::max(1, static_cast<int>(std::ceil((max_pt.x() - min_pt.x()) / FULL_COLOR_PIXEL_SIZE_MM)));
    const int height = std::max(1, static_cast<int>(std::ceil((max_pt.y() - min_pt.y()) / FULL_COLOR_PIXEL_SIZE_MM)));
    const std::vector<LayerRasterInfo> layers = collect_layers(print);
    if (layers.empty()) {
        BOOST_LOG_TRIVIAL(info) << "FullColor Raster: enabled, but no print layers are available";
        return output;
    }

    const auto grid_build_start = RasterClock::now();
    const TriangleGrid triangle_grid = build_triangle_grid(triangles, shell_thickness);
    timing.grid_build_ms = elapsed_ms(grid_build_start);
    BOOST_LOG_TRIVIAL(info) << "FullColor Raster: triangle_count=" << triangles.size()
                            << ", grid_cells=" << triangle_grid.cells.size()
                            << ", grid_cell_size=" << triangle_grid.cell_size << " mm";

    std::string output_reason;
    const boost::filesystem::path chroma_path = chroma_output_path_for_gcode(gcode_path, output_reason);
    const boost::filesystem::path staging_dir =
        boost::filesystem::temp_directory_path() / boost::filesystem::unique_path("orcaslicer_chroma_stage_%%%%-%%%%-%%%%");
    const boost::filesystem::path layer_dir = staging_dir / "layers";
    const boost::filesystem::path layer_metadata_dir = staging_dir / "layer_metadata";

    const auto staging_start = RasterClock::now();
    boost::filesystem::create_directories(layer_dir);
    boost::filesystem::create_directories(layer_metadata_dir);
    timing.staging_ms += elapsed_ms(staging_start);

    BOOST_LOG_TRIVIAL(info) << "FullColor Raster: staging=" << staging_dir.string();
    BOOST_LOG_TRIVIAL(info) << "FullColor Raster: chroma=" << chroma_path.string();
    BOOST_LOG_TRIVIAL(info) << "FullColor Raster: output_reason=" << output_reason;
    BOOST_LOG_TRIVIAL(info) << "FullColor Raster: layer_count=" << layers.size();

    const bool triangle_mapping_validated =
        validated_volume_count > 0 && skipped_mapping_mismatch_volume_count == 0 &&
        triangle_count_mesh == triangle_count_full_color_metadata;
    const std::string gcode_filename = safe_package_gcode_filename(package_gcode_filename, gcode_path);

    if (!gcode_path.empty() && boost::filesystem::exists(gcode_path)) {
        const auto copy_start = RasterClock::now();
        try {
            const boost::filesystem::path gcode_src(gcode_path);
            boost::filesystem::copy_file(
                gcode_src,
                staging_dir / gcode_filename,
                boost::filesystem::copy_option::overwrite_if_exists);
        } catch (const std::exception &e) {
            BOOST_LOG_TRIVIAL(error) << "FullColor Raster: failed to copy G-code into package staging: " << e.what();
        }
        timing.staging_ms += elapsed_ms(copy_start);
    } else {
        BOOST_LOG_TRIVIAL(warning) << "FullColor Raster: G-code file not available for package root: " << gcode_path;
    }

    const double first_layer_height = layers.empty() ? 0.0 : layers.front().z1 - layers.front().z0;
    const double nominal_layer_height = layers.size() > 1 ? layers[1].z1 - layers[0].z1 : first_layer_height;
    const bool vertex_colors_present = std::any_of(print.model().objects.begin(), print.model().objects.end(), [](const ModelObject *object) {
        if (object == nullptr)
            return false;
        return std::any_of(object->volumes.begin(), object->volumes.end(), [](const ModelVolume *volume) {
            return volume != nullptr && volume->full_color_data && volume->full_color_data->has_vertex_colors;
        });
    });
    const bool face_colors_present = std::any_of(print.model().objects.begin(), print.model().objects.end(), [](const ModelObject *object) {
        if (object == nullptr)
            return false;
        return std::any_of(object->volumes.begin(), object->volumes.end(), [](const ModelVolume *volume) {
            return volume != nullptr && volume->full_color_data && volume->full_color_data->has_face_colors;
        });
    });
    const bool uv_triangles_present = std::any_of(triangles.begin(), triangles.end(), [](const RasterTriangle &tri) {
        return tri.has_uv;
    });

    json manifest;
    manifest["format"] = "OrcaSlicerChroma";
    manifest["version"] = 1;
    manifest["generator"] = {
        {"name", "OrcaSlicer"},
        {"build", std::string(SLIC3R_VERSION)},
        {"full_color_mvp", true}
    };
    manifest["package"] = {
        {"type", "chroma"},
        {"container", "zip"},
        {"gcode_file", gcode_filename},
        {"layers_dir", "layers"},
        {"layer_metadata_dir", "layer_metadata"},
        {"debug_log", "debug_log.txt"}
    };
    manifest["source"] = {
        {"source_color_formats", {"OBJ"}},
        {"primary_model_format", "OBJ"},
        {"texture_format_support", {"PNG"}},
        {"known_deferred_formats", {
            "3MF texture/color support",
            "glTF/GLB textured mesh support",
            "PLY vertex colors",
            "VRML/X3D if useful"
        }}
    };
    manifest["config"] = {
        {"enable_full_color_printing", true},
        {"full_color_shell_thickness_mm", shell_thickness},
        {"full_color_pixel_size_mm", FULL_COLOR_PIXEL_SIZE_MM},
        {"fdm_mask_bleed_mm", FULL_COLOR_FDM_MASK_BLEED_MM},
        {"normal_tolerance_mm", FULL_COLOR_NORMAL_TOLERANCE_MM},
        {"shell_mode", "normal-inward"},
        {"raster_strategy", "vertical-layer-projection-with-closest-point-fallback-uniform-grid-fdm-mask"},
        {"raster_clip", "actual-fdm-extrusion-mask"},
        {"fdm_mask_source", "LayerRegion perimeters/fills/thin_fills polygons_covered_by_width_or_spacing"},
        {"texture_sampling", "bilinear"},
        {"uv_flip_v", FULL_COLOR_OBJ_FLIP_V},
        {"cpu_texture_v_flip", FULL_COLOR_CPU_TEXTURE_FLIP_V},
        {"texture_address_mode", FULL_COLOR_TEXTURE_ADDRESS_MODE},
        {"texture_buffer_origin", FULL_COLOR_TEXTURE_BUFFER_ORIGIN},
        {"background", "white"},
        {"units", "mm"}
    };
    manifest["print"] = {
        {"layer_count", layers.size()},
        {"first_layer_height_mm", first_layer_height},
        {"layer_height_mm", nominal_layer_height},
        {"plate_index", 0},
        {"printer_profile", print.config().printer_model.value.empty() ? std::string("unknown") : print.config().printer_model.value},
        {"filament_profile", "unknown"}
    };
    manifest["bounds"] = {
        {"coordinate_space", "print-bed"},
        {"origin", "min model bounds in print-bed XY"},
        {"min_mm", {min_pt.x(), min_pt.y(), min_pt.z()}},
        {"max_mm", {max_pt.x(), max_pt.y(), max_pt.z()}},
        {"image_width_px", width},
        {"image_height_px", height},
        {"image_row_0_y_mm", min_pt.y()},
        {"image_y_flipped", false}
    };
    manifest["full_color_data"] = {
        {"volume_count", output.summary.volume_count},
        {"textured_volume_count", output.summary.textured_volume_count},
        {"triangle_count", triangles.size()},
        {"triangle_count_mesh", triangle_count_mesh},
        {"triangle_count_full_color_metadata", triangle_count_full_color_metadata},
        {"texture_count", textures.size()},
        {"loaded_texture_count", loaded_texture_count},
        {"triangle_mapping_strategy", "mesh-triangle-index-to-full-color-triangle-index"},
        {"triangle_mapping_validated", triangle_mapping_validated},
        {"validated_full_color_volumes", validated_volume_count},
        {"skipped_mapping_mismatch_volumes", skipped_mapping_mismatch_volume_count},
        {"vertex_colors_present", vertex_colors_present},
        {"face_colors_present", face_colors_present},
        {"uv_triangles_present", uv_triangles_present}
    };
    manifest["debug"] = {
        {"raster_debug_mode", raster_debug_mode_manifest_name(debug_mode)},
        {"uv_debug_available", true},
        {"triangle_debug_available", true},
        {"debug_env_vars", {
            {"ORCA_FULL_COLOR_RASTER_DEBUG", "none | uv | triangle"},
            {"ORCA_FULL_COLOR_PREVIEW", "0 | 1"}
        }},
        {"warnings", json::array()}
    };
    manifest["implementation"] = {
        {"raster_geometry_source", "PrintObject::trafo_centered() * ModelVolume::get_matrix() * ModelVolume::mesh().its"},
        {"viewport_geometry_source", "ModelVolume::mesh().its with duplicated textured vertices, transformed by GLVolume world_matrix"},
        {"coordinate_space", "slicer object space with PrintInstance::shift_without_plate_offset applied in XY"},
        {"grid_cells", triangle_grid.cells.size()},
        {"output_reason", output_reason}
    };
    manifest["overlay_preview"] = {
        {"enabled_by_viewer_toggle", "Show full-color layers"},
        {"rendering_method", "transparent-white textured quad in G-code preview"},
        {"white_to_alpha_threshold", 250},
        {"two_sided", true},
        {"coordinate_space", "print-bed"},
        {"z_offset_mm", 0.02},
        {"layer_matching", "manifest layer_index matched to current G-code preview layer slider"}
    };
    manifest["layers"] = json::array();

    std::vector<double> pixel_center_x(static_cast<size_t>(width));
    std::vector<double> pixel_center_y(static_cast<size_t>(height));
    std::vector<coord_t> scaled_pixel_center_x(static_cast<size_t>(width));
    std::vector<coord_t> scaled_pixel_center_y(static_cast<size_t>(height));
    for (int x = 0; x < width; ++x) {
        pixel_center_x[static_cast<size_t>(x)] = min_pt.x() + (x + 0.5) * FULL_COLOR_PIXEL_SIZE_MM;
        scaled_pixel_center_x[static_cast<size_t>(x)] = scale_(pixel_center_x[static_cast<size_t>(x)]);
    }
    for (int y = 0; y < height; ++y) {
        pixel_center_y[static_cast<size_t>(y)] = min_pt.y() + (y + 0.5) * FULL_COLOR_PIXEL_SIZE_MM;
        scaled_pixel_center_y[static_cast<size_t>(y)] = scale_(pixel_center_y[static_cast<size_t>(y)]);
    }

    const size_t image_size = static_cast<size_t>(width) * static_cast<size_t>(height) * 3;
    std::vector<std::uint8_t> image(image_size, 255);
    std::vector<LayerDebugCounters> row_counters(static_cast<size_t>(height));

    for (const LayerRasterInfo &layer : layers) {
        if (status_callback) {
            const int percent = 82 + static_cast<int>((12.0 * static_cast<double>(layer.index)) / std::max<size_t>(layers.size(), 1));
            status_callback(percent, "Generating full-color rasters: layer " + std::to_string(layer.index + 1) + "/" + std::to_string(layers.size()));
        }

        const auto layer_start = RasterClock::now();
        LayerDebugCounters counters;
        counters.total_pixels = static_cast<size_t>(width) * static_cast<size_t>(height);
        std::fill(image.begin(), image.end(), 255);
        std::fill(row_counters.begin(), row_counters.end(), LayerDebugCounters{});
        const LayerRasterMask fdm_mask = build_layer_fdm_mask(print, layer);
        const LayerMaskRowIndex fdm_row_index = build_layer_mask_row_index(fdm_mask, min_pt, height);
        const LayerPixelBounds color_bounds = layer_pixel_bounds(triangles, min_pt, width, height, layer.sample_z, shell_thickness);
        const LayerPixelBounds mask_bounds = layer_mask_pixel_bounds(fdm_mask, min_pt, width, height);
        const LayerPixelBounds active_bounds = intersect_bounds(color_bounds, mask_bounds);

        if (!fdm_mask.empty() && !active_bounds.empty()) {
            tbb::parallel_for(
                tbb::blocked_range<int>(active_bounds.min_y, active_bounds.max_y + 1, 8),
                [&](const tbb::blocked_range<int> &range) {
                    for (int y = range.begin(); y != range.end(); ++y) {
                        LayerDebugCounters local;
                        const double py = pixel_center_y[static_cast<size_t>(y)];
                        const coord_t sy = scaled_pixel_center_y[static_cast<size_t>(y)];
                        for (int x = active_bounds.min_x; x <= active_bounds.max_x; ++x) {
                            const Vec3d query(
                                pixel_center_x[static_cast<size_t>(x)],
                                py,
                                layer.sample_z);
                            if (!contains_point_indexed(fdm_mask, fdm_row_index, y, Point(scaled_pixel_center_x[static_cast<size_t>(x)], sy))) {
                                ++local.rejected_by_fdm_mask;
                                continue;
                            }

                            RGBA color{1.0f, 1.0f, 1.0f, 1.0f};
                            if (sample_pixel_color(triangles, textures, triangle_grid, query, shell_thickness, debug_mode, local, color)) {
                                ++local.colored_pixels;
                                const size_t idx = (static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x)) * 3;
                                image[idx + 0] = to_u8(color[0]);
                                image[idx + 1] = to_u8(color[1]);
                                image[idx + 2] = to_u8(color[2]);
                            }
                        }
                        row_counters[static_cast<size_t>(y)] = local;
                    }
                }
            );
        }

        for (const LayerDebugCounters &row_counter : row_counters) {
            LayerDebugCounters row_no_total = row_counter;
            row_no_total.total_pixels = 0;
            add_counters(counters, row_no_total);
        }
        const double layer_ms = elapsed_ms(layer_start);
        timing.layer_raster_total_ms += layer_ms;
        timing.slowest_layer_ms = std::max(timing.slowest_layer_ms, layer_ms);

        const std::string base = layer_basename(layer.index);
        const boost::filesystem::path png_path = layer_dir / (base + ".png");
        const boost::filesystem::path json_path = layer_metadata_dir / (base + ".json");
        BOOST_LOG_TRIVIAL(info) << "FullColor Raster: writing " << png_path.filename().string();
        png::write_rgb_to_file(png_path.string(), width, height, image);

        json layer_json;
        layer_json["format"] = "OrcaSlicerChromaLayer";
        layer_json["version"] = 1;
        layer_json["layer"] = {
            {"layer_index", layer.index},
            {"z0_mm", layer.z0},
            {"z1_mm", layer.z1},
            {"sample_z_mm", layer.sample_z},
            {"source", "actual PrintObject layer slice_z/bottom_z/print_z"}
        };
        layer_json["image"] = {
            {"file", (boost::filesystem::path("layers") / png_path.filename()).string()},
            {"width_px", width},
            {"height_px", height},
            {"channels", "RGB"},
            {"background", "white"},
            {"pixel_size_mm", FULL_COLOR_PIXEL_SIZE_MM},
            {"origin_xy_mm", {min_pt.x(), min_pt.y()}},
            {"image_row_0_y_mm", min_pt.y()},
            {"image_y_flipped", false},
            {"coordinate_space", "print-bed"},
            {"units", "mm"}
        };
        layer_json["config"] = {
            {"enable_full_color_printing", true},
            {"full_color_shell_thickness_mm", shell_thickness},
            {"fdm_mask_bleed_mm", FULL_COLOR_FDM_MASK_BLEED_MM},
            {"normal_tolerance_mm", FULL_COLOR_NORMAL_TOLERANCE_MM},
            {"shell_mode", "normal-inward"},
            {"raster_strategy", "vertical-layer-projection-with-closest-point-fallback-uniform-grid-fdm-mask"},
            {"raster_clip", "actual-fdm-extrusion-mask"},
            {"fdm_mask_source", "LayerRegion perimeters/fills/thin_fills polygons_covered_by_width_or_spacing"},
            {"texture_sampling", "bilinear"},
            {"uv_flip_v", FULL_COLOR_OBJ_FLIP_V},
            {"cpu_texture_v_flip", FULL_COLOR_CPU_TEXTURE_FLIP_V},
            {"texture_address_mode", FULL_COLOR_TEXTURE_ADDRESS_MODE},
            {"texture_buffer_origin", FULL_COLOR_TEXTURE_BUFFER_ORIGIN}
        };
        layer_json["debug"] = {
            {"raster_debug_mode", raster_debug_mode_manifest_name(debug_mode)},
            {"uv_debug_available", true},
            {"triangle_debug_available", true},
            {"total_pixels", counters.total_pixels},
            {"colored_pixels", counters.colored_pixels},
            {"candidate_misses", counters.candidate_misses},
            {"candidate_triangle_tests", counters.candidate_triangle_tests},
            {"accepted_triangle_hits", counters.accepted_triangle_hits},
            {"subpixel_edge_hits", counters.subpixel_edge_hits},
            {"active_bounds_empty", active_bounds.empty()},
            {"active_bounds_px", active_bounds.empty() ? json(nullptr) : json{
                {"min_x", active_bounds.min_x},
                {"max_x", active_bounds.max_x},
                {"min_y", active_bounds.min_y},
                {"max_y", active_bounds.max_y}
            }},
            {"color_bounds_empty", color_bounds.empty()},
            {"fdm_mask_empty", fdm_mask.empty()},
            {"fdm_mask_expolygon_count", fdm_mask.printable_area.size()},
            {"fdm_mask_bbox_count", fdm_mask.bboxes.size()},
            {"fdm_mask_bounds_px", mask_bounds.empty() ? json(nullptr) : json{
                {"min_x", mask_bounds.min_x},
                {"max_x", mask_bounds.max_x},
                {"min_y", mask_bounds.min_y},
                {"max_y", mask_bounds.max_y}
            }},
            {"layer_raster_time_ms", layer_ms},
            {"vertical_projected_samples", counters.vertical_projected_samples},
            {"closest_point_samples", counters.closest_point_samples},
            {"rejected_by_xy_projection", counters.rejected_by_xy_projection},
            {"rejected_by_shell_distance", counters.rejected_by_shell_distance},
            {"rejected_by_outward_shell", counters.rejected_by_outward_shell},
            {"rejected_by_fdm_mask", counters.rejected_by_fdm_mask},
            {"rejected_by_normal", counters.rejected_by_normal},
            {"invalid_uv_texture", counters.invalid_uv_texture}
        };
        layer_json["full_color_data"] = {
            {"triangle_count", triangles.size()},
            {"texture_count", textures.size()},
            {"loaded_texture_count", loaded_texture_count},
            {"triangle_mapping_strategy", "mesh-triangle-index-to-full-color-triangle-index"},
            {"triangle_mapping_validated", triangle_mapping_validated}
        };
        layer_json["bounds"] = {
            {"min_mm", {min_pt.x(), min_pt.y(), min_pt.z()}},
            {"max_mm", {max_pt.x(), max_pt.y(), max_pt.z()}}
        };
        layer_json["notes"] = json::array();
        write_json_file(json_path, layer_json);

        manifest["layers"].push_back({
            {"layer_index", layer.index},
            {"z0_mm", layer.z0},
            {"z1_mm", layer.z1},
            {"sample_z_mm", layer.sample_z},
            {"colored_pixels", counters.colored_pixels},
            {"fdm_mask_empty", fdm_mask.empty()},
            {"layer_raster_time_ms", layer_ms},
            {"png", (boost::filesystem::path("layers") / png_path.filename()).string()},
            {"metadata", (boost::filesystem::path("layer_metadata") / json_path.filename()).string()}
        });
    }

    const boost::filesystem::path manifest_path = staging_dir / "manifest.json";
    timing.layer_average_ms = layers.empty() ? 0.0 : timing.layer_raster_total_ms / static_cast<double>(layers.size());
    timing.total_ms = elapsed_ms(total_start);
    if (status_callback)
        status_callback(95, "Packaging full-color Chroma output");
    manifest["timing"] = {
        {"total_before_packaging_ms", timing.total_ms},
        {"surface_triangle_preparation_ms", timing.surface_prep_ms},
        {"grid_build_ms", timing.grid_build_ms},
        {"layer_raster_total_ms", timing.layer_raster_total_ms},
        {"layer_average_ms", timing.layer_average_ms},
        {"slowest_layer_ms", timing.slowest_layer_ms},
        {"staging_ms", timing.staging_ms},
        {"manifest_write_ms", timing.manifest_ms},
        {"packaging_time_ms", timing.packaging_ms},
        {"packaging_time_note", "Actual package write time is logged after the ZIP is closed; it is not known while writing files inside the package."},
        {"parallelization", "per-layer rows via tbb::parallel_for"}
    };

    const auto manifest_start = RasterClock::now();
    write_json_file(manifest_path, manifest);
    timing.manifest_ms = elapsed_ms(manifest_start);
    timing.total_ms = elapsed_ms(total_start);
    manifest["timing"]["total_before_packaging_ms"] = timing.total_ms;
    manifest["timing"]["manifest_write_ms"] = timing.manifest_ms;
    write_json_file(manifest_path, manifest);

    write_debug_log(staging_dir / "debug_log.txt", output_reason, debug_mode, triangles.size(),
                    triangle_count_mesh, triangle_count_full_color_metadata, triangle_mapping_validated,
                    triangle_grid.cells.size(), textures.size(), loaded_texture_count, layers.size(),
                    min_pt, max_pt, width, height, shell_thickness, timing);

    std::string zip_error;
    const auto packaging_start = RasterClock::now();
    const bool zip_ok = write_stored_zip_from_directory(staging_dir, chroma_path, zip_error);
    timing.packaging_ms = elapsed_ms(packaging_start);
    if (!zip_ok) {
        BOOST_LOG_TRIVIAL(error) << "FullColor Raster: failed to write .chroma package: " << zip_error;
        output.output_dir = staging_dir.string();
        output.manifest_path = manifest_path.string();
        output.layer_count = layers.size();
        output.generated = false;
        return output;
    }

    BOOST_LOG_TRIVIAL(info) << "FullColor Raster: chroma package written=" << chroma_path.string();
    BOOST_LOG_TRIVIAL(info) << "FullColor Raster: timings total_before_packaging_ms=" << timing.total_ms
                            << ", surface_prep_ms=" << timing.surface_prep_ms
                            << ", grid_build_ms=" << timing.grid_build_ms
                            << ", layer_raster_total_ms=" << timing.layer_raster_total_ms
                            << ", layer_average_ms=" << timing.layer_average_ms
                            << ", slowest_layer_ms=" << timing.slowest_layer_ms
                            << ", staging_ms=" << timing.staging_ms
                            << ", manifest_write_ms=" << timing.manifest_ms
                            << ", packaging_ms=" << timing.packaging_ms;

    try {
        boost::filesystem::remove_all(staging_dir);
    } catch (const std::exception &e) {
        BOOST_LOG_TRIVIAL(warning) << "FullColor Raster: failed to remove staging dir '" << staging_dir.string()
                                   << "': " << e.what();
    }

    output.output_dir = chroma_path.string();
    output.manifest_path = "manifest.json";
    output.layer_count = layers.size();
    output.generated = true;

    BOOST_LOG_TRIVIAL(info) << "FullColor Preview: Show full-color layers scaffold package=" << output.output_dir;

    return output;
}

} // namespace Slic3r::FullColor
