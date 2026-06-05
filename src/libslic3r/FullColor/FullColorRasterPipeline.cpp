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

#include <zlib.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <cstdint>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace Slic3r::FullColor {

namespace {

using json = nlohmann::json;

static constexpr double FULL_COLOR_PIXEL_SIZE_MM        = 0.0846667; // ~300 DPI
static constexpr double FULL_COLOR_NORMAL_TOLERANCE_MM = 0.02;
static constexpr bool   FULL_COLOR_OBJ_FLIP_V          = true;

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

    u = u - std::floor(u);
    v = v - std::floor(v);

    // OBJ/MTL UVs use a bottom-left texture origin in most exporters, while PNG rows are top-down.
    // Match the earlier prototype and typical OBJ raster convention by flipping V on CPU raster sampling.
    if (FULL_COLOR_OBJ_FLIP_V)
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

static bool sample_best_triangle(
    const std::vector<RasterTriangle> &triangles,
    const std::vector<TextureBitmap> &textures,
    const TriangleGrid &grid,
    const Vec3d &query,
    double shell_thickness,
    RGBA &out_color)
{
    const GridKey key = grid_key(query, grid.cell_size);
    const auto it = grid.cells.find(key);
    if (it == grid.cells.end())
        return false;

    const double shell_sq = shell_thickness * shell_thickness;
    double best_dist_sq = shell_sq;
    bool found = false;
    RGBA best_color{1.0f, 1.0f, 1.0f, 1.0f};

    for (const size_t tri_idx : it->second) {
        if (tri_idx >= triangles.size())
            continue;

        const RasterTriangle &tri = triangles[tri_idx];
        const Vec3d closest = closest_point_on_triangle(query, tri.p0, tri.p1, tri.p2);
        const Vec3d delta = query - closest;
        const double normal_distance = delta.dot(tri.normal);
        if (normal_distance > FULL_COLOR_NORMAL_TOLERANCE_MM)
            continue;

        const double dist_sq = delta.squaredNorm();
        if (dist_sq > best_dist_sq)
            continue;

        RGBA color = tri.fallback_color;
        if (tri.has_uv && tri.texture_index >= 0 && static_cast<size_t>(tri.texture_index) < textures.size()) {
            const TextureBitmap &texture = textures[static_cast<size_t>(tri.texture_index)];
            const Vec3d bary = barycentric_coords(closest, tri.p0, tri.p1, tri.p2);
            const Vec2f uv = tri.uv[0] * static_cast<float>(bary.x()) +
                             tri.uv[1] * static_cast<float>(bary.y()) +
                             tri.uv[2] * static_cast<float>(bary.z());
            color = sample_texture_bilinear(texture, uv, color);
        }

        best_dist_sq = dist_sq;
        best_color = color;
        found = true;
    }

    if (found)
        out_color = best_color;
    return found;
}

static bool looks_like_internal_metadata_path(const boost::filesystem::path &path)
{
    if (path.empty())
        return true;

    const std::string parent = path.parent_path().filename().string();
    const std::string stem = path.stem().string();

    if (parent == "Metadata")
        return true;
    if (!stem.empty() && stem.front() == '.')
        return true;
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

    if (!gcode_path.empty() && !gcode.stem().empty() && !looks_like_internal_metadata_path(gcode)) {
        reason = "final_gcode_path";
        return gcode.parent_path() / (gcode.stem().string() + ".chroma");
    }

    reason = gcode_path.empty() ? "missing_gcode_path" : "internal_or_hidden_metadata_path";
    const boost::filesystem::path fallback = fallback_chroma_output_path();
    BOOST_LOG_TRIVIAL(warning) << "FullColor Raster: no user-facing G-code output path available, using debug chroma=" << fallback.string();
    return fallback;
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
};

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

            write_u32(out, 0x04034b50);
            write_u16(out, 20);
            write_u16(out, 0);
            write_u16(out, 0);
            write_u16(out, 0);
            write_u16(out, 0);
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
            write_u16(out, 0);
            write_u16(out, 0);
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
    size_t triangle_count,
    size_t grid_cells,
    size_t texture_count,
    size_t loaded_texture_count,
    size_t layer_count,
    const Vec3d &min_pt,
    const Vec3d &max_pt,
    int width,
    int height,
    double shell_thickness)
{
    boost::nowide::ofstream out(path.string());
    out << "FullColor Raster Debug Log\n";
    out << "output_reason=" << output_reason << "\n";
    out << "raster_strategy=closest-triangle-uniform-grid\n";
    out << "texture_sampling=bilinear\n";
    out << "uv_flip_v=" << (FULL_COLOR_OBJ_FLIP_V ? "true" : "false") << "\n";
    out << "pixel_size_mm=" << FULL_COLOR_PIXEL_SIZE_MM << "\n";
    out << "shell_thickness_mm=" << shell_thickness << "\n";
    out << "normal_tolerance_mm=" << FULL_COLOR_NORMAL_TOLERANCE_MM << "\n";
    out << "triangle_count=" << triangle_count << "\n";
    out << "grid_cells=" << grid_cells << "\n";
    out << "texture_count=" << texture_count << "\n";
    out << "loaded_texture_count=" << loaded_texture_count << "\n";
    out << "layer_count=" << layer_count << "\n";
    out << "image_width=" << width << "\n";
    out << "image_height=" << height << "\n";
    out << "bounds_min=" << min_pt.x() << "," << min_pt.y() << "," << min_pt.z() << "\n";
    out << "bounds_max=" << max_pt.x() << "," << max_pt.y() << "," << max_pt.z() << "\n";
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
    BOOST_LOG_TRIVIAL(info) << "FullColor Raster: raster_strategy=closest-triangle-uniform-grid";

    std::vector<RasterTriangle> triangles;
    triangles.reserve(output.summary.total_color_triangles);
    std::vector<TextureBitmap> textures;
    textures.reserve(output.summary.total_textures);
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
                const size_t texture_base = textures.size();
                for (const TextureImage &texture : surface.textures) {
                    TextureBitmap bitmap;
                    if (load_texture_bitmap(texture, bitmap))
                        ++loaded_texture_count;
                    textures.emplace_back(std::move(bitmap));
                }

                const Transform3d transform = instance_transform * volume->get_matrix();
                const indexed_triangle_set &its = volume->mesh().its;
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
                    tri.normal = n_raw.normalized();
                    tri.uv = triangle_data.uv;
                    tri.fallback_color = triangle_data.fallback_color;
                    tri.has_uv = triangle_data.has_uv;
                    tri.texture_index = triangle_data.texture_index >= 0 ?
                        static_cast<int>(texture_base + static_cast<size_t>(triangle_data.texture_index)) : -1;
                    triangles.emplace_back(std::move(tri));
                }
            }
        }
    }

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

    const TriangleGrid triangle_grid = build_triangle_grid(triangles, shell_thickness);
    BOOST_LOG_TRIVIAL(info) << "FullColor Raster: triangle_count=" << triangles.size()
                            << ", grid_cells=" << triangle_grid.cells.size()
                            << ", grid_cell_size=" << triangle_grid.cell_size << " mm";

    std::string output_reason;
    const boost::filesystem::path chroma_path = chroma_output_path_for_gcode(gcode_path, output_reason);
    const boost::filesystem::path staging_dir =
        boost::filesystem::temp_directory_path() / boost::filesystem::unique_path("orcaslicer_chroma_stage_%%%%-%%%%-%%%%");
    const boost::filesystem::path layer_dir = staging_dir / "layers";
    const boost::filesystem::path layer_metadata_dir = staging_dir / "layer_metadata";

    boost::filesystem::create_directories(layer_dir);
    boost::filesystem::create_directories(layer_metadata_dir);

    BOOST_LOG_TRIVIAL(info) << "FullColor Raster: staging=" << staging_dir.string();
    BOOST_LOG_TRIVIAL(info) << "FullColor Raster: chroma=" << chroma_path.string();
    BOOST_LOG_TRIVIAL(info) << "FullColor Raster: output_reason=" << output_reason;
    BOOST_LOG_TRIVIAL(info) << "FullColor Raster: layer_count=" << layers.size();

    write_debug_log(staging_dir / "debug_log.txt", output_reason, triangles.size(), triangle_grid.cells.size(),
                    textures.size(), loaded_texture_count, layers.size(), min_pt, max_pt, width, height, shell_thickness);

    if (!gcode_path.empty() && boost::filesystem::exists(gcode_path)) {
        try {
            const boost::filesystem::path gcode_src(gcode_path);
            boost::filesystem::copy_file(
                gcode_src,
                staging_dir / gcode_src.filename(),
                boost::filesystem::copy_option::overwrite_if_exists);
        } catch (const std::exception &e) {
            BOOST_LOG_TRIVIAL(error) << "FullColor Raster: failed to copy G-code into package staging: " << e.what();
        }
    } else {
        BOOST_LOG_TRIVIAL(warning) << "FullColor Raster: G-code file not available for package root: " << gcode_path;
    }

    json manifest;
    manifest["format"] = "OrcaSlicerFullColorRaster";
    manifest["version"] = 3;
    manifest["raster_strategy"] = "closest-triangle-uniform-grid";
    manifest["texture_sampling"] = "bilinear";
    manifest["uv_flip_v"] = FULL_COLOR_OBJ_FLIP_V;
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
    manifest["triangle_count"] = triangles.size();
    manifest["grid_cells"] = triangle_grid.cells.size();
    manifest["output_reason"] = output_reason;
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

                RGBA color{1.0f, 1.0f, 1.0f, 1.0f};
                if (sample_best_triangle(triangles, textures, triangle_grid, query, shell_thickness, color)) {
                    const size_t idx = (static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x)) * 3;
                    image[idx + 0] = to_u8(color[0]);
                    image[idx + 1] = to_u8(color[1]);
                    image[idx + 2] = to_u8(color[2]);
                }
            }
        }

        const std::string base = layer_basename(layer.index);
        const boost::filesystem::path png_path = layer_dir / (base + ".png");
        const boost::filesystem::path json_path = layer_metadata_dir / (base + ".json");
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
        layer_json["raster_strategy"] = "closest-triangle-uniform-grid";
        layer_json["texture_sampling"] = "bilinear";
        layer_json["uv_flip_v"] = FULL_COLOR_OBJ_FLIP_V;
        layer_json["image"] = png_path.filename().string();
        write_json_file(json_path, layer_json);

        manifest["layers"].push_back({
            {"layer_index", layer.index},
            {"z0", layer.z0},
            {"z1", layer.z1},
            {"sample_z", layer.sample_z},
            {"png", (boost::filesystem::path("layers") / png_path.filename()).string()},
            {"json", (boost::filesystem::path("layer_metadata") / json_path.filename()).string()}
        });
    }

    const boost::filesystem::path manifest_path = staging_dir / "manifest.json";
    manifest["layer_count"] = layers.size();
    manifest["package"] = {
        {"type", "chroma"},
        {"container", "zip"},
        {"output", chroma_path.string()},
        {"gcode_filename", boost::filesystem::path(gcode_path).filename().string()},
        {"layers_dir", "layers"},
        {"layer_metadata_dir", "layer_metadata"}
    };
    manifest["future_format_support_todo"] = {
        "3MF texture/color support",
        "glTF/GLB textured mesh support",
        "PLY vertex colors",
        "VRML/X3D if useful",
        "OBJ+MTL+texture is the currently proven MVP path"
    };
    write_json_file(manifest_path, manifest);

    std::string zip_error;
    const bool zip_ok = write_stored_zip_from_directory(staging_dir, chroma_path, zip_error);
    if (!zip_ok) {
        BOOST_LOG_TRIVIAL(error) << "FullColor Raster: failed to write .chroma package: " << zip_error;
        output.output_dir = staging_dir.string();
        output.manifest_path = manifest_path.string();
        output.layer_count = layers.size();
        output.generated = false;
        return output;
    }

    BOOST_LOG_TRIVIAL(info) << "FullColor Raster: chroma package written=" << chroma_path.string();

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
