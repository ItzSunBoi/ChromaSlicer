#pragma once

#include "libslic3r/Point.hpp"
#include "libslic3r/Color.hpp"

#include <array>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace Slic3r::FullColor {

struct TextureImage
{
    std::string           name;
    std::filesystem::path source_path;

    int width  = 0;
    int height = 0;

    // RGBA8 pixels. Texture loading is intentionally deferred.
    std::vector<std::uint8_t> rgba;
};

struct TriangleColorData
{
    std::array<Vec2f, 3> uv;
    RGBA                 fallback_color{0.8f, 0.8f, 0.8f, 1.0f};

    int texture_index = -1;

    bool has_uv    = false;
    bool has_color = false;
};

struct SurfaceData
{
    std::vector<RGBA>              vertex_colors;
    std::vector<TriangleColorData> triangles;
    std::vector<TextureImage>      textures;

    bool has_vertex_colors = false;
    bool has_face_colors   = false;
    bool has_textures      = false;

    std::string source_path;

    bool empty() const;
};

} // namespace Slic3r::FullColor