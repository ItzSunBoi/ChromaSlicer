#include "ObjColorConvert.hpp"

#include "FullColorSurfaceData.hpp"
#include "libslic3r/Format/OBJ.hpp"

#include <algorithm>
#include <filesystem>
#include <unordered_map>

namespace Slic3r::FullColor {

std::shared_ptr<SurfaceData> build_surface_data_from_obj_info(
    const std::string &obj_path,
    const ObjInfo     &obj_info)
{
    auto surface_data = std::make_shared<SurfaceData>();
    surface_data->source_path       = obj_path;
    surface_data->vertex_colors     = obj_info.vertex_colors;
    surface_data->has_vertex_colors = !surface_data->vertex_colors.empty();
    surface_data->has_face_colors   = !obj_info.face_colors.empty();

    size_t triangle_count = std::max(obj_info.face_colors.size(), obj_info.uvs.size());
    for (const auto &[face_index, texture_name] : obj_info.uv_map_pngs)
        if (face_index >= 0)
            triangle_count = std::max(triangle_count, static_cast<size_t>(face_index) + 1);
    surface_data->triangles.resize(triangle_count);

    for (size_t i = 0; i < obj_info.face_colors.size(); ++i) {
        surface_data->triangles[i].fallback_color = obj_info.face_colors[i];
        surface_data->triangles[i].has_color      = true;
    }
    for (size_t i = 0; i < obj_info.uvs.size(); ++i) {
        surface_data->triangles[i].uv     = obj_info.uvs[i];
        surface_data->triangles[i].has_uv = true;
    }

    const std::filesystem::path obj_directory = std::filesystem::path(obj_path).parent_path();
    std::unordered_map<std::string, int> texture_indices;
    for (const auto &texture_entry : obj_info.pngs) {
        const std::string &texture_name = texture_entry.first;
        const std::filesystem::path texture_path(texture_name);
        TextureImage texture;
        texture.name        = texture_name;
        texture.source_path = texture_path.is_absolute() ? texture_path : obj_directory / texture_path;

        texture_indices.emplace(texture_name, static_cast<int>(surface_data->textures.size()));
        surface_data->textures.emplace_back(std::move(texture));
    }

    for (const auto &[face_index, texture_name] : obj_info.uv_map_pngs) {
        const auto texture_it = texture_indices.find(texture_name);
        if (face_index >= 0 && static_cast<size_t>(face_index) < surface_data->triangles.size() &&
            texture_it != texture_indices.end())
            surface_data->triangles[face_index].texture_index = texture_it->second;
    }

    surface_data->has_textures = !surface_data->textures.empty();
    return surface_data;
}

} // namespace Slic3r::FullColor