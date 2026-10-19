#pragma once

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <regex>
#include <stdexcept>
#include <string>

#include <mujoco/mujoco.h>

namespace model_loader
{

inline std::filesystem::path resolve_model(std::filesystem::path model,
                                           const std::filesystem::path& model_dir)
{
    if (model.empty() || model == "default") {
        return {};
    }
    if (model == "complete") {
        model = "g1_29dof_mode_15_complete_collisions.xml";
    } else if (model == "complete-shoes") {
        model = "g1_29dof_mode_15_complete_collisions_shoes.xml";
    }
    if (model.is_relative()) {
        model = model_dir / model;
    }
    model = std::filesystem::absolute(model).lexically_normal();
    if (!std::filesystem::is_regular_file(model) || model.extension() != ".xml") {
        throw std::runtime_error("G1 model XML not found: " + model.string());
    }
    return model;
}

inline std::string escape_xml(const std::string& value)
{
    std::string escaped;
    for (char c : value) {
        switch (c) {
        case '&': escaped += "&amp;"; break;
        case '<': escaped += "&lt;"; break;
        case '>': escaped += "&gt;"; break;
        case '"': escaped += "&quot;"; break;
        case '\'': escaped += "&apos;"; break;
        default: escaped += c;
        }
    }
    return escaped;
}

inline std::string selected_scene_xml(const std::filesystem::path& scene,
                                      const std::filesystem::path& model)
{
    std::ifstream stream(scene);
    if (!stream) {
        throw std::runtime_error("Cannot read scene: " + scene.string());
    }
    std::string xml{std::istreambuf_iterator<char>(stream), {}};
    const std::regex include(R"xml((<include\s+file\s*=\s*["'])(?:\./)?g1_29dof_mode_15_complete_collisions\.xml(["']\s*/>))xml");
    std::smatch match;
    if (!std::regex_search(xml, match, include)) {
        throw std::runtime_error(
            "Model selection requires a scene including g1_29dof_mode_15_complete_collisions.xml: " + scene.string());
    }
    const auto start = match.position() + match[1].length();
    const auto length = match.length() - match[1].length() - match[2].length();
    xml.replace(start, length, escape_xml(model.generic_string()));
    return xml;
}

inline mjModel* load(const std::filesystem::path& scene,
                     const std::filesystem::path& model, char* error, int error_size)
{
    if (model.empty()) {
        return mj_loadXML(scene.c_str(), nullptr, error, error_size);
    }
    try {
        const auto xml = selected_scene_xml(scene, model);
        mjVFS vfs;
        mj_defaultVFS(&vfs);
        if (mj_addBufferVFS(&vfs, scene.c_str(), xml.data(), static_cast<int>(xml.size())) != 0) {
            mj_deleteVFS(&vfs);
            throw std::runtime_error("Cannot prepare selected model for scene: " + scene.string());
        }
        mjModel* loaded = mj_loadXML(scene.c_str(), &vfs, error, error_size);
        mj_deleteVFS(&vfs);
        return loaded;
    } catch (const std::exception& e) {
        std::snprintf(error, error_size, "%s", e.what());
        return nullptr;
    }
}

} // namespace model_loader
