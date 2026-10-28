#include "AppSettings.h"

#include <SDL3/SDL.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <utility>

namespace {
using json = nlohmann::json;

std::filesystem::path settings_path() {
  char* preference_path = SDL_GetPrefPath("CombatSim", "CombatSim");
  if (!preference_path) return {};

  const std::filesystem::path path =
      std::filesystem::path(preference_path) / "visual_settings.json";
  SDL_free(preference_path);
  return path;
}
}  // namespace

void load_app_settings(AppSettings& settings) {
  const auto path = settings_path();
  if (path.empty()) return;

  try {
    std::ifstream input(path);
    if (!input) return;
    const json saved = json::parse(input);
    settings.renderer = std::clamp(saved.value("renderer", settings.renderer), 0, 1);
    settings.spawn_layout = std::clamp(saved.value("spawnLayout", settings.spawn_layout), 0, 1);
    settings.arena_size_m = std::clamp(saved.value("arenaSizeM", settings.arena_size_m),
                                       min_arena_size_m, max_arena_size_m);

    const auto presets = saved.find("arenaPresets");
    if (presets != saved.end() && presets->is_array()) {
      for (const auto& preset : *presets) {
        if (!preset.is_object() || !preset.contains("name") ||
            !preset["name"].is_string() || !preset.contains("sizeM") ||
            !preset["sizeM"].is_number()) continue;
        const std::string name = preset["name"].get<std::string>();
        const float size_m = preset["sizeM"].get<float>();
        if (name.empty() || name.size() > 64 || !std::isfinite(size_m)) continue;
        settings.arena_presets.push_back({name,
          std::clamp(size_m, min_arena_size_m, max_arena_size_m)});
      }
    }

    // Migrate older bright lighting values once. Keep the saved spawn choice,
    // then write the calmer defaults in the current settings format.
    if (saved.value("lightingVersion", 0) >= 1) {
      settings.ambient_light = std::clamp(saved.value("ambientLight", settings.ambient_light), 0.0f, 0.8f);
      settings.key_light = std::clamp(saved.value("keyLight", settings.key_light), 0.0f, 2.0f);
      settings.fill_light = std::clamp(saved.value("fillLight", settings.fill_light), 0.0f, 2.0f);
      settings.camera_fill = std::clamp(saved.value("cameraFill", settings.camera_fill), 0.0f, 1.0f);
    } else {
      input.close();
      save_app_settings(settings);
    }
  } catch (const std::exception&) {
    // A malformed preference file should never block the simulator from opening.
  }
}

bool save_app_settings(const AppSettings& settings) {
  const auto path = settings_path();
  if (path.empty()) return false;

  try {
    json arena_presets = json::array();
    for (const auto& preset : settings.arena_presets) {
      if (preset.name.empty() || preset.name.size() > 64 || !std::isfinite(preset.size_m)) continue;
      arena_presets.push_back({{"name", preset.name},
        {"sizeM", std::clamp(preset.size_m, min_arena_size_m, max_arena_size_m)}});
    }

    const json saved{{"lightingVersion", 1}, {"renderer", settings.renderer},
      {"spawnLayout", settings.spawn_layout},
      {"arenaSizeM", settings.arena_size_m},
      {"arenaPresets", std::move(arena_presets)},
      {"ambientLight", settings.ambient_light}, {"keyLight", settings.key_light},
      {"fillLight", settings.fill_light}, {"cameraFill", settings.camera_fill}};
    std::ofstream output(path, std::ios::trunc);
    if (!output) return false;
    output << saved.dump(2) << '\n';
    return output.good();
  } catch (const std::exception&) {
    return false;
  }
}
