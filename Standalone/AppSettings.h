#pragma once

#include <string>
#include <vector>

#if defined(COMBATSIM_USE_FILAMENT)
inline constexpr int default_renderer_id = 1;
#else
inline constexpr int default_renderer_id = 0;
#endif

inline constexpr float min_arena_size_m = 0.75f;
inline constexpr float max_arena_size_m = 10.0f;

struct ArenaSizePreset {
  std::string name;
  float size_m = 2.0f;
};

// Settings that belong to the application, rather than to an individual bot.
// Keeping this data separate makes the UI and persistence code easier to read.
struct AppSettings {
  int renderer = default_renderer_id;  // 0 = classic, 1 = Filament PBR.
  int spawn_layout = 0;                // 0 = center line, 1 = opposite diagonal.
  float arena_size_m = 2.0f;           // Full side length of the square arena.
  std::vector<ArenaSizePreset> arena_presets;
  float ambient_light = 0.12f;
  float key_light = 0.68f;
  float fill_light = 0.24f;
  float camera_fill = 0.07f;
};

void load_app_settings(AppSettings& settings);
bool save_app_settings(const AppSettings& settings);
