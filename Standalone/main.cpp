#include <mujoco/mujoco.h>
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <shellapi.h>
#if defined(COMBATSIM_USE_FILAMENT)
#include "experimental/filament/render_context_filament.h"
#endif
#include <SDL3/SDL.h>
#include <SDL3/SDL_opengl.h>
#include "imgui.h"
#include "backends/imgui_impl_sdl3.h"
#include "backends/imgui_impl_opengl3.h"
#include "FusionBundle.h"
#include "LanSession.h"
#include "AppSettings.h"
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <string>
#include <thread>
#include <vector>

namespace {
constexpr double pi = 3.14159265358979323846;
constexpr double drive_command_ramp_seconds = 0.30;
using json = nlohmann::json;

// Runtime flow: resolve the stock model, load app and bot settings, compile the
// MuJoCo model, then run input, physics, rendering, and page UI in the main loop.
std::filesystem::path standalone_source_dir() {
  const char* executable_dir = SDL_GetBasePath();
  if (!executable_dir) return {};
  auto candidate = std::filesystem::path(executable_dir);
  for (int depth = 0; depth < 7 && !candidate.empty(); ++depth) {
    if (std::filesystem::exists(candidate / "build_filament.ps1")) return candidate;
    const auto parent = candidate.parent_path();
    if (parent == candidate) break;
    candidate = parent;
  }
  return {};
}

std::filesystem::path executable_directory() {
  std::wstring path(32768, L'\0');
  const DWORD length = GetModuleFileNameW(nullptr, path.data(), DWORD(path.size()));
  if (length == 0 || length >= path.size()) return {};
  path.resize(length);
  return std::filesystem::path(path).parent_path();
}

// Find the sample model relative to the checkout or packaged executable so
// launching CombatSim from another working directory still opens the same scene.
std::filesystem::path stock_model_path() {
  constexpr const char* model_name = "CombatRobot500g.xml";
  auto candidate = executable_directory();
  for (int depth = 0; depth < 8 && !candidate.empty(); ++depth) {
    const auto beside_app = candidate / "models" / model_name;
    if (std::filesystem::exists(beside_app)) return beside_app;
    const auto in_checkout = candidate / "Standalone" / "models" / model_name;
    if (std::filesystem::exists(in_checkout)) return in_checkout;
    const auto parent = candidate.parent_path();
    if (parent == candidate) break;
    candidate = parent;
  }
  return std::filesystem::path("Standalone/models/CombatRobot500g.xml");
}

std::filesystem::path renderer_executable(int renderer_id) {
  const auto source_dir = standalone_source_dir();
  if (source_dir.empty()) return {};
  return renderer_id == 1
      ? source_dir / "filament-build" / "bin" / "Release" / "CombatSimStandalone.exe"
      : source_dir / "build" / "Release" / "CombatSimStandalone.exe";
}

bool launch_renderer_build(int renderer_id, const std::filesystem::path& model_path,
                           std::string& error,
                           const std::filesystem::path& player_manifest = {},
                           const std::filesystem::path& opponent_manifest = {}) {
  const auto target = renderer_executable(renderer_id);
  if (target.empty()) {
    error = "Could not locate the CombatSim Standalone source folder.";
    return false;
  }
  if (!std::filesystem::exists(target)) {
    error = renderer_id == 1
        ? "Filament build not found. Run Standalone\\build_filament.ps1 first."
        : "Classic build not found. Run Standalone\\setup.ps1 first.";
    return false;
  }
  std::error_code path_error;
  const auto absolute_model = std::filesystem::absolute(model_path, path_error);
  if (path_error) {
    error = "Could not resolve the active model path for the renderer switch.";
    return false;
  }
  const auto quote = [](const std::filesystem::path& path) {
    return L"\"" + path.wstring() + L"\"";
  };
  std::wstring parameters;
  if (!player_manifest.empty()) {
    parameters = L"--renderer-resume " + quote(player_manifest) + L" " +
      (opponent_manifest.empty() ? L"\"\"" : quote(opponent_manifest));
  } else {
    parameters = quote(absolute_model);
  }
  const auto source_dir = standalone_source_dir();
  const std::wstring working_dir = source_dir.parent_path().wstring();
  const auto launched = ShellExecuteW(nullptr, L"open", target.c_str(),
                                      parameters.c_str(), working_dir.c_str(),
                                      SW_SHOWNORMAL);
  if (reinterpret_cast<INT_PTR>(launched) <= 32) {
    error = "Windows could not start the selected renderer build.";
    return false;
  }
  return true;
}

#if defined(COMBATSIM_USE_FILAMENT)
int load_filament_asset(const char* filename, void*, unsigned char** contents,
                        uint64_t* size) {
  const auto path = std::filesystem::path(COMBATSIM_FILAMENT_ASSETS_DIR) / filename;
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    std::fprintf(stderr, "Filament asset missing: %s\n", path.string().c_str());
    return 1;
  }
  file.seekg(0, std::ios::end);
  const auto end = file.tellg();
  if (end <= 0) return 1;
  *size = static_cast<uint64_t>(end);
  file.seekg(0, std::ios::beg);
  *contents = static_cast<unsigned char*>(std::malloc(*size));
  if (!*contents) return 1;
  file.read(reinterpret_cast<char*>(*contents), static_cast<std::streamsize>(*size));
  if (!file) {
    std::free(*contents);
    *contents = nullptr;
    return 1;
  }
  return 0;
}

void make_render_context(const mjModel* model, mjrContext* context) {
  mjrFilamentConfig config;
  mjr_defaultFilamentConfig(&config);
  config.load_asset = load_filament_asset;
  config.graphics_api = mjGFX_OPENGL;
  config.enable_gui = false;
  mjr_makeFilamentContext(model, context, &config);
}
#else
void make_render_context(const mjModel* model, mjrContext* context) {
  mjr_makeContext(model, context, mjFONTSCALE_100);
}
#endif

#if defined(COMBATSIM_USE_FILAMENT)
bool write_rgb_bitmap(const std::filesystem::path& path, int width, int height,
                      const std::vector<unsigned char>& rgb) {
  if (width <= 0 || height <= 0 || rgb.size() != size_t(width) * height * 3)
    return false;
  const uint32_t stride = (uint32_t(width) * 3 + 3) & ~uint32_t(3);
  const uint32_t image_size = stride * uint32_t(height);
  auto u16 = [](std::ofstream& file, uint16_t value) {
    const char bytes[2] = {char(value), char(value >> 8)};
    file.write(bytes, sizeof(bytes));
  };
  auto u32 = [](std::ofstream& file, uint32_t value) {
    const char bytes[4] = {char(value), char(value >> 8),
                           char(value >> 16), char(value >> 24)};
    file.write(bytes, sizeof(bytes));
  };
  std::ofstream file(path, std::ios::binary);
  if (!file) return false;
  file.write("BM", 2);
  u32(file, 54 + image_size); u16(file, 0); u16(file, 0); u32(file, 54);
  u32(file, 40); u32(file, uint32_t(width)); u32(file, uint32_t(height));
  u16(file, 1); u16(file, 24); u32(file, 0); u32(file, image_size);
  u32(file, 2835); u32(file, 2835); u32(file, 0); u32(file, 0);
  const uint32_t padding = stride - uint32_t(width) * 3;
  const char zeros[3] = {};
  for (int y = height - 1; y >= 0; --y) {
    for (int x = 0; x < width; ++x) {
      const size_t index = (size_t(y) * width + x) * 3;
      const char bgr[3] = {char(rgb[index + 2]), char(rgb[index + 1]),
                           char(rgb[index])};
      file.write(bgr, sizeof(bgr));
    }
    file.write(zeros, padding);
  }
  return bool(file);
}
#endif

constexpr int max_input_channels = 16;
const ImVec4 ui_ink(0.90f, 0.93f, 0.97f, 1.0f);
const ImVec4 ui_blue(0.39f, 0.70f, 1.00f, 1.0f);
const ImVec4 ui_teal(0.30f, 0.84f, 0.82f, 1.0f);
const ImVec4 ui_accent(1.00f, 0.48f, 0.22f, 1.0f);
const ImVec4 ui_green(0.40f, 0.84f, 0.60f, 1.0f);
const ImVec4 ui_violet(0.74f, 0.62f, 1.00f, 1.0f);
const ImVec4 ui_coral(1.00f, 0.57f, 0.45f, 1.0f);
const ImVec4 ui_amber(1.00f, 0.75f, 0.36f, 1.0f);
const ImVec4 ui_red(1.00f, 0.42f, 0.44f, 1.0f);

void push_button_palette(ImVec4 base, ImVec4 hovered, ImVec4 active) {
  ImGui::PushStyleColor(ImGuiCol_Button, base);
  ImGui::PushStyleColor(ImGuiCol_ButtonHovered, hovered);
  ImGui::PushStyleColor(ImGuiCol_ButtonActive, active);
}

void pop_button_palette() { ImGui::PopStyleColor(3); }

mjModel* LoadModelFromMemory(const std::string& xml,
                             const std::vector<MemoryBundleAsset>& assets,
                             char* error, int error_size) {
  if (xml.empty() || xml.size() > size_t(std::numeric_limits<int>::max())) {
    if (error && error_size > 0) std::snprintf(error, size_t(error_size), "Invalid in-memory MJCF size.");
    return nullptr;
  }
  mjVFS vfs;
  mj_defaultVFS(&vfs);
  bool ok = mj_addBufferVFS(&vfs, "session.xml", xml.data(), int(xml.size())) == 0;
  for (const auto& asset : assets) {
    if (asset.name.empty() || asset.bytes.empty() ||
        asset.bytes.size() > size_t(std::numeric_limits<int>::max()) ||
        mj_addBufferVFS(&vfs, asset.name.c_str(), asset.bytes.data(),
                        int(asset.bytes.size())) != 0) {
      ok = false;
      if (error && error_size > 0)
        std::snprintf(error, size_t(error_size), "Could not add a CAD mesh to MuJoCo's memory VFS.");
      break;
    }
  }
  mjModel* model = ok ? mj_loadXML("session.xml", &vfs, error, error_size) : nullptr;
  mj_deleteVFS(&vfs);
  return model;
}

struct InputChannel {
  bool button = false;
  int source_index = 0;
  bool inverted = false;
  float deadzone = 0.12f;
};

struct InputMixer {
  bool enabled = false;
  int channel_count = 3;
  int capture_channel = -1;  // Temporary input-detection state; never persisted.
  std::string preferred_guid;
  std::array<InputChannel, max_input_channels> channels{};
  std::map<std::string, std::array<float, max_input_channels>> outputs;
  std::map<std::string, std::array<float, max_input_channels>> output_offsets;
  std::map<std::string, std::array<float, max_input_channels>> output_expos;
};

struct Controls {
  float wheel_rpm = 4000.0f;
  bool manual_wheel_rpm_override = false;
  float weapon_rpm = 30000.0f;
  float spinup_seconds = 1.5f;
  float deadzone = 0.15f;
  double weapon_command = 0.0;
  std::vector<std::pair<int, double>> left, right;
  std::vector<int> weapon;
  std::map<int, MotorSpec> motor_models;
  std::map<int, MotorSpec> weapon_motor_models;
  std::map<int, double> filtered_wheel_speeds;
  std::map<int, int> manual_velocity_actuators;
  double battery_voltage = 0.0;
  double weapon_sign = -1.0;
};

int actuator(const mjModel* model, const char* name) {
  return mj_name2id(model, mjOBJ_ACTUATOR, name);
}

std::array<double, 2> mix_drive(float throttle, float turn,
                                double left_speed, double right_speed) {
  double left = throttle * left_speed, right = throttle * right_speed;
  if (throttle == 0.0f) {
    left = -turn * left_speed;
    right = turn * right_speed;
  } else if (turn != 0.0f) {
    if (turn * throttle > 0.0f) left = 0.0;
    else right = 0.0;
  }
  return {left, right};
}

std::array<double, 2> mix_drive(float throttle, float turn, double speed) {
  return mix_drive(throttle, turn, speed, speed);
}

float mix_output(const std::array<float, max_input_channels>& weights,
                 const std::array<float, max_input_channels>& channels,
                 int channel_count, bool unipolar = false,
                 const std::array<float, max_input_channels>* offsets = nullptr,
                 const std::array<float, max_input_channels>* expos = nullptr) {
  float output = 0.0f;
  for (int i = 0; i < std::clamp(channel_count, 0, max_input_channels); ++i) {
    const float input = channels[i];
    const float expo = expos ? std::clamp((*expos)[i], -1.0f, 1.0f) : 0.0f;
    const float curved = input + expo * (input * input * input - input);
    output += weights[i] * curved + (offsets ? (*offsets)[i] : 0.0f);
  }
  return std::clamp(output, unipolar ? 0.0f : -1.0f, 1.0f);
}

std::string motor_output_key(int group) {
  return "motor_group:" + std::to_string(group);
}

std::string ungrouped_wheel_output_key(bool left, size_t index) {
  return std::string(left ? "left_wheel:" : "right_wheel:") + std::to_string(index);
}

std::string weapon_output_key() { return "weapon"; }

int CadPartIndexFromGeom(const mjModel* model, int geom_id) {
  if (!model || geom_id < 0) return -1;
  const char* name = mj_id2name(model, mjOBJ_GEOM, geom_id);
  if (!name || std::string(name).rfind("cad_part_", 0) != 0) return -1;
  const std::string index_text = std::string(name).substr(9);
  if (index_text.empty() || !std::all_of(index_text.begin(), index_text.end(),
      [](unsigned char c) { return c >= '0' && c <= '9'; })) return -1;
  try { return std::stoi(index_text); }
  catch (...) { return -1; }
}

int SelectPreviewPartForHover(const mjModel* model, const mjData* data,
                              const mjvOption* options, double aspect,
                              double relx, double rely, const mjvScene& scene,
                              const FusionBundle& bundle, mjtNum hit[3]) {
  mjvScene pick_scene = scene;
  std::vector<mjvGeom> cad_pick_geoms;
  std::vector<int> cad_pick_order;
  if (bundle.AxisPicking()) {
    cad_pick_geoms.reserve(scene.ngeom);
    cad_pick_order.reserve(scene.ngeom);
    for (int i = 0; i < scene.ngeom; ++i) {
      const mjvGeom& geom = scene.geoms[i];
      if (geom.objtype != mjOBJ_GEOM) continue;
      const int part = CadPartIndexFromGeom(model, geom.objid);
      if (part < 0 || !bundle.PreviewVisible(part)) continue;
      cad_pick_geoms.push_back(geom);
      cad_pick_order.push_back(int(cad_pick_order.size()));
    }
    // The selected assembly is the axis destination. A shaft or hole reference
    // may be on any visible CAD part, so ray-pick CAD only and ignore decor.
    pick_scene.geoms = cad_pick_geoms.data();
    pick_scene.geomorder = cad_pick_order.data();
    pick_scene.ngeom = int(cad_pick_geoms.size());
  }
  int geom_id = -1, flex_id = -1, skin_id = -1;
  const int selected = mjv_select(model, data, options, aspect, relx, rely,
                                  &pick_scene, hit, &geom_id, &flex_id, &skin_id);
  if (selected < 0 || geom_id < 0) return -1;
  return CadPartIndexFromGeom(model, geom_id);
}

void initialize_default_mixer(InputMixer& mixer) {
  mixer = InputMixer{};
  mixer.enabled = false;
  mixer.channel_count = 3;
  mixer.channels[0] = {false, 1, true, 0.12f};  // stick up is negative on common USB pads.
  mixer.channels[1] = {false, 0, false, 0.12f};
  mixer.channels[2] = {true, 0, false, 0.0f};
}

json mixer_profile_json(const InputMixer& mixer) {
  json saved;
  saved["enabled"] = mixer.enabled;
  saved["channelCount"] = mixer.channel_count;
  saved["channels"] = json::array();
  for (const auto& channel : mixer.channels) {
    saved["channels"].push_back({{"button", channel.button},
      {"sourceIndex", channel.source_index}, {"inverted", channel.inverted},
      {"deadzone", channel.deadzone}});
  }
  saved["outputs"] = json::object();
  for (const auto& [key, weights] : mixer.outputs)
    saved["outputs"][key] = weights;
  saved["outputOffsets"] = json::object();
  for (const auto& [key, offsets] : mixer.output_offsets)
    saved["outputOffsets"][key] = offsets;
  saved["outputExpos"] = json::object();
  for (const auto& [key, expos] : mixer.output_expos)
    saved["outputExpos"][key] = expos;
  return saved;
}

void load_mixer_profile(InputMixer& mixer, const json& saved) {
  if (!saved.is_object()) return;
  mixer.enabled = saved.value("enabled", mixer.enabled);
  mixer.channel_count = std::clamp(saved.value("channelCount", mixer.channel_count), 1,
                                    max_input_channels);
  if (saved.contains("channels") && saved["channels"].is_array()) {
    const auto& channels = saved["channels"];
    for (int i = 0; i < std::min<int>(max_input_channels, channels.size()); ++i) {
      const auto& item = channels[i];
      if (!item.is_object()) continue;
      mixer.channels[i].button = item.value("button", mixer.channels[i].button);
      mixer.channels[i].source_index = std::max(0,
        item.value("sourceIndex", mixer.channels[i].source_index));
      mixer.channels[i].inverted = item.value("inverted", mixer.channels[i].inverted);
      mixer.channels[i].deadzone = std::clamp(
        item.value("deadzone", mixer.channels[i].deadzone), 0.0f, 0.95f);
    }
  }
  if (saved.contains("outputs") && saved["outputs"].is_object()) {
    for (auto it = saved["outputs"].begin(); it != saved["outputs"].end(); ++it) {
      if (!it.value().is_array()) continue;
      std::array<float, max_input_channels> weights{};
      for (int i = 0; i < std::min<int>(max_input_channels, it.value().size()); ++i)
        if (it.value()[i].is_number())
          weights[i] = std::clamp(it.value()[i].get<float>(), -1.0f, 1.0f);
      mixer.outputs[it.key()] = weights;
    }
  }
  auto load_output_adjustments = [&](const char* key, auto& destination) {
    if (!saved.contains(key) || !saved[key].is_object()) return;
    for (auto it = saved[key].begin(); it != saved[key].end(); ++it) {
      if (!it.value().is_array()) continue;
      std::array<float, max_input_channels> values{};
      for (int i = 0; i < std::min<int>(max_input_channels, it.value().size()); ++i)
        if (it.value()[i].is_number())
          values[i] = std::clamp(it.value()[i].get<float>(), -1.0f, 1.0f);
      destination[it.key()] = values;
    }
  };
  load_output_adjustments("outputOffsets", mixer.output_offsets);
  load_output_adjustments("outputExpos", mixer.output_expos);
}

std::string mixer_model_profile_key(const FusionBundle& bundle,
                                    const std::filesystem::path& model_path) {
  std::filesystem::path identity = bundle.HasBundle() && !bundle.ManifestPath().empty()
      ? bundle.ManifestPath() : model_path;
  std::error_code path_error;
  const auto absolute_identity = std::filesystem::absolute(identity, path_error);
  if (!path_error) identity = absolute_identity;
  identity = identity.lexically_normal();
  std::string key = identity.generic_u8string();
  std::transform(key.begin(), key.end(), key.begin(), [](unsigned char ch) {
    return char(std::tolower(ch));
  });
  return (bundle.HasBundle() && !bundle.ManifestPath().empty() ? "cad:" : "model:") + key;
}

std::string mixer_model_profile_name(const FusionBundle& bundle,
                                     const std::filesystem::path& model_path) {
  if (bundle.HasBundle() && !bundle.DesignName().empty()) return bundle.DesignName();
  const auto identity = bundle.HasBundle() && !bundle.ManifestPath().empty()
      ? bundle.ManifestPath() : model_path;
  const std::string name = identity.stem().string();
  return name.empty() ? "Current model" : name;
}

void load_mixer_settings(InputMixer& mixer, const std::string& model_key) {
  initialize_default_mixer(mixer);
  char* preference_path = SDL_GetPrefPath("CombatSim", "CombatSim");
  if (!preference_path) return;
  const std::filesystem::path path = std::filesystem::path(preference_path) / "input_mixer.json";
  SDL_free(preference_path);
  try {
    std::ifstream input(path);
    if (!input) return;
    const json saved = json::parse(input);
    mixer.preferred_guid = saved.value("preferredGuid", std::string{});
    const auto profiles = saved.find("profiles");
    if (profiles != saved.end() && profiles->is_object()) {
      const auto profile = profiles->find(model_key);
      if (profile != profiles->end()) load_mixer_profile(mixer, *profile);
    } else {
      // Older builds stored one global mixer. Keep it as the initial model's
      // profile so existing controller setups survive this format change.
      load_mixer_profile(mixer, saved);
    }
  } catch (const std::exception&) {
    // A malformed preference file should not prevent the simulator from starting.
  }
}

bool save_mixer_settings(const InputMixer& mixer, const std::string& model_key) {
  char* preference_path = SDL_GetPrefPath("CombatSim", "CombatSim");
  if (!preference_path) return false;
  const std::filesystem::path path = std::filesystem::path(preference_path) / "input_mixer.json";
  SDL_free(preference_path);
  json saved = json::object();
  try {
    std::ifstream input(path);
    if (input) {
      json existing = json::parse(input);
      if (existing.is_object()) saved = std::move(existing);
    }
  } catch (const std::exception&) {
    // Replace a damaged preferences file with the current valid controller setup.
  }
  if (!saved.contains("profiles") || !saved["profiles"].is_object())
    saved["profiles"] = json::object();
  saved["formatVersion"] = 2;
  saved["activeModel"] = model_key;
  saved["preferredGuid"] = mixer.preferred_guid;
  saved["profiles"][model_key] = mixer_profile_json(mixer);
  std::ofstream output(path, std::ios::trunc);
  if (!output) return false;
  output << saved.dump(2) << '\n';
  return bool(output);
}

void bind_stock_actuators(const mjModel* model, Controls& c) {
  c.left.clear(); c.right.clear(); c.weapon.clear();
  c.motor_models.clear(); c.weapon_motor_models.clear();
  c.manual_velocity_actuators.clear(); c.battery_voltage = 0.0;
  for (const auto* name : {"front_left_drive", "rear_left_drive"}) {
    const int id = actuator(model, name); if (id >= 0) c.left.emplace_back(id, 1.0);
  }
  for (const auto* name : {"front_right_drive", "rear_right_drive"}) {
    const int id = actuator(model, name); if (id >= 0) c.right.emplace_back(id, 1.0);
  }
  const int id = actuator(model, "weapon_drive");
  if (id >= 0) c.weapon.push_back(id);
  c.weapon_sign = -1.0;
}

float axis_with_deadzone(SDL_Gamepad* pad, SDL_GamepadAxis axis, float deadzone) {
  if (!pad) return 0.0f;
  float value = SDL_GetGamepadAxis(pad, axis) / 32767.0f;
  return std::abs(value) > deadzone ? std::clamp(value, -1.0f, 1.0f) : 0.0f;
}

struct SideMotorSpeedEstimate {
  double rpm = 424.0;
  int modeled_wheels = 0;
};

SideMotorSpeedEstimate side_motor_speed_estimate(
    const std::vector<std::pair<int, double>>& wheels,
    const std::map<int, MotorSpec>& motors, double battery_voltage) {
  SideMotorSpeedEstimate estimate;
  double slowest_motor_rpm = std::numeric_limits<double>::infinity();
  for (const auto& [actuator_id, direction] : wheels) {
    (void)direction;
    const auto it = motors.find(actuator_id);
    if (it == motors.end()) continue;
    const double rpm = MotorNoLoadOutputRpm(it->second, battery_voltage);
    if (rpm <= 0.0) continue;
    slowest_motor_rpm = std::min(slowest_motor_rpm, rpm);
    ++estimate.modeled_wheels;
  }
  if (estimate.modeled_wheels > 0) estimate.rpm = slowest_motor_rpm;
  return estimate;
}

double motor_group_torque_per_wheel(const MotorSpec& motor, double battery_voltage,
                                    double desired_output_speed,
                                    double actual_output_speed, size_t wheel_count) {
  if (wheel_count == 0) return 0.0;
  return MotorOutputTorque(motor, battery_voltage, desired_output_speed,
                             actual_output_speed) / static_cast<double>(wheel_count);
}

void apply_wheel_drive_targets(const mjModel* model, mjData* data, Controls& controls,
                               double left_speed, double right_speed,
                               const std::map<int, double>* custom_speeds = nullptr) {
  struct PendingMotorGroup {
    MotorSpec motor;
    std::vector<std::pair<int, double>> members;
    double desired_speed_sum = 0.0;
    double actual_speed_sum = 0.0;
  };
  std::map<int, PendingMotorGroup> pending_motor_groups;
  auto set_wheel_actuator = [&](int id, double forward_sign, double desired_forward_speed) {
    const auto motor = controls.motor_models.find(id);
    const auto manual_actuator = controls.manual_velocity_actuators.find(id);
    if (motor == controls.motor_models.end()) {
      controls.filtered_wheel_speeds.erase(id);
      const double desired_joint_speed = forward_sign * desired_forward_speed;
      data->ctrl[id] = desired_joint_speed;
      return;
    }
    if (controls.manual_wheel_rpm_override &&
        manual_actuator != controls.manual_velocity_actuators.end()) {
      controls.filtered_wheel_speeds[id] = desired_forward_speed;
      const double desired_joint_speed = forward_sign * desired_forward_speed;
      data->ctrl[id] = 0.0;
      data->ctrl[manual_actuator->second] = desired_joint_speed;
      return;
    }
    const double no_load_rpm = MotorNoLoadOutputRpm(motor->second, controls.battery_voltage);
    const double reference_speed = std::max(std::abs(desired_forward_speed),
      no_load_rpm * 2.0 * pi / 60.0);
    const double max_speed_step = reference_speed * model->opt.timestep /
      drive_command_ramp_seconds;
    double& filtered_speed = controls.filtered_wheel_speeds[id];
    filtered_speed += std::clamp(desired_forward_speed - filtered_speed,
      -max_speed_step, max_speed_step);
    desired_forward_speed = filtered_speed;
    const double desired_joint_speed = forward_sign * desired_forward_speed;
    if (manual_actuator != controls.manual_velocity_actuators.end()) {
      const int joint = model->actuator_trnid[2 * id];
      if (joint >= 0 && joint < model->njnt) {
        const int dof = model->jnt_dofadr[joint];
        data->ctrl[manual_actuator->second] = data->qvel[dof];
      }
    }
    if (model->actuator_trntype[id] != mjTRN_JOINT &&
        model->actuator_trntype[id] != mjTRN_JOINTINPARENT) {
      data->ctrl[id] = 0.0;
      return;
    }
    const int joint = model->actuator_trnid[2 * id];
    if (joint < 0 || joint >= model->njnt || model->jnt_type[joint] != mjJNT_HINGE) {
      data->ctrl[id] = 0.0;
      return;
    }
    const int dof = model->jnt_dofadr[joint];
    auto& motor_group = pending_motor_groups[motor->second.drive_group];
    if (motor_group.members.empty()) motor_group.motor = motor->second;
    motor_group.members.emplace_back(id, forward_sign);
    motor_group.desired_speed_sum += desired_forward_speed;
    motor_group.actual_speed_sum += forward_sign * data->qvel[dof];
  };
  for (const auto& [id, direction] : controls.left) {
    double speed = left_speed;
    if (custom_speeds) {
      const auto custom = custom_speeds->find(id);
      if (custom != custom_speeds->end()) speed = custom->second;
    }
    set_wheel_actuator(id, direction, speed);
  }
  for (const auto& [id, direction] : controls.right) {
    double speed = right_speed;
    if (custom_speeds) {
      const auto custom = custom_speeds->find(id);
      if (custom != custom_speeds->end()) speed = custom->second;
    }
    set_wheel_actuator(id, direction, speed);
  }
  for (auto& [group_id, motor_group] : pending_motor_groups) {
    (void)group_id;
    if (motor_group.members.empty()) continue;
    const double member_count = static_cast<double>(motor_group.members.size());
    const double group_torque = motor_group_torque_per_wheel(
      motor_group.motor, controls.battery_voltage,
      motor_group.desired_speed_sum / member_count,
      motor_group.actual_speed_sum / member_count, motor_group.members.size());
    for (const auto& [id, forward_sign] : motor_group.members)
      data->ctrl[id] = forward_sign * group_torque;
  }
}

bool weapon_uses_torque_model(const Controls& controls) {
  return !controls.weapon.empty() &&
    std::all_of(controls.weapon.begin(), controls.weapon.end(), [&](int id) {
      return controls.weapon_motor_models.count(id) != 0;
    });
}

double weapon_no_load_rpm(const Controls& controls) {
  if (controls.weapon.empty()) return 0.0;
  double estimate = std::numeric_limits<double>::infinity();
  for (const int id : controls.weapon) {
    const auto motor = controls.weapon_motor_models.find(id);
    if (motor == controls.weapon_motor_models.end()) return 0.0;
    estimate = std::min(estimate,
      MotorNoLoadOutputRpm(motor->second, controls.battery_voltage));
  }
  return std::isfinite(estimate) ? estimate : 0.0;
}

void apply_weapon_drive_targets(const mjModel* model, mjData* data,
                                Controls& controls, float weapon_throttle) {
  weapon_throttle = std::clamp(weapon_throttle, 0.0f, 1.0f);
  const bool all_motorized = weapon_uses_torque_model(controls);
  const double max_weapon_speed = controls.weapon_rpm * 2.0 * pi / 60.0;
  if (all_motorized) {
    controls.weapon_command = 0.0;
  } else {
    const double target = max_weapon_speed * weapon_throttle;
    const double delta = max_weapon_speed * model->opt.timestep /
      std::max(0.01f, controls.spinup_seconds);
    controls.weapon_command += std::clamp(target - controls.weapon_command, -delta, delta);
  }

  for (const int id : controls.weapon) {
    const auto motor = controls.weapon_motor_models.find(id);
    if (motor != controls.weapon_motor_models.end()) {
      if (weapon_throttle <= 1e-4f) {
        // Zero commanded motor voltage lets the weapon coast down under its
        // simulated inertia and mechanical losses.
        data->ctrl[id] = 0.0;
        continue;
      }
      if ((model->actuator_trntype[id] != mjTRN_JOINT &&
           model->actuator_trntype[id] != mjTRN_JOINTINPARENT)) {
        data->ctrl[id] = 0.0;
        continue;
      }
      const int joint = model->actuator_trnid[2 * id];
      if (joint < 0 || joint >= model->njnt || model->jnt_type[joint] != mjJNT_HINGE) {
        data->ctrl[id] = 0.0;
        continue;
      }
      const int dof = model->jnt_dofadr[joint];
      const double free_speed = MotorNoLoadOutputRpm(motor->second,
        controls.battery_voltage) * 2.0 * pi / 60.0;
      const double actual_speed = controls.weapon_sign * data->qvel[dof];
      const double torque = MotorOutputTorque(motor->second,
        controls.battery_voltage, free_speed * weapon_throttle, actual_speed);
      data->ctrl[id] = controls.weapon_sign * torque;
    } else {
      data->ctrl[id] = controls.weapon_sign * controls.weapon_command;
    }
  }
}

double actuator_rpm(const mjModel* model, const mjData* data, int actuator_id,
                    double direction = 1.0) {
  if (actuator_id < 0 || actuator_id >= model->nu ||
      (model->actuator_trntype[actuator_id] != mjTRN_JOINT &&
       model->actuator_trntype[actuator_id] != mjTRN_JOINTINPARENT))
    return std::nan("");
  const int joint_id = model->actuator_trnid[2 * actuator_id];
  if (joint_id < 0 || joint_id >= model->njnt || model->jnt_type[joint_id] != mjJNT_HINGE)
    return std::nan("");
  const int dof = model->jnt_dofadr[joint_id];
  return direction * data->qvel[dof] * 60.0 / (2.0 * pi);
}

struct WheelSlip {
  bool valid = false;
  bool has_ground_contact = false;
  double speed = 0.0;
};

bool body_is_in_subtree(const mjModel* model, int body, int root) {
  for (int current = body; current > 0; current = model->body_parentid[current])
    if (current == root) return true;
  return false;
}

using Vec3 = std::array<double, 3>;
using ScreenPoint = std::array<double, 2>;
constexpr std::array<Vec3, 3> world_axes{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
constexpr std::array<std::array<Vec3, 2>, 3> ring_basis{{
  {{{0, 1, 0}, {0, 0, 1}}},
  {{{0, 0, 1}, {1, 0, 0}}},
  {{{1, 0, 0}, {0, 1, 0}}}
}};

double dot3(const Vec3& a, const Vec3& b) {
  return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

Vec3 cross3(const Vec3& a, const Vec3& b) {
  return {a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2],
          a[0]*b[1] - a[1]*b[0]};
}

bool project_to_screen(const mjModel* model, const mjvScene& scene,
                       int width, int height, const Vec3& point, ScreenPoint& screen) {
  if (!model || width <= 0 || height <= 0) return false;
  const mjvGLCamera& camera = scene.camera[0];
  const Vec3 position{camera.pos[0], camera.pos[1], camera.pos[2]};
  Vec3 forward{camera.forward[0], camera.forward[1], camera.forward[2]};
  Vec3 up{camera.up[0], camera.up[1], camera.up[2]};
  const double forward_length = std::sqrt(dot3(forward, forward));
  const double up_length = std::sqrt(dot3(up, up));
  if (forward_length < 1e-9 || up_length < 1e-9) return false;
  for (double& value : forward) value /= forward_length;
  for (double& value : up) value /= up_length;
  Vec3 right = cross3(forward, up);
  const double right_length = std::sqrt(dot3(right, right));
  if (right_length < 1e-9) return false;
  for (double& value : right) value /= right_length;
  const Vec3 relative{point[0] - position[0], point[1] - position[1], point[2] - position[2]};
  const double depth = dot3(relative, forward);
  if (depth <= std::max(1e-5, double(camera.frustum_near) * 0.25)) return false;
  const double fovy = model->vis.global.fovy;
  if (!std::isfinite(fovy) || fovy <= 1e-4) return false;
  const double focal = camera.orthographic
    ? height / fovy
    : height / (2.0 * std::tan(fovy * pi / 360.0));
  const double horizontal = dot3(relative, right);
  const double vertical = dot3(relative, up);
  const double divisor = camera.orthographic ? 1.0 : depth;
  screen = {width * 0.5 + horizontal * focal / divisor,
            height * 0.5 - vertical * focal / divisor};
  return std::isfinite(screen[0]) && std::isfinite(screen[1]);
}

double point_segment_distance(const ScreenPoint& point, const ScreenPoint& first,
                              const ScreenPoint& second, double* fraction = nullptr) {
  const double dx = second[0] - first[0], dy = second[1] - first[1];
  const double length2 = dx*dx + dy*dy;
  const double t = length2 > 1e-9
    ? std::clamp(((point[0] - first[0])*dx + (point[1] - first[1])*dy) / length2, 0.0, 1.0)
    : 0.0;
  if (fraction) *fraction = t;
  const double ex = point[0] - (first[0] + t*dx);
  const double ey = point[1] - (first[1] + t*dy);
  return std::sqrt(ex*ex + ey*ey);
}

Vec3 ring_point(const Vec3& center, int axis, double radius, double angle) {
  const auto& basis = ring_basis[axis];
  return {center[0] + radius*(std::cos(angle)*basis[0][0] + std::sin(angle)*basis[1][0]),
          center[1] + radius*(std::cos(angle)*basis[0][1] + std::sin(angle)*basis[1][1]),
          center[2] + radius*(std::cos(angle)*basis[0][2] + std::sin(angle)*basis[1][2])};
}

struct GizmoHit {
  int axis = -1;
  double distance = std::numeric_limits<double>::infinity();
  double angle = 0.0;
};

GizmoHit pick_translation_axis(const mjModel* model, const mjvScene& scene,
                               int width, int height, const Vec3& center,
                               double length, const ScreenPoint& mouse) {
  GizmoHit best;
  ScreenPoint start;
  if (!project_to_screen(model, scene, width, height, center, start)) return best;
  for (int axis = 0; axis < 3; ++axis) {
    const Vec3 end_world{center[0] + length*world_axes[axis][0],
                         center[1] + length*world_axes[axis][1],
                         center[2] + length*world_axes[axis][2]};
    ScreenPoint end;
    if (!project_to_screen(model, scene, width, height, end_world, end)) continue;
    const double distance = point_segment_distance(mouse, start, end);
    if (distance < best.distance) best = {axis, distance, 0.0};
  }
  return best;
}

GizmoHit pick_rotation_ring(const mjModel* model, const mjvScene& scene,
                            int width, int height, const Vec3& center,
                            double radius, const ScreenPoint& mouse, int only_axis = -1) {
  GizmoHit best;
  constexpr int segments = 96;
  for (int axis = 0; axis < 3; ++axis) {
    if (only_axis >= 0 && axis != only_axis) continue;
    ScreenPoint previous;
    if (!project_to_screen(model, scene, width, height,
                           ring_point(center, axis, radius, 0.0), previous)) continue;
    for (int i = 0; i < segments; ++i) {
      const double next_angle = 2.0*pi*(i + 1)/segments;
      ScreenPoint next;
      if (!project_to_screen(model, scene, width, height,
                             ring_point(center, axis, radius, next_angle), next)) continue;
      double fraction = 0.0;
      const double distance = point_segment_distance(mouse, previous, next, &fraction);
      if (distance < best.distance)
        best = {axis, distance, 2.0*pi*(i + fraction)/segments};
      previous = next;
    }
  }
  return best;
}

int free_joint_for_body(const mjModel* model, int body) {
  if (body <= 0 || body >= model->nbody) return -1;
  const int first = model->body_jntadr[body];
  for (int i = 0; i < model->body_jntnum[body]; ++i) {
    const int joint = first + i;
    if (model->jnt_type[joint] == mjJNT_FREE) return joint;
  }
  return -1;
}

void robot_geometry_bounds(const mjModel* model, const mjData* data, int root_body,
                           Vec3& low, Vec3& high) {
  low = {std::numeric_limits<double>::infinity(),
         std::numeric_limits<double>::infinity(),
         std::numeric_limits<double>::infinity()};
  high = {-std::numeric_limits<double>::infinity(),
          -std::numeric_limits<double>::infinity(),
          -std::numeric_limits<double>::infinity()};
  for (int geom = 0; geom < model->ngeom; ++geom) {
    int body = model->geom_bodyid[geom];
    while (body > 0 && body != root_body) body = model->body_parentid[body];
    if (body != root_body) continue;
    const double radius = std::max(0.0, model->geom_rbound[geom]);
    for (int axis = 0; axis < 3; ++axis) {
      const double center = data->geom_xpos[3 * geom + axis];
      low[axis] = std::min(low[axis], center - radius);
      high[axis] = std::max(high[axis], center + radius);
    }
  }
  if (!std::isfinite(low[0])) {
    low = high = {data->xpos[3 * root_body], data->xpos[3 * root_body + 1],
                  data->xpos[3 * root_body + 2]};
  }
}

Vec3 robot_geometry_center(const mjModel* model, const mjData* data, int root_body) {
  Vec3 low{}, high{};
  robot_geometry_bounds(model, data, root_body, low, high);
  return {(low[0] + high[0]) * 0.5, (low[1] + high[1]) * 0.5,
          (low[2] + high[2]) * 0.5};
}

double clamp_arena_size_m(double size_m) {
  return std::clamp(size_m, double(min_arena_size_m), double(max_arena_size_m));
}

void set_arena_geom(mjModel* model, const char* name, Vec3 position, Vec3 size) {
  const int geom = mj_name2id(model, mjOBJ_GEOM, name);
  if (geom < 0 || geom >= model->ngeom) return;
  for (int axis = 0; axis < 3; ++axis) {
    model->geom_pos[3 * geom + axis] = position[axis];
    model->geom_size[3 * geom + axis] = size[axis];
    model->geom_aabb[6 * geom + axis] = 0.0;
    model->geom_aabb[6 * geom + axis + 3] = size[axis];
  }
  // Keep MuJoCo's broadphase radius in sync with the changed box dimensions.
  model->geom_rbound[geom] = std::sqrt(size[0] * size[0] + size[1] * size[1] +
                                       size[2] * size[2]);
}

void apply_arena_size(mjModel* model, double size_m) {
  if (!model) return;
  const double half = clamp_arena_size_m(size_m) * 0.5;
  constexpr double floor_visual_thickness = 0.001;
  set_arena_geom(model, "floor_visual", {0, 0, -floor_visual_thickness},
                 {half, half, floor_visual_thickness});

  constexpr double catch_depth = 0.30;
  constexpr double catch_margin = 0.02;
  constexpr double panel_thickness = 0.004;
  constexpr double wall_height = 0.55;
  const double wall_center = half + catch_depth;
  const double panel_center = half + panel_thickness;
  // Extend the side-wall ends across the full outer depth so the four boxes
  // overlap at arena corners. A short tangential span leaves diagonal escape gaps.
  const double catch_outer_half_extent = half + 2.0 * catch_depth + catch_margin;
  set_arena_geom(model, "north_wall", {0, wall_center, wall_height},
                 {catch_outer_half_extent, catch_depth, wall_height});
  set_arena_geom(model, "south_wall", {0, -wall_center, wall_height},
                 {catch_outer_half_extent, catch_depth, wall_height});
  set_arena_geom(model, "east_wall", {wall_center, 0, wall_height},
                 {catch_depth, catch_outer_half_extent, wall_height});
  set_arena_geom(model, "west_wall", {-wall_center, 0, wall_height},
                 {catch_depth, catch_outer_half_extent, wall_height});
  set_arena_geom(model, "north_wall_acrylic", {0, panel_center, wall_height},
                 {half, panel_thickness, wall_height});
  set_arena_geom(model, "south_wall_acrylic", {0, -panel_center, wall_height},
                 {half, panel_thickness, wall_height});
  set_arena_geom(model, "east_wall_acrylic", {panel_center, 0, wall_height},
                 {panel_thickness, half, wall_height});
  set_arena_geom(model, "west_wall_acrylic", {-panel_center, 0, wall_height},
                 {panel_thickness, half, wall_height});
  // Hidden wall sides rise to 1.05 m and this roof starts at 0.95 m, leaving
  // a 10 cm overlap to close the wall/roof seam. The clear visual panels stay
  // at the user's 1 m arena height.
  set_arena_geom(model, "roof", {0, 0, 1.125},
                 {catch_outer_half_extent, catch_outer_half_extent, 0.175});
  set_arena_geom(model, "roof_acrylic", {0, 0, 1.004}, {half, half, panel_thickness});
}

void apply_arena_spawn_layout(const mjModel* model, mjData* data, int layout,
                              double arena_size_m) {
  if (!model || !data) return;
  mj_resetData(model, data);
  mj_forward(model, data);
  const int roots[2] = {mj_name2id(model, mjOBJ_BODY, "Player_Robot"),
                        mj_name2id(model, mjOBJ_BODY, "Target_Robot")};
  const double diagonal_turn = layout == 1 ? pi / 4.0 : 0.0;
  for (int root : roots) {
    const int joint = free_joint_for_body(model, root);
    if (joint < 0) continue;
    const int qpos = model->jnt_qposadr[joint];
    const double half = diagonal_turn * 0.5;
    const mjtNum delta[4] = {std::cos(half), 0.0, 0.0, std::sin(half)};
    mjtNum rotated[4];
    mju_mulQuat(rotated, delta, data->qpos + qpos + 3);
    const double norm = std::sqrt(rotated[0] * rotated[0] + rotated[1] * rotated[1] +
                                  rotated[2] * rotated[2] + rotated[3] * rotated[3]);
    if (norm > 1e-12 && std::isfinite(norm))
      for (int i = 0; i < 4; ++i) data->qpos[qpos + 3 + i] = rotated[i] / norm;
  }
  mj_forward(model, data);
  std::array<Vec3, 2> lows{}, highs{};
  double maximum_half_extent = 0.0;
  for (int i = 0; i < 2; ++i) {
    robot_geometry_bounds(model, data, roots[i], lows[i], highs[i]);
    maximum_half_extent = std::max(maximum_half_extent,
      std::max((highs[i][0] - lows[i][0]) * 0.5,
               (highs[i][1] - lows[i][1]) * 0.5));
  }
  constexpr double wall_clearance = 0.01;
  const double arena_inner_half_extent = clamp_arena_size_m(arena_size_m) * 0.5;
  const double safe_offset = std::max(0.0, arena_inner_half_extent - wall_clearance - maximum_half_extent);
  const double preferred_offset = arena_inner_half_extent * (layout == 1 ? 0.42 : 0.55);
  const double offset = std::min(preferred_offset, safe_offset);
  const double targets[2][2] = {{-offset, layout == 1 ? -offset : 0.0},
                                 { offset, layout == 1 ?  offset : 0.0}};
  for (int i = 0; i < 2; ++i) {
    const int root = roots[i];
    const int joint = free_joint_for_body(model, root);
    if (joint < 0) continue;
    const int qpos = model->jnt_qposadr[joint];
    const Vec3 center = robot_geometry_center(model, data, root);
    data->qpos[qpos] += targets[i][0] - center[0];
    data->qpos[qpos + 1] += targets[i][1] - center[1];
  }
  mj_forward(model, data);
}

void apply_arena_lighting(mjModel* model, const AppSettings& settings) {
  if (!model) return;
  for (int i = 0; i < 3; ++i) {
    model->vis.headlight.ambient[i] = settings.ambient_light;
    model->vis.headlight.diffuse[i] = settings.camera_fill;
    model->vis.headlight.specular[i] = settings.camera_fill * 0.12f;
  }
  model->vis.headlight.active = 1;
  auto set_light = [&](const char* name, float strength,
                       const std::array<float, 3>& diffuse,
                       float specular) {
    const int id = mj_name2id(model, mjOBJ_LIGHT, name);
    if (id < 0 || id >= model->nlight) return;
    model->light_active[id] = strength > 0.001f;
    // Transparent arena skins still write into MuJoCo's shadow map and leave
    // hard-edged, overlapping light patches across the MDF floor.
    model->light_castshadow[id] = 0;
    for (int channel = 0; channel < 3; ++channel) {
      model->light_diffuse[3 * id + channel] = diffuse[channel] * strength;
      model->light_specular[3 * id + channel] = specular * strength;
    }
  };
  set_light("arena_key_light", settings.key_light, {0.88f, 0.85f, 0.78f}, 0.18f);
  set_light("arena_fill_light", settings.fill_light, {0.42f, 0.48f, 0.54f}, 0.035f);
}

#if defined(COMBATSIM_USE_FILAMENT)
void apply_filament_material_defaults(mjModel* model) {
  if (!model) return;
  std::vector<bool> material_has_uv_mesh(model->nmat, false);
  for (int geom_id = 0; geom_id < model->ngeom; ++geom_id) {
    if (model->geom_type[geom_id] != mjGEOM_MESH) continue;
    const int material_id = model->geom_matid[geom_id];
    // geom_dataid is the mesh ID directly for mjGEOM_MESH.
    const int mesh_id = model->geom_dataid[geom_id];
    if (material_id >= 0 && material_id < model->nmat &&
        mesh_id >= 0 && mesh_id < model->nmesh &&
        model->mesh_texcoordadr[mesh_id] >= 0)
      material_has_uv_mesh[material_id] = true;
  }
  for (int i = 0; i < model->nmat; ++i) {
    // This PBR material samples Fusion's OBJ UVs. MuJoCo primitive shapes and
    // legacy STL-only bundles have no UV attribute, so leave them on the
    // regular Filament Phong material and avoid incomplete vertex layouts.
    if (!material_has_uv_mesh[i]) {
      model->mat_metallic[i] = -1.0f;
      model->mat_roughness[i] = -1.0f;
      continue;
    }
    const char* material_name = mj_id2name(model, mjOBJ_MATERIAL, i);
    std::string name = material_name ? material_name : "";
    std::transform(name.begin(), name.end(), name.begin(),
      [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
    const bool metal = name.find("steel") != std::string::npos ||
      name.find("aluminum") != std::string::npos ||
      name.find("aluminium") != std::string::npos ||
      name.find("titanium") != std::string::npos ||
      name.find("metal") != std::string::npos ||
      name.find("7075") != std::string::npos;
    if (model->mat_metallic[i] < 0.0f) {
      const bool painted = name.find("paint") != std::string::npos ||
                           name.find("alloy") != std::string::npos;
      model->mat_metallic[i] = metal ? (painted ? 0.18f : 0.82f) : 0.0f;
    }
    if (model->mat_roughness[i] < 0.0f) {
      float roughness = std::clamp(1.0f - 0.80f * model->mat_shininess[i],
                                   0.22f, 0.92f);
      if (name.find("floor") != std::string::npos ||
          name.find("mdf") != std::string::npos) roughness = 0.78f;
      else if (name.find("rubber") != std::string::npos ||
               name.find("silicone") != std::string::npos) roughness = 0.82f;
      else if (name.find("steel") != std::string::npos) roughness = 0.26f;
      model->mat_roughness[i] = roughness;
    }
  }
}
#endif

bool translate_free_body_axis(const mjModel* model, mjData* data, int body,
                              int axis, double distance) {
  if (axis < 0 || axis >= 3 || !std::isfinite(distance)) return false;
  const int joint = free_joint_for_body(model, body);
  if (joint < 0) return false;
  const int qpos = model->jnt_qposadr[joint];
  const int dof = model->jnt_dofadr[joint];
  for (int i = 0; i < 3; ++i) data->qpos[qpos + i] += world_axes[axis][i] * distance;
  for (int i = 0; i < 6; ++i) data->qvel[dof + i] = 0.0;
  mj_forward(model, data);
  return true;
}

bool rotate_free_body_axis(const mjModel* model, mjData* data, int body, int axis,
                           double angle, const Vec3& pivot, const Vec3& local_pivot) {
  if (axis < 0 || axis >= 3 || !std::isfinite(angle)) return false;
  const int joint = free_joint_for_body(model, body);
  if (joint < 0) return false;
  const int qpos = model->jnt_qposadr[joint];
  const int dof = model->jnt_dofadr[joint];
  const double half = angle * 0.5;
  const mjtNum delta_quat[4] = {std::cos(half),
    world_axes[axis][0]*std::sin(half), world_axes[axis][1]*std::sin(half),
    world_axes[axis][2]*std::sin(half)};
  mjtNum next_quat[4];
  mju_mulQuat(next_quat, delta_quat, data->qpos + qpos + 3);
  const double norm = std::sqrt(next_quat[0]*next_quat[0] + next_quat[1]*next_quat[1] +
                                next_quat[2]*next_quat[2] + next_quat[3]*next_quat[3]);
  if (!std::isfinite(norm) || norm < 1e-9) return false;
  for (int i = 0; i < 4; ++i) data->qpos[qpos + 3 + i] = next_quat[i] / norm;
  mjtNum rotated_pivot[3];
  mju_rotVecQuat(rotated_pivot, local_pivot.data(), data->qpos + qpos + 3);
  for (int i = 0; i < 3; ++i) data->qpos[qpos + i] = pivot[i] - rotated_pivot[i];
  for (int i = 0; i < 6; ++i) data->qvel[dof + i] = 0.0;
  mj_forward(model, data);
  return true;
}

void add_transform_gizmo(const mjModel* model, const mjData* data, int body,
                         bool rotation_mode, int active_axis, double length,
                         mjvScene& scene) {
  if (body <= 0 || body >= model->nbody || scene.ngeom >= scene.maxgeom) return;
  const Vec3 center{data->subtree_com[3*body], data->subtree_com[3*body + 1],
                    data->subtree_com[3*body + 2]};
  const float colors[3][4] = {{0.96f, 0.18f, 0.16f, 1.0f},
                              {0.20f, 0.86f, 0.28f, 1.0f},
                              {0.20f, 0.48f, 1.0f, 1.0f}};
  const float selected[4] = {1.0f, 0.78f, 0.18f, 1.0f};
  auto connector = [&](int type, double width, const Vec3& from, const Vec3& to,
                       const float* rgba) {
    if (scene.ngeom >= scene.maxgeom) return;
    mjtNum a[3] = {from[0], from[1], from[2]};
    mjtNum b[3] = {to[0], to[1], to[2]};
    const int index = scene.ngeom++;
    scene.geomorder[index] = index;
    mjvGeom& geom = scene.geoms[index];
    mjv_initGeom(&geom, type, nullptr, nullptr, nullptr, rgba);
    mjv_connector(&geom, type, width, a, b);
    geom.objtype = mjOBJ_UNKNOWN;
    geom.objid = -1;
    geom.category = mjCAT_DECOR;
  };
  if (!rotation_mode) {
    const mjtNum marker_size[3] = {0.018, 0.018, 0.018};
    if (scene.ngeom < scene.maxgeom) {
      mjtNum origin[3] = {center[0], center[1], center[2]};
      const int index = scene.ngeom++;
      scene.geomorder[index] = index;
      mjvGeom& marker = scene.geoms[index];
      mjv_initGeom(&marker, mjGEOM_SPHERE, marker_size, origin, nullptr, selected);
      marker.objtype = mjOBJ_UNKNOWN;
      marker.objid = -1;
      marker.category = mjCAT_DECOR;
    }
    for (int axis = 0; axis < 3; ++axis) {
      const Vec3 end{center[0] + length*world_axes[axis][0],
                     center[1] + length*world_axes[axis][1],
                     center[2] + length*world_axes[axis][2]};
      const float* color = axis == active_axis ? selected : colors[axis];
      connector(mjGEOM_ARROW, 0.014, center, end, color);
    }
  } else {
    constexpr int segments = 48;
    const double radius = length * 0.82;
    for (int axis = 0; axis < 3; ++axis) {
      const float* color = axis == active_axis ? selected : colors[axis];
      Vec3 previous = ring_point(center, axis, radius, 0.0);
      for (int i = 1; i <= segments; ++i) {
        const Vec3 next = ring_point(center, axis, radius, 2.0*pi*i/segments);
        connector(mjGEOM_LINE, 3.0, previous, next, color);
        previous = next;
      }
    }
  }
}

WheelSlip wheel_slip(const mjModel* model, const mjData* data, int actuator_id) {
  WheelSlip result;
  if (actuator_id < 0 || actuator_id >= model->nu ||
      (model->actuator_trntype[actuator_id] != mjTRN_JOINT &&
       model->actuator_trntype[actuator_id] != mjTRN_JOINTINPARENT))
    return result;
  const int joint = model->actuator_trnid[2 * actuator_id];
  if (joint < 0 || joint >= model->njnt || model->jnt_type[joint] != mjJNT_HINGE)
    return result;
  const int wheel_root = model->jnt_bodyid[joint];
  std::vector<mjtNum> jacp(3 * model->nv), jacr(3 * model->nv);
  for (int i = 0; i < data->ncon; ++i) {
    const mjContact& contact = data->contact[i];
    if (contact.dist > 0.003) continue;
    const int geom0 = contact.geom[0], geom1 = contact.geom[1];
    const bool ground0 = model->geom_type[geom0] == mjGEOM_PLANE;
    const bool ground1 = model->geom_type[geom1] == mjGEOM_PLANE;
    if (ground0 == ground1) continue;
    const int wheel_geom = ground0 ? geom1 : geom0;
    const int ground_geom = ground0 ? geom0 : geom1;
    const int wheel_body = model->geom_bodyid[wheel_geom];
    const int ground_body = model->geom_bodyid[ground_geom];
    if (!body_is_in_subtree(model, wheel_body, wheel_root)) continue;
    result.has_ground_contact = true;

    mjtNum wheel_velocity[3]{}, ground_velocity[3]{};
    auto point_velocity = [&](int body, mjtNum velocity[3]) {
      if (body == 0) return;
      mj_jac(model, data, jacp.data(), jacr.data(), contact.pos, body);
      for (int axis = 0; axis < 3; ++axis)
        for (int dof = 0; dof < model->nv; ++dof)
          velocity[axis] += jacp[axis * model->nv + dof] * data->qvel[dof];
    };
    point_velocity(wheel_body, wheel_velocity);
    point_velocity(ground_body, ground_velocity);
    mjtNum normal_velocity = 0.0;
    mjtNum relative[3]{};
    for (int axis = 0; axis < 3; ++axis) {
      relative[axis] = wheel_velocity[axis] - ground_velocity[axis];
      normal_velocity += relative[axis] * contact.frame[axis];
    }
    double tangent_speed_squared = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
      const double tangent = relative[axis] - normal_velocity * contact.frame[axis];
      tangent_speed_squared += tangent * tangent;
    }
    result.speed = std::max(result.speed, std::sqrt(tangent_speed_squared));
  }
  result.valid = true;
  return result;
}

SDL_Gamepad* first_gamepad() {
  int count = 0;
  SDL_JoystickID* ids = SDL_GetGamepads(&count);
  SDL_Gamepad* pad = count > 0 ? SDL_OpenGamepad(ids[0]) : nullptr;
  SDL_free(ids);
  return pad;
}

struct JoystickDevice {
  SDL_JoystickID id = 0;
  std::string name;
  std::string guid;
};

std::string joystick_guid(SDL_JoystickID id) {
  char text[64]{};
  SDL_GUIDToString(SDL_GetJoystickGUIDForID(id), text, sizeof(text));
  return text;
}

std::vector<JoystickDevice> joystick_devices() {
  std::vector<JoystickDevice> devices;
  int count = 0;
  SDL_JoystickID* ids = SDL_GetJoysticks(&count);
  for (int i = 0; ids && i < count; ++i) {
    const char* name = SDL_GetJoystickNameForID(ids[i]);
    devices.push_back({ids[i], name ? name : "USB joystick", joystick_guid(ids[i])});
  }
  SDL_free(ids);
  return devices;
}

void refresh_joystick(SDL_Joystick*& joystick, SDL_JoystickID& instance_id,
                      InputMixer& mixer) {
  const auto devices = joystick_devices();
  const JoystickDevice* selected = nullptr;
  if (mixer.preferred_guid != "none") {
    if (!mixer.preferred_guid.empty()) {
      for (const auto& device : devices)
        if (device.guid == mixer.preferred_guid) { selected = &device; break; }
    } else if (!devices.empty()) {
      selected = &devices.front();
    }
  }
  if (selected && joystick && instance_id == selected->id) return;
  if (joystick) SDL_CloseJoystick(joystick);
  joystick = nullptr;
  instance_id = 0;
  if (selected) {
    joystick = SDL_OpenJoystick(selected->id);
    if (joystick) {
      instance_id = SDL_GetJoystickID(joystick);
      if (mixer.preferred_guid.empty())
        mixer.preferred_guid = selected->guid;
    }
  }
}

float normalize_axis(Sint16 raw, float deadzone) {
  float value = raw < 0 ? float(raw) / 32768.0f : float(raw) / 32767.0f;
  const float magnitude = std::abs(value);
  deadzone = std::clamp(deadzone, 0.0f, 0.95f);
  if (magnitude <= deadzone) return 0.0f;
  return std::copysign((magnitude - deadzone) / (1.0f - deadzone), value);
}

std::array<float, max_input_channels> read_input_channels(
    SDL_Joystick* joystick, const InputMixer& mixer) {
  std::array<float, max_input_channels> values{};
  if (!joystick) return values;
  for (int i = 0; i < mixer.channel_count; ++i) {
    const auto& channel = mixer.channels[i];
    float value = 0.0f;
    if (channel.button) {
      if (channel.source_index < SDL_GetNumJoystickButtons(joystick))
        value = SDL_GetJoystickButton(joystick, channel.source_index) ? 1.0f : 0.0f;
    } else if (channel.source_index < SDL_GetNumJoystickAxes(joystick)) {
      value = normalize_axis(SDL_GetJoystickAxis(joystick, channel.source_index), channel.deadzone);
      if (channel.inverted) value = -value;
    }
    values[i] = value;
  }
  return values;
}

struct MixerOutputDescriptor {
  std::string key;
  std::string label;
  bool left = false;
  bool right = false;
  bool weapon = false;
  int wheel_count = 0;
};

std::vector<MixerOutputDescriptor> mixer_outputs(const Controls& controls) {
  std::map<std::string, MixerOutputDescriptor> drives;
  auto add_wheels = [&](const std::vector<std::pair<int, double>>& wheels, bool left) {
    size_t ungrouped_index = 0;
    for (const auto& [id, direction] : wheels) {
      (void)direction;
      const auto motor = controls.motor_models.find(id);
      std::string key;
      std::string label;
      if (motor != controls.motor_models.end()) {
        const int group = std::clamp(motor->second.drive_group, 0, 7);
        key = motor_output_key(group);
        label = "Motor " + std::string(1, char('A' + group));
      } else {
        key = ungrouped_wheel_output_key(left, ungrouped_index);
        label = std::string(left ? "Left motor " : "Right motor ") +
                std::to_string(ungrouped_index + 1);
        ++ungrouped_index;
      }
      auto& output = drives[key];
      output.key = key;
      output.label = label;
      output.left = output.left || left;
      output.right = output.right || !left;
      ++output.wheel_count;
    }
  };
  add_wheels(controls.left, true);
  add_wheels(controls.right, false);

  std::vector<MixerOutputDescriptor> result;
  for (auto& [key, output] : drives) {
    (void)key;
    if (output.label.rfind("Motor ", 0) == 0 && output.wheel_count > 1)
      output.label += " (" + std::to_string(output.wheel_count) + " wheels)";
    result.push_back(output);
  }
  result.push_back({weapon_output_key(), "Weapon", false, false, true});
  return result;
}

std::string wheel_output_key(const Controls& controls, int actuator_id,
                             bool left, size_t ungrouped_index) {
  const auto motor = controls.motor_models.find(actuator_id);
  if (motor != controls.motor_models.end())
    return motor_output_key(std::clamp(motor->second.drive_group, 0, 7));
  return ungrouped_wheel_output_key(left, ungrouped_index);
}

std::array<float, max_input_channels> default_output_weights(
    const MixerOutputDescriptor& output) {
  std::array<float, max_input_channels> weights{};
  if (output.weapon) {
    weights[2] = 1.0f;
  } else {
    weights[0] = 1.0f;
    if (output.left && !output.right) weights[1] = -1.0f;
    else if (output.right && !output.left) weights[1] = 1.0f;
  }
  return weights;
}

float output_value(const InputMixer& mixer, const MixerOutputDescriptor& output,
                   const std::array<float, max_input_channels>& channels) {
  const auto found = mixer.outputs.find(output.key);
  const auto weights = found == mixer.outputs.end()
    ? default_output_weights(output) : found->second;
  const auto offsets = mixer.output_offsets.find(output.key);
  const auto expos = mixer.output_expos.find(output.key);
  return mix_output(weights, channels, mixer.channel_count, output.weapon,
    offsets == mixer.output_offsets.end() ? nullptr : &offsets->second,
    expos == mixer.output_expos.end() ? nullptr : &expos->second);
}

float output_channel_response(const InputMixer& mixer, const MixerOutputDescriptor& output,
                              int channel_index, float channel_value) {
  std::array<float, max_input_channels> sample{};
  if (channel_index >= 0 && channel_index < max_input_channels)
    sample[channel_index] = channel_value;
  return output_value(mixer, output, sample);
}

void draw_mixer_dialog(InputMixer& mixer, SDL_Joystick*& joystick,
                       SDL_JoystickID& joystick_id, const Controls& controls,
                       const std::string& model_key,
                       const std::string& model_name) {
  bool changed = false;
  const auto devices = joystick_devices();
  const auto outputs = mixer_outputs(controls);
  if (!joystick) mixer.capture_channel = -1;
  const char* joystick_name = joystick ? SDL_GetJoystickName(joystick) : nullptr;
  const std::string selected_name = joystick_name ? joystick_name :
    mixer.preferred_guid == "none" ? "No controller selected" :
    devices.empty() ? "No controller detected" : "Choose a controller";

  const float dialog_width = ImGui::GetContentRegionAvail().x;
  const float input_panel_width = std::min(500.0f, std::max(340.0f, dialog_width * 0.46f));
  constexpr float footer_reserve = 72.0f;
  ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.075f, 0.081f, 0.092f, 1.0f));
  ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.22f, 0.23f, 0.25f, 1.0f));
  ImGui::BeginChild("Controller input setup", ImVec2(input_panel_width, -footer_reserve),
                    ImGuiChildFlags_Borders | ImGuiChildFlags_AlwaysUseWindowPadding,
                    ImGuiWindowFlags_AlwaysVerticalScrollbar);
  ImGui::TextColored(ui_accent, "1  CONNECT YOUR CONTROLLER");
  ImGui::TextDisabled("This control setup is saved for %s.", model_name.c_str());
  ImGui::SetNextItemWidth(-1.0f);
  if (ImGui::BeginCombo("Controller", selected_name.c_str())) {
    if (ImGui::Selectable("No controller", mixer.preferred_guid == "none")) {
      mixer.preferred_guid = "none";
      changed = true;
      mixer.capture_channel = -1;
      refresh_joystick(joystick, joystick_id, mixer);
    }
    for (size_t device_index = 0; device_index < devices.size(); ++device_index) {
      const auto& device = devices[device_index];
      const bool selected = mixer.preferred_guid == device.guid ||
        (mixer.preferred_guid.empty() && joystick_id == device.id);
      const std::string label = device.name + "##controller_" + std::to_string(device_index);
      if (ImGui::Selectable(label.c_str(), selected)) {
        mixer.preferred_guid = device.guid;
        changed = true;
        mixer.capture_channel = -1;
        refresh_joystick(joystick, joystick_id, mixer);
      }
    }
    ImGui::EndCombo();
  }
  if (joystick) {
    ImGui::TextColored(ui_green, "CONNECTED");
    ImGui::SameLine();
    ImGui::TextDisabled("%d axes  |  %d buttons", SDL_GetNumJoystickAxes(joystick),
                        SDL_GetNumJoystickButtons(joystick));
  } else {
    ImGui::TextDisabled(devices.empty()
      ? "Plug in a USB controller, then choose it here."
      : "Choose a connected controller from the list.");
  }
  if (ImGui::Checkbox("Use controller mapping", &mixer.enabled)) changed = true;
  ImGui::TextDisabled("This can drive motors and weapon; keyboard stays available when disabled.");
  if (mixer.enabled && !joystick)
    ImGui::TextColored(ui_amber, "Controller mapping is enabled, but no controller is connected.");

  ImGui::SeparatorText("2  SET UP STICKS AND BUTTONS");
  ImGui::TextWrapped("Each input is one stick axis or button. Use Detect, then move the control you want.");
  if (ImGui::SmallButton("- input") && mixer.channel_count > 1) {
    --mixer.channel_count; changed = true;
    if (mixer.capture_channel >= mixer.channel_count) mixer.capture_channel = -1;
  }
  ImGui::SameLine();
  if (ImGui::SmallButton("+ input") && mixer.channel_count < max_input_channels) {
    const int new_index = mixer.channel_count++;
    mixer.channels[new_index] = {false, new_index, false, 0.12f};
    changed = true;
  }
  ImGui::SameLine();
  ImGui::TextDisabled("%d inputs", mixer.channel_count);

  const auto live_channels = read_input_channels(joystick, mixer);
  const float input_table_height = std::min(300.0f, 36.0f + 42.0f * mixer.channel_count);
  if (ImGui::BeginTable("##joystick_input_channels", 7,
      ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg |
      ImGuiTableFlags_SizingFixedFit,
      ImVec2(0.0f, input_table_height))) {
    ImGui::TableSetupColumn("Input", ImGuiTableColumnFlags_WidthFixed, 57.0f);
    ImGui::TableSetupColumn("Type", ImGuiTableColumnFlags_WidthFixed, 63.0f);
    ImGui::TableSetupColumn("Control #", ImGuiTableColumnFlags_WidthFixed, 98.0f);
    ImGui::TableSetupColumn("Reverse", ImGuiTableColumnFlags_WidthFixed, 58.0f);
    ImGui::TableSetupColumn("Deadzone", ImGuiTableColumnFlags_WidthFixed, 65.0f);
    ImGui::TableSetupColumn("Live", ImGuiTableColumnFlags_WidthFixed, 54.0f);
    ImGui::TableSetupColumn("Detect", ImGuiTableColumnFlags_WidthFixed, 58.0f);
    ImGui::TableHeadersRow();
    for (int i = 0; i < mixer.channel_count; ++i) {
      auto& channel = mixer.channels[i];
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0); ImGui::Text("Input %d", i + 1);
      ImGui::TableSetColumnIndex(1);
      ImGui::PushID(3000 + i);
      ImGui::SetNextItemWidth(58.0f);
      if (ImGui::BeginCombo("##source_type", channel.button ? "Button" : "Axis")) {
        if (ImGui::Selectable("Axis", !channel.button)) { channel.button = false; changed = true; }
        if (ImGui::Selectable("Button", channel.button)) { channel.button = true; changed = true; }
        ImGui::EndCombo();
      }
      ImGui::TableSetColumnIndex(2);
      const int control_count = joystick
        ? (channel.button ? SDL_GetNumJoystickButtons(joystick) : SDL_GetNumJoystickAxes(joystick))
        : (channel.button ? 16 : 8);
      channel.source_index = std::clamp(channel.source_index, 0,
        std::max(0, std::min(channel.button ? 127 : 63, control_count - 1)));
      char selected_control[48]{};
      std::snprintf(selected_control, sizeof(selected_control), "%s %d",
        channel.button ? "Button" : "Axis", channel.source_index + 1);
      ImGui::SetNextItemWidth(92.0f);
      if (ImGui::BeginCombo("##source_index", selected_control)) {
        const int visible_count = std::max(control_count, channel.source_index + 1);
        for (int control = 0; control < visible_count; ++control) {
          char control_label[48]{};
          std::snprintf(control_label, sizeof(control_label), "%s %d",
            channel.button ? "Button" : "Axis", control + 1);
          if (ImGui::Selectable(control_label, channel.source_index == control)) {
            channel.source_index = control;
            changed = true;
          }
        }
        ImGui::EndCombo();
      }
      ImGui::TableSetColumnIndex(3);
      if (channel.button) ImGui::TextDisabled("—");
      else if (ImGui::Checkbox("##inverted", &channel.inverted)) changed = true;
      ImGui::TableSetColumnIndex(4);
      if (channel.button) ImGui::TextDisabled("—");
      else {
        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::SliderFloat("##deadzone", &channel.deadzone, 0.0f, 0.50f, "%.2f")) changed = true;
      }
      ImGui::TableSetColumnIndex(5);
      const float live_value = live_channels[i];
      ImGui::ProgressBar(channel.button ? live_value : (live_value + 1.0f) * 0.5f,
                         ImVec2(-1.0f, 0.0f), "");
      ImGui::TableSetColumnIndex(6);
      const bool is_listening = mixer.capture_channel == i;
      ImGui::BeginDisabled(!joystick);
      if (ImGui::SmallButton(is_listening ? "Cancel" : "Detect"))
        mixer.capture_channel = is_listening ? -1 : i;
      ImGui::EndDisabled();
      if (!joystick && ImGui::IsItemHovered())
        ImGui::SetTooltip("Connect a controller first.");
      ImGui::PopID();
    }
    ImGui::EndTable();
  }
  if (mixer.capture_channel >= 0 && mixer.capture_channel < mixer.channel_count && joystick) {
    SDL_UpdateJoysticks();
    auto& channel = mixer.channels[mixer.capture_channel];
    bool detected = false;
    const int axis_count = SDL_GetNumJoystickAxes(joystick);
    for (int axis = 0; axis < axis_count; ++axis) {
      const float value = float(SDL_GetJoystickAxis(joystick, axis)) / 32768.0f;
      if (std::abs(value) >= 0.62f) {
        channel.button = false;
        channel.source_index = axis;
        detected = true;
        break;
      }
    }
    if (!detected) {
      const int button_count = SDL_GetNumJoystickButtons(joystick);
      for (int button = 0; button < button_count; ++button) {
        if (SDL_GetJoystickButton(joystick, button)) {
          channel.button = true;
          channel.source_index = button;
          detected = true;
          break;
        }
      }
    }
    if (detected) {
      changed = true;
      mixer.capture_channel = -1;
    } else {
      ImGui::TextColored(ui_amber, "Move one stick fully or press one button to detect it.");
    }
  }
  ImGui::TextDisabled("Control numbers start at 1. Reverse an axis if its direction feels wrong.");
  ImGui::TextDisabled("Default mix: Input 1 throttle, Input 2 steering, Input 3 weapon.");
  ImGui::EndChild();
  ImGui::PopStyleColor(2);

  ImGui::SameLine(0.0f, 12.0f);
  ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.075f, 0.081f, 0.092f, 1.0f));
  ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.22f, 0.23f, 0.25f, 1.0f));
  ImGui::BeginChild("Controller output mixer", ImVec2(0.0f, -footer_reserve),
                    ImGuiChildFlags_Borders | ImGuiChildFlags_AlwaysUseWindowPadding,
                    ImGuiWindowFlags_AlwaysVerticalScrollbar);
  ImGui::TextColored(ui_accent, "3  MAP INPUTS TO BOT CONTROLS");
  ImGui::TextDisabled("Choose a motor group or weapon, then tune how each input affects it.");
  static std::string selected_output_key;
  static int graph_channel = 0;
  if (!outputs.empty() && std::none_of(outputs.begin(), outputs.end(), [&](const auto& output) {
        return output.key == selected_output_key;
      })) selected_output_key = outputs.front().key;
  auto selected_output = std::find_if(outputs.begin(), outputs.end(), [&](const auto& output) {
    return output.key == selected_output_key;
  });
  const char* selected_output_name = selected_output == outputs.end()
    ? "No outputs available" : selected_output->label.c_str();
  if (ImGui::BeginCombo("Robot control", selected_output_name)) {
    for (const auto& output : outputs) {
      if (!ImGui::Selectable(output.label.c_str(), output.key == selected_output_key)) continue;
      selected_output_key = output.key;
      selected_output = std::find_if(outputs.begin(), outputs.end(), [&](const auto& item) {
        return item.key == selected_output_key;
      });
    }
    ImGui::EndCombo();
  }

  if (selected_output != outputs.end()) {
    const auto& output = *selected_output;
    auto [weights_it, weights_inserted] = mixer.outputs.try_emplace(
      output.key, default_output_weights(output));
    auto& weights = weights_it->second;
    auto& offsets = mixer.output_offsets[output.key];
    auto& expos = mixer.output_expos[output.key];
    if (weights_inserted) changed = true;
    if (ImGui::SmallButton("Reset this control")) {
      weights = default_output_weights(output);
      offsets.fill(0.0f);
      expos.fill(0.0f);
      changed = true;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("Weight = strength/direction. Offset = trim. Expo = stick feel.");
    const float output_table_height = std::min(260.0f, 36.0f + 28.0f * mixer.channel_count);
    if (ImGui::BeginTable("##output_mix_lines", 5,
        ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit,
        ImVec2(0.0f, output_table_height))) {
      ImGui::TableSetupColumn("Input", ImGuiTableColumnFlags_WidthFixed, 58.0f);
      ImGui::TableSetupColumn("Live", ImGuiTableColumnFlags_WidthFixed, 70.0f);
      ImGui::TableSetupColumn("Weight", ImGuiTableColumnFlags_WidthFixed, 108.0f);
      ImGui::TableSetupColumn("Offset", ImGuiTableColumnFlags_WidthFixed, 108.0f);
      ImGui::TableSetupColumn("Expo", ImGuiTableColumnFlags_WidthFixed, 108.0f);
      ImGui::TableHeadersRow();
      for (int i = 0; i < mixer.channel_count; ++i) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0); ImGui::Text("Input %d", i + 1);
        ImGui::TableSetColumnIndex(1); ImGui::Text("%+.0f%%", live_channels[i] * 100.0f);
        ImGui::PushID(output.key.c_str()); ImGui::PushID(i);
        ImGui::TableSetColumnIndex(2);
        float weight_percent = weights[i] * 100.0f;
        ImGui::SetNextItemWidth(100.0f);
        if (ImGui::DragFloat("##weight", &weight_percent, 1.0f, -100.0f, 100.0f, "%.0f%%")) {
          weights[i] = weight_percent / 100.0f; changed = true;
        }
        ImGui::TableSetColumnIndex(3);
        float offset_percent = offsets[i] * 100.0f;
        ImGui::SetNextItemWidth(100.0f);
        if (ImGui::DragFloat("##offset", &offset_percent, 1.0f, -100.0f, 100.0f, "%.0f%%")) {
          offsets[i] = offset_percent / 100.0f; changed = true;
        }
        ImGui::TableSetColumnIndex(4);
        float expo_percent = expos[i] * 100.0f;
        ImGui::SetNextItemWidth(100.0f);
        if (ImGui::DragFloat("##expo", &expo_percent, 1.0f, -100.0f, 100.0f, "%.0f%%")) {
          expos[i] = expo_percent / 100.0f; changed = true;
        }
        ImGui::PopID(); ImGui::PopID();
      }
      ImGui::EndTable();
    }

    graph_channel = std::clamp(graph_channel, 0, std::max(0, mixer.channel_count - 1));
    const std::string graph_channel_name = "Input " + std::to_string(graph_channel + 1);
    if (ImGui::BeginCombo("Graph input", graph_channel_name.c_str())) {
      for (int i = 0; i < mixer.channel_count; ++i) {
        const std::string label = "Input " + std::to_string(i + 1);
        if (ImGui::Selectable(label.c_str(), graph_channel == i)) graph_channel = i;
      }
      ImGui::EndCombo();
    }
    const float live_output = output_value(mixer, output, live_channels);
    ImGui::SameLine();
    ImGui::TextColored(ui_violet,
                       "Mixed output %+.0f%%", live_output * 100.0f);
    ImGui::TextDisabled("The curve shows this input. The amber dot follows your controller live.");
    const ImVec2 graph_size(std::max(260.0f, ImGui::GetContentRegionAvail().x), 200.0f);
    const ImVec2 graph_min = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("##mixer_response_graph", graph_size);
    const ImVec2 graph_max(graph_min.x + graph_size.x, graph_min.y + graph_size.y);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const ImU32 graph_bg = ImGui::GetColorU32(ImGuiCol_FrameBg);
    const ImU32 grid_color = ImGui::GetColorU32(ImGuiCol_Border);
    const ImU32 axis_color = ImGui::GetColorU32(ImGuiCol_TextDisabled);
    const ImU32 curve_color = ImGui::GetColorU32(ui_teal);
    const ImVec2 plot_min(graph_min.x + 36.0f, graph_min.y + 12.0f);
    const ImVec2 plot_max(graph_max.x - 12.0f, graph_max.y - 26.0f);
    const float input_min = mixer.channels[graph_channel].button ? 0.0f : -1.0f;
    const float input_max = 1.0f;
    const float output_min = output.weapon ? 0.0f : -1.0f;
    const float output_max = 1.0f;
    auto map_point = [&](float x, float y) {
      return ImVec2(plot_min.x + (x - input_min) / (input_max - input_min) * (plot_max.x - plot_min.x),
        plot_min.y + (output_max - y) / (output_max - output_min) * (plot_max.y - plot_min.y));
    };
    draw->AddRectFilled(graph_min, graph_max, graph_bg, 4.0f);
    draw->AddRect(plot_min, plot_max, grid_color, 0.0f, 0, 1.0f);
    for (int i = 1; i < 4; ++i) {
      const float y = plot_min.y + (plot_max.y - plot_min.y) * i / 4.0f;
      draw->AddLine(ImVec2(plot_min.x, y), ImVec2(plot_max.x, y), grid_color, 1.0f);
    }
    for (int i = 1; i < 4; ++i) {
      const float x = plot_min.x + (plot_max.x - plot_min.x) * i / 4.0f;
      draw->AddLine(ImVec2(x, plot_min.y), ImVec2(x, plot_max.y), grid_color, 1.0f);
    }
    if (!output.weapon && input_min < 0.0f) {
      const ImVec2 zero = map_point(0.0f, 0.0f);
      draw->AddLine(ImVec2(plot_min.x, zero.y), ImVec2(plot_max.x, zero.y), axis_color, 1.2f);
    }
    draw->AddText(ImVec2(graph_min.x + 4.0f, graph_min.y + 4.0f), axis_color,
                  output.weapon ? "OUT 0..100%" : "OUT -100..100%");
    draw->AddText(ImVec2(plot_min.x, graph_max.y - 19.0f), axis_color,
                  input_min < 0.0f ? "-100% INPUT" : "0% INPUT");
    draw->AddText(ImVec2(plot_max.x - 67.0f, graph_max.y - 19.0f), axis_color,
                  "+100% INPUT");
    auto evaluate_graph = [&](float x) {
      return output_channel_response(mixer, output, graph_channel, x);
    };
    ImVec2 previous = map_point(input_min, evaluate_graph(input_min));
    constexpr int curve_samples = 64;
    for (int i = 1; i <= curve_samples; ++i) {
      const float x = input_min + (input_max - input_min) * i / curve_samples;
      const ImVec2 next = map_point(x, evaluate_graph(x));
      draw->AddLine(previous, next, curve_color, 2.5f);
      previous = next;
    }
    const ImVec2 dot = map_point(live_channels[graph_channel],
                                 evaluate_graph(live_channels[graph_channel]));
    draw->AddCircleFilled(dot, 7.0f, ImGui::GetColorU32(ImVec4(0.91f, 0.53f, 0.10f, 1.0f)));
    draw->AddCircle(dot, 9.0f, ImGui::GetColorU32(ui_ink), 16, 1.5f);
  }
  ImGui::TextWrapped("Drive outputs use -100 to 100%%; weapon uses 0 to 100%%. Positive expo softens the center, negative expo sharpens it.");
  ImGui::EndChild();
  ImGui::PopStyleColor(2);
  if (changed) {
    if (!save_mixer_settings(mixer, model_key))
      ImGui::TextColored(ui_red, "Could not save mixer settings.");
  }
}

struct KeyboardControls {
  float throttle = 0.0f;
  float turn = 0.0f;
  bool weapon = false;
};

constexpr double opponent_ai_start_delay_seconds = 2.0;

bool opponent_ai_delay_elapsed(double simulation_time) {
  return std::isfinite(simulation_time) &&
         simulation_time >= opponent_ai_start_delay_seconds;
}

KeyboardControls read_keyboard_controls(const bool* keys, bool opponent = false) {
  if (!keys) return {};
  if (opponent) {
    return {
      float(keys[SDL_SCANCODE_I]) - float(keys[SDL_SCANCODE_K]),
      float(keys[SDL_SCANCODE_J]) - float(keys[SDL_SCANCODE_L]),
      keys[SDL_SCANCODE_M]
    };
  }
  return {float(keys[SDL_SCANCODE_W]) - float(keys[SDL_SCANCODE_S]),
          float(keys[SDL_SCANCODE_A]) - float(keys[SDL_SCANCODE_D]),
          keys[SDL_SCANCODE_SPACE]};
}

KeyboardControls aim_and_ram_command(const Vec3& robot_position,
                                     const Vec3& target_position,
                                     const Vec3& forward_heading,
                                     bool weapon_available,
                                     float aggression_percent) {
  const double dx = target_position[0] - robot_position[0];
  const double dy = target_position[1] - robot_position[1];
  const double distance = std::hypot(dx, dy);
  const double heading_length = std::hypot(forward_heading[0], forward_heading[1]);
  if (!std::isfinite(distance) || distance <= 1e-5 ||
      !std::isfinite(heading_length) || heading_length <= 1e-6)
    return {};

  const double target_x = dx / distance;
  const double target_y = dy / distance;
  const double forward_x = forward_heading[0] / heading_length;
  const double forward_y = forward_heading[1] / heading_length;
  const double heading_error = std::atan2(
      forward_x * target_y - forward_y * target_x,
      forward_x * target_x + forward_y * target_y);

  KeyboardControls command;
  command.weapon = weapon_available;
  const float aggression = std::clamp(aggression_percent, 25.0f, 100.0f) / 100.0f;
  constexpr double ram_alignment_tolerance = 0.18;
  if (std::abs(heading_error) <= ram_alignment_tolerance) {
    command.throttle = aggression;
  } else {
    command.turn = float(std::clamp(heading_error * 1.7, -1.0, 1.0)) * aggression;
  }
  return command;
}

KeyboardControls opponent_ram_ai(const mjModel* model, const mjData* data,
                                 const Controls& opponent_controls,
                                 float aggression_percent) {
  if (!model || !data || opponent_controls.left.empty() ||
      opponent_controls.right.empty()) return {};
  const int robot_body = mj_name2id(model, mjOBJ_BODY, "Target_Robot");
  const int target_body = mj_name2id(model, mjOBJ_BODY, "Player_Robot");
  if (robot_body <= 0 || target_body <= 0) return {};

  auto average_wheel_center = [&](const auto& wheels, Vec3& center) {
    int count = 0;
    for (const auto& [actuator_id, direction] : wheels) {
      (void)direction;
      if (actuator_id < 0 || actuator_id >= model->nu) continue;
      const int joint_id = model->actuator_trnid[2 * actuator_id];
      if (joint_id < 0 || joint_id >= model->njnt) continue;
      const int body_id = model->jnt_bodyid[joint_id];
      if (body_id <= 0 || body_id >= model->nbody) continue;
      for (int axis = 0; axis < 3; ++axis)
        center[axis] += data->xpos[3 * body_id + axis];
      ++count;
    }
    if (count == 0) return false;
    for (double& value : center) value /= count;
    return true;
  };

  Vec3 left_center{}, right_center{};
  if (!average_wheel_center(opponent_controls.left, left_center) ||
      !average_wheel_center(opponent_controls.right, right_center)) return {};
  Vec3 lateral{right_center[0] - left_center[0],
               right_center[1] - left_center[1], 0.0};
  Vec3 heading = cross3({0.0, 0.0, 1.0}, lateral);
  const double heading_length = std::hypot(heading[0], heading[1]);
  if (!std::isfinite(heading_length) || heading_length <= 1e-6) return {};
  heading[0] /= heading_length;
  heading[1] /= heading_length;

  const Vec3 robot_position{data->subtree_com[3 * robot_body],
                            data->subtree_com[3 * robot_body + 1],
                            data->subtree_com[3 * robot_body + 2]};
  const Vec3 target_position{data->subtree_com[3 * target_body],
                             data->subtree_com[3 * target_body + 1],
                             data->subtree_com[3 * target_body + 2]};
  return aim_and_ram_command(robot_position, target_position, heading,
                             !opponent_controls.weapon.empty(), aggression_percent);
}

void drive(const mjModel* model, mjData* data, Controls& c, SDL_Gamepad* pad,
           SDL_Joystick* joystick, const InputMixer& mixer,
           const KeyboardControls& keyboard, bool external_inputs_enabled) {
  float throttle = keyboard.throttle;
  float turn = keyboard.turn;
  const bool custom_input = external_inputs_enabled && mixer.enabled && joystick;
  std::array<float, max_input_channels> channels{};
  float weapon_throttle = 0.0f;
  if (custom_input) {
    channels = read_input_channels(joystick, mixer);
  } else {
    const float pad_throttle = external_inputs_enabled
      ? -axis_with_deadzone(pad, SDL_GAMEPAD_AXIS_LEFTY, c.deadzone) : 0.0f;
    const float pad_turn = external_inputs_enabled
      ? -axis_with_deadzone(pad, SDL_GAMEPAD_AXIS_LEFTX, c.deadzone) : 0.0f;
    if (pad_throttle != 0.0f) throttle = pad_throttle;
    if (pad_turn != 0.0f) turn = pad_turn;
    weapon_throttle = (keyboard.weapon ||
      (external_inputs_enabled && pad && (SDL_GetGamepadButton(pad, SDL_GAMEPAD_BUTTON_LEFT_SHOULDER) ||
               SDL_GetGamepadButton(pad, SDL_GAMEPAD_BUTTON_SOUTH)))) ? 1.0f : 0.0f;
  }

  const auto left_estimate = side_motor_speed_estimate(c.left, c.motor_models,
                                                       c.battery_voltage);
  const auto right_estimate = side_motor_speed_estimate(c.right, c.motor_models,
                                                        c.battery_voltage);
  const double left_rpm = c.manual_wheel_rpm_override ? c.wheel_rpm : left_estimate.rpm;
  const double right_rpm = c.manual_wheel_rpm_override ? c.wheel_rpm : right_estimate.rpm;
  if (custom_input) {
    const auto outputs = mixer_outputs(c);
    std::map<int, double> custom_speeds;
    auto set_custom_speeds = [&](const std::vector<std::pair<int, double>>& wheels,
                                 bool left_side, double max_rpm) {
      size_t ungrouped_index = 0;
      for (const auto& [id, direction] : wheels) {
        (void)direction;
        const auto motor = c.motor_models.find(id);
        const std::string key = wheel_output_key(c, id, left_side, ungrouped_index);
        if (motor == c.motor_models.end()) ++ungrouped_index;
        const auto output = std::find_if(outputs.begin(), outputs.end(), [&](const auto& item) {
          return item.key == key;
        });
        const float normalized = output == outputs.end() ? 0.0f : output_value(mixer, *output, channels);
        custom_speeds[id] = normalized * max_rpm * 2.0 * pi / 60.0;
      }
    };
    set_custom_speeds(c.left, true, left_rpm);
    set_custom_speeds(c.right, false, right_rpm);
    const auto weapon = std::find_if(outputs.begin(), outputs.end(), [](const auto& output) {
      return output.weapon;
    });
    if (weapon != outputs.end()) weapon_throttle = output_value(mixer, *weapon, channels);
    apply_wheel_drive_targets(model, data, c, 0.0, 0.0, &custom_speeds);
  } else {
    const auto [left, right] = mix_drive(throttle, turn,
      left_rpm * 2.0 * pi / 60.0, right_rpm * 2.0 * pi / 60.0);
    apply_wheel_drive_targets(model, data, c, left, right);
  }
  apply_weapon_drive_targets(model, data, c, weapon_throttle);
}
}

// Command-line checks are handled before the interactive app is initialized;
// they share the same model builders so diagnostics exercise production paths.
int main(int argc, char** argv) {
  FusionBundle bundle;
  FusionBundle opponent_bundle;
  LanSession lan;
  if (argc >= 3 && std::string(argv[1]) == "--test-memory-bundle") {
    if (!bundle.Load(argv[2])) {
      std::fprintf(stderr, "Source bundle load failed: %s\n", bundle.Error().c_str());
      return 1;
    }
    std::vector<unsigned char> package;
    if (!bundle.ExportSessionPackage(package)) {
      std::fprintf(stderr, "In-memory package export failed: %s\n", bundle.Error().c_str());
      return 1;
    }
    FusionBundle received;
    if (!received.LoadSessionPackage(package) || !received.IsMemoryLoaded()) {
      std::fprintf(stderr, "In-memory package load failed: %s\n", received.Error().c_str());
      return 1;
    }
    std::vector<unsigned char> roundtrip;
    if (!received.ExportSessionPackage(roundtrip) || roundtrip != package) {
      std::fprintf(stderr, "In-memory package round trip changed bundle data.\n");
      return 1;
    }
    if (received.Save()) {
      std::fprintf(stderr, "Session-loaded CAD unexpectedly allowed saving to disk.\n");
      return 1;
    }
    std::filesystem::path forbidden_preview;
    if (received.GeneratePreview(forbidden_preview)) {
      std::fprintf(stderr, "Session-loaded CAD unexpectedly generated a disk preview.\n");
      return 1;
    }
    std::vector<unsigned char> malformed = package;
    malformed[0] ^= 0xFF;
    FusionBundle rejected;
    if (rejected.LoadSessionPackage(malformed) || rejected.HasBundle()) {
      std::fprintf(stderr, "Malformed in-memory package was not rejected.\n");
      return 1;
    }
    malformed.assign(package.begin(), package.begin() + 15);
    if (rejected.LoadSessionPackage(malformed)) {
      std::fprintf(stderr, "Truncated in-memory package was not rejected.\n");
      return 1;
    }
    std::string xml;
    std::vector<MemoryBundleAsset> assets;
    if (!received.GenerateInMemory(xml, assets, &received)) {
      std::fprintf(stderr, "In-memory MJCF generation failed: %s\n", received.Error().c_str());
      return 1;
    }
    std::array<char, 8192> diagnostic{};
    mjModel* memory_model = LoadModelFromMemory(xml, assets, diagnostic.data(),
                                                 int(diagnostic.size()));
    if (!memory_model) {
      std::fprintf(stderr, "MuJoCo VFS rejected the in-memory model: %s\n", diagnostic.data());
      return 1;
    }
    mjData* memory_data = mj_makeData(memory_model);
    const int player = mj_name2id(memory_model, mjOBJ_BODY, "Player_Robot");
    const int target = mj_name2id(memory_model, mjOBJ_BODY, "Target_Robot");
    const bool valid = memory_data && player > 0 && target > 0 && assets.size() > 0 &&
                       memory_model->opt.timestep <= 0.0004 + 1e-12;
    const int body_count = memory_model->nbody;
    const int geom_count = memory_model->ngeom;
    if (memory_data) {
      for (int step = 0; step < 100; ++step) mj_step(memory_model, memory_data);
    }
    const bool finite = memory_data && std::isfinite(memory_data->time) &&
      std::all_of(memory_data->qpos, memory_data->qpos + memory_model->nq,
                  [](mjtNum value) { return std::isfinite(value); });
    if (memory_data) mj_deleteData(memory_data);
    mj_deleteModel(memory_model);
    if (!valid || !finite) {
      std::fprintf(stderr, "In-memory MuJoCo model failed body, solver, or stepping checks.\n");
      return 1;
    }
    std::printf("In-memory CAD passed: %zu-byte package round trip, %zu VFS meshes, "
                "%d bodies, %d geoms, 100 simulation steps; disk save/preview blocked.\n",
                package.size(), assets.size(), body_count, geom_count);
    return 0;
  }
  if (argc >= 4 && std::string(argv[1]) == "--test-lan-session") {
    if (!bundle.Load(argv[2])) {
      std::fprintf(stderr, "LAN test host bundle load failed: %s\n", bundle.Error().c_str());
      return 1;
    }
    if (!opponent_bundle.Load(argv[3])) {
      std::fprintf(stderr, "LAN test joining bundle load failed: %s\n", opponent_bundle.Error().c_str());
      return 1;
    }
    std::vector<unsigned char> host_package, joining_package;
    if (!bundle.ExportSessionPackage(host_package) ||
        !opponent_bundle.ExportSessionPackage(joining_package)) {
      std::fprintf(stderr, "LAN test package export failed: %s\n",
                   !bundle.Error().empty() ? bundle.Error().c_str() : opponent_bundle.Error().c_str());
      return 1;
    }
    constexpr char test_key[] = "c9f4a18d37e06b52d1a88f4c7203be16";
    LanSession host, client;
    std::string error;
    if (!host.StartHost(0, host_package, test_key, error)) {
      std::fprintf(stderr, "LAN self-test host start failed: %s\n", error.c_str());
      return 1;
    }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
    while (host.BoundPort() == 0 && host.GetState() != LanSession::State::Failed &&
           std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if (host.BoundPort() == 0 || host.GetState() != LanSession::State::Waiting) {
      std::fprintf(stderr, "LAN self-test host listener failed: %s\n", host.Error().c_str());
      host.Stop();
      return 1;
    }
    if (!client.StartClient("127.0.0.1", host.BoundPort(), joining_package, test_key, error)) {
      std::fprintf(stderr, "LAN self-test client start failed: %s\n", error.c_str());
      host.Stop();
      return 1;
    }
    auto both_in_state = [&](LanSession::State expected, int milliseconds) {
      const auto until = std::chrono::steady_clock::now() +
        std::chrono::milliseconds(milliseconds);
      while (std::chrono::steady_clock::now() < until) {
        if (host.GetState() == LanSession::State::Failed ||
            client.GetState() == LanSession::State::Failed) return false;
        if (host.GetState() == expected && client.GetState() == expected) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      return host.GetState() == expected && client.GetState() == expected;
    };
    if (!both_in_state(LanSession::State::PackageReady, 10000)) {
      std::fprintf(stderr, "LAN encrypted package handshake failed: host='%s', client='%s'\n",
                   host.Error().c_str(), client.Error().c_str());
      host.Stop(); client.Stop();
      return 1;
    }
    std::vector<unsigned char> from_host, from_client;
    if (!host.TakeRemotePackage(from_host) || !client.TakeRemotePackage(from_client) ||
        from_host != joining_package || from_client != host_package) {
      std::fprintf(stderr, "LAN peers received different in-memory bot bundles.\n");
      host.Stop(); client.Stop();
      return 1;
    }

    FusionBundle host_player, host_guest, client_player, client_guest;
    const bool memory_bundles_loaded =
      host_player.LoadSessionPackage(host_package) &&
      host_guest.LoadSessionPackage(from_host) &&
      client_player.LoadSessionPackage(from_client) &&
      client_guest.LoadSessionPackage(joining_package);
    std::string host_xml, client_xml;
    std::vector<MemoryBundleAsset> host_assets, client_assets;
    const bool models_generated = memory_bundles_loaded &&
      host_player.GenerateInMemory(host_xml, host_assets, &host_guest) &&
      client_player.GenerateInMemory(client_xml, client_assets, &client_guest);
    std::array<char, 8192> host_diagnostic{}, client_diagnostic{};
    mjModel* host_model = models_generated
      ? LoadModelFromMemory(host_xml, host_assets, host_diagnostic.data(),
                            int(host_diagnostic.size())) : nullptr;
    mjModel* client_model = models_generated
      ? LoadModelFromMemory(client_xml, client_assets, client_diagnostic.data(),
                            int(client_diagnostic.size())) : nullptr;
    bool same_model = host_model && client_model &&
      host_model->nq == client_model->nq && host_model->nv == client_model->nv &&
      host_model->nu == client_model->nu && host_model->nbody == client_model->nbody &&
      host_model->ngeom == client_model->ngeom;
    if (same_model) {
      for (int i = 0; i < host_model->nbody; ++i) {
        const char* host_name = mj_id2name(host_model, mjOBJ_BODY, i);
        const char* client_name = mj_id2name(client_model, mjOBJ_BODY, i);
        if ((!host_name) != (!client_name) ||
            (host_name && std::strcmp(host_name, client_name) != 0)) same_model = false;
      }
      for (int i = 0; i < host_model->nu; ++i) {
        const char* host_name = mj_id2name(host_model, mjOBJ_ACTUATOR, i);
        const char* client_name = mj_id2name(client_model, mjOBJ_ACTUATOR, i);
        if ((!host_name) != (!client_name) ||
            (host_name && std::strcmp(host_name, client_name) != 0)) same_model = false;
      }
    }
    if (host_model) mj_deleteModel(host_model);
    if (client_model) mj_deleteModel(client_model);
    for (auto& asset : host_assets) std::fill(asset.bytes.begin(), asset.bytes.end(), 0);
    for (auto& asset : client_assets) std::fill(asset.bytes.begin(), asset.bytes.end(), 0);
    if (!same_model) {
      std::fprintf(stderr, "Host and client did not build the same MuJoCo model layout: host='%s', client='%s'\n",
                   host_diagnostic.data(), client_diagnostic.data());
      host.Stop(); client.Stop();
      return 1;
    }
    host.MarkReady();
    client.MarkReady();
    if (!both_in_state(LanSession::State::Connected, 5000)) {
      std::fprintf(stderr, "LAN readiness handshake failed: host='%s', client='%s'\n",
                   host.Error().c_str(), client.Error().c_str());
      host.Stop(); client.Stop();
      return 1;
    }
    if (!client.SendInput({0.75f, -0.25f, 1.0f})) {
      std::fprintf(stderr, "LAN controller input send failed.\n");
      host.Stop(); client.Stop();
      return 1;
    }
    const auto input_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    LanSession::Input input{};
    do {
      input = host.LatestRemoteInput();
      if (std::abs(input.throttle - 0.75f) < 1e-6f &&
          std::abs(input.turn + 0.25f) < 1e-6f &&
          std::abs(input.weapon - 1.0f) < 1e-6f) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    } while (std::chrono::steady_clock::now() < input_deadline);
    if (std::abs(input.throttle - 0.75f) >= 1e-6f ||
        std::abs(input.turn + 0.25f) >= 1e-6f ||
        std::abs(input.weapon - 1.0f) >= 1e-6f) {
      std::fprintf(stderr, "LAN input replication timed out.\n");
      host.Stop(); client.Stop();
      return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    const LanSession::Input timed_out_input = host.LatestRemoteInput();
    if (timed_out_input.throttle != 0.0f || timed_out_input.turn != 0.0f ||
        timed_out_input.weapon != 0.0f) {
      std::fprintf(stderr, "LAN input timeout did not release stale opponent controls.\n");
      host.Stop(); client.Stop();
      return 1;
    }
    const double qpos[]{1.0, 2.0, 0.7071067811865476, 0.7071067811865476};
    const double qvel[]{-0.5, 3.25, 0.125};
    if (!host.SendSnapshot(12.5, qpos, 4, qvel, 3)) {
      std::fprintf(stderr, "LAN physics snapshot send failed.\n");
      host.Stop(); client.Stop();
      return 1;
    }
    LanSession::Snapshot snapshot;
    const auto snapshot_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!client.TakeLatestSnapshot(snapshot) &&
           std::chrono::steady_clock::now() < snapshot_deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    const bool snapshot_ok = snapshot.time == 12.5 && snapshot.qpos.size() == 4 &&
      snapshot.qvel.size() == 3 && snapshot.qpos[2] == qpos[2] &&
      snapshot.qvel[1] == qvel[1];
    host.Stop();
    client.Stop();
    if (!snapshot_ok) {
      std::fprintf(stderr, "LAN physics snapshot replication timed out or changed values.\n");
      return 1;
    }

    LanSession locked_host, wrong_key_client;
    if (!locked_host.StartHost(0, host_package, test_key, error)) {
      std::fprintf(stderr, "LAN authentication test host failed: %s\n", error.c_str());
      return 1;
    }
    const auto auth_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (locked_host.BoundPort() == 0 &&
           std::chrono::steady_clock::now() < auth_deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    if (!locked_host.BoundPort() ||
        !wrong_key_client.StartClient("127.0.0.1", locked_host.BoundPort(), joining_package,
                                      "00000000000000000000000000000000", error)) {
      std::fprintf(stderr, "LAN wrong-key test could not start: %s\n", error.c_str());
      locked_host.Stop(); wrong_key_client.Stop();
      return 1;
    }
    const auto auth_check_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (wrong_key_client.GetState() != LanSession::State::Failed &&
           std::chrono::steady_clock::now() < auth_check_deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    const bool rejected_wrong_key = wrong_key_client.GetState() == LanSession::State::Failed;
    locked_host.Stop();
    wrong_key_client.Stop();
    if (!rejected_wrong_key) {
      std::fprintf(stderr, "LAN accepted a peer using the wrong session key.\n");
      return 1;
    }
    std::printf("LAN self-test passed: AES-GCM peer authentication, two distinct RAM bundles, "
                "matching host/client MuJoCo layouts, readiness gate, opponent controls, "
                "authoritative state snapshots, and wrong-key rejection.\n");
    return 0;
  }
  if (argc >= 2 && std::string(argv[1]) == "--test-input-mixer") {
    std::array<float, max_input_channels> input{0.6f, 0.25f, 1.0f};
    std::array<float, max_input_channels> left_weights{};
    std::array<float, max_input_channels> right_weights{};
    std::array<float, max_input_channels> weapon_weights{};
    std::array<float, max_input_channels> line_offsets{};
    std::array<float, max_input_channels> line_expos{};
    left_weights[0] = 1.0f; left_weights[1] = -1.0f;
    right_weights[0] = 1.0f; right_weights[1] = 1.0f;
    weapon_weights[2] = 1.0f;
    line_offsets[0] = 0.1f;
    line_expos[0] = 0.5f;
    const float left = mix_output(left_weights, input, 3);
    const float right = mix_output(right_weights, input, 3);
    const float weapon = mix_output(weapon_weights, input, 3, true);
    const float shaped_line = mix_output(left_weights, input, 1, false,
                                         &line_offsets, &line_expos);
    std::array<float, max_input_channels> saturation_input{1.0f, 1.0f};
    std::array<float, max_input_channels> negative_weapon_input{0.0f, 0.0f, -1.0f};
    const float capped = mix_output(right_weights, saturation_input, 3);
    const float weapon_off = mix_output(weapon_weights, negative_weapon_input, 3, true);
    Controls sample_controls;
    sample_controls.left = {{10, 1.0}, {11, 1.0}};
    sample_controls.right = {{12, 1.0}, {13, 1.0}};
    MotorSpec motor_a; motor_a.drive_group = 0;
    MotorSpec motor_b; motor_b.drive_group = 1;
    sample_controls.motor_models[10] = motor_a;
    sample_controls.motor_models[11] = motor_a;
    sample_controls.motor_models[12] = motor_b;
    sample_controls.motor_models[13] = motor_b;
    InputMixer defaults;
    const auto descriptors = mixer_outputs(sample_controls);
    auto descriptor_for = [&](const std::string& key) -> const MixerOutputDescriptor* {
      const auto found = std::find_if(descriptors.begin(), descriptors.end(), [&](const auto& output) {
        return output.key == key;
      });
      return found == descriptors.end() ? nullptr : &*found;
    };
    const auto* motor_a_output = descriptor_for(motor_output_key(0));
    const auto* motor_b_output = descriptor_for(motor_output_key(1));
    const auto* weapon_output = descriptor_for(weapon_output_key());
    InputMixer first_bot_profile, second_bot_profile;
    initialize_default_mixer(first_bot_profile);
    initialize_default_mixer(second_bot_profile);
    first_bot_profile.enabled = true;
    first_bot_profile.channel_count = 5;
    first_bot_profile.channels[0].source_index = 6;
    first_bot_profile.outputs[motor_output_key(0)][0] = 0.65f;
    second_bot_profile.channel_count = 3;
    second_bot_profile.channels[0].source_index = 1;
    second_bot_profile.outputs[motor_output_key(0)][0] = 0.35f;
    json model_profiles = json::object();
    model_profiles["bot-one"] = mixer_profile_json(first_bot_profile);
    model_profiles["bot-two"] = mixer_profile_json(second_bot_profile);
    InputMixer restored_first, restored_second;
    initialize_default_mixer(restored_first);
    initialize_default_mixer(restored_second);
    load_mixer_profile(restored_first, model_profiles["bot-one"]);
    load_mixer_profile(restored_second, model_profiles["bot-two"]);
    const auto first_model_key = mixer_model_profile_key(
      FusionBundle{}, std::filesystem::path("models/bot-one.xml"));
    const auto second_model_key = mixer_model_profile_key(
      FusionBundle{}, std::filesystem::path("models/bot-two.xml"));
    const bool model_profiles_isolated = first_model_key != second_model_key &&
      restored_first.enabled && restored_first.channel_count == 5 &&
      restored_first.channels[0].source_index == 6 &&
      std::abs(restored_first.outputs[motor_output_key(0)][0] - 0.65f) < 1e-6f &&
      !restored_second.enabled && restored_second.channel_count == 3 &&
      restored_second.channels[0].source_index == 1 &&
      std::abs(restored_second.outputs[motor_output_key(0)][0] - 0.35f) < 1e-6f;
    if (std::abs(left - 0.35f) > 1e-5f || std::abs(right - 0.85f) > 1e-5f ||
        std::abs(weapon - 1.0f) > 1e-5f ||
        std::abs(shaped_line - 0.508f) > 1e-5f ||
        std::abs(capped - 1.0f) > 1e-5f || std::abs(weapon_off) > 1e-6f ||
        !motor_a_output || !motor_b_output || !weapon_output ||
        std::abs(output_value(defaults, *motor_a_output, input) - 0.35f) > 1e-5f ||
        std::abs(output_channel_response(defaults, *motor_a_output, 0, input[0]) - 0.6f) > 1e-5f ||
        std::abs(output_value(defaults, *motor_b_output, input) - 0.85f) > 1e-5f ||
        std::abs(output_value(defaults, *weapon_output, input) - 1.0f) > 1e-5f ||
        std::abs(normalize_axis(0, 0.12f)) > 1e-6f ||
        std::abs(normalize_axis(32767, 0.12f) - 1.0f) > 1e-6f ||
        std::abs(normalize_axis(-32768, 0.12f) + 1.0f) > 1e-6f ||
        !model_profiles_isolated) {
      std::fprintf(stderr, "Input mixer failed: motor/weapon mixes, axis calibration, or model-specific profile isolation are incorrect.\n");
      return 1;
    }
    std::puts("Input mixer passed: drive/weapon mixes, axis calibration, and independent saved profiles for different model keys.");
    return 0;
  }
  if (argc >= 4 && std::string(argv[1]) == "--test-opponent-bundle") {
    if (!bundle.Load(argv[2])) {
      std::fprintf(stderr, "Player bundle load failed: %s\n", bundle.Error().c_str());
      return 1;
    }
    if (!opponent_bundle.Load(argv[3])) {
      std::fprintf(stderr, "Opponent bundle load failed: %s\n", opponent_bundle.Error().c_str());
      return 1;
    }
    std::filesystem::path generated;
    if (!bundle.Generate(generated, &opponent_bundle)) {
      std::fprintf(stderr, "Opponent model generation failed: %s\n", bundle.Error().c_str());
      return 1;
    }
    std::array<char, 8192> diagnostic{};
    mjModel* test_model = mj_loadXML(generated.string().c_str(), nullptr,
                                      diagnostic.data(), diagnostic.size());
    if (!test_model) {
      std::fprintf(stderr, "Generated opponent model did not load: %s\n", diagnostic.data());
      return 1;
    }
    mjData* test_data = mj_makeData(test_model);
    const int target_body = mj_name2id(test_model, mjOBJ_BODY, "Target_Robot");
    const int target_joint = mj_name2id(test_model, mjOBJ_JOINT, "target_free");
    const int opponent_geom = mj_name2id(test_model, mjOBJ_GEOM, "opponent_part_0");
    double opponent_total_mass = target_body >= 0 ? test_model->body_mass[target_body] : 0.0;
    for (int group = 1; group <= 4; ++group) {
      const std::string body_name = "opponent_assembly_" + std::to_string(group);
      const int body_id = mj_name2id(test_model, mjOBJ_BODY, body_name.c_str());
      if (body_id >= 0) opponent_total_mass += test_model->body_mass[body_id];
    }
    if (!test_data || target_body < 0 || target_joint < 0 || opponent_geom < 0 ||
        std::abs(opponent_total_mass - 0.34) > 1e-6 ||
        test_model->geom_contype[opponent_geom] == 0 ||
        test_model->geom_conaffinity[opponent_geom] == 0) {
      std::fprintf(stderr, "Opponent assembly bodies, imported collision mesh, or combined mass is missing.\n");
      if (test_data) mj_deleteData(test_data);
      mj_deleteModel(test_model);
      return 1;
    }
    for (int group = 1; group <= 4; ++group) {
      const std::string joint_name = "opponent_joint_" + std::to_string(group);
      const int joint_id = mj_name2id(test_model, mjOBJ_JOINT, joint_name.c_str());
      if (joint_id < 0 || test_model->jnt_type[joint_id] != mjJNT_HINGE) {
        std::fprintf(stderr, "Opponent wheel/weapon/fork hinge %s is missing.\n", joint_name.c_str());
        mj_deleteData(test_data); mj_deleteModel(test_model);
        return 1;
      }
    }
    Controls opponent_test_controls;
    opponent_bundle.BindControls(test_model, opponent_test_controls.left,
        opponent_test_controls.right, opponent_test_controls.weapon,
        opponent_test_controls.motor_models, opponent_test_controls.weapon_motor_models,
        opponent_test_controls.manual_velocity_actuators,
        opponent_test_controls.battery_voltage, true);
    const int opponent_weapon_actuator = mj_name2id(
        test_model, mjOBJ_ACTUATOR, "opponent_assembly_1_drive");
    const int opponent_wheel_actuator = mj_name2id(
        test_model, mjOBJ_ACTUATOR, "opponent_assembly_2_drive");
    const int opponent_right_wheel_actuator = mj_name2id(
        test_model, mjOBJ_ACTUATOR, "opponent_assembly_3_drive");
    if (opponent_weapon_actuator < 0 || opponent_wheel_actuator < 0 ||
        opponent_right_wheel_actuator < 0 || opponent_test_controls.left.size() != 1 ||
        opponent_test_controls.right.size() != 1 ||
        opponent_test_controls.weapon.size() != 1 ||
        opponent_test_controls.motor_models.size() != 2 ||
        opponent_test_controls.weapon_motor_models.size() != 1 ||
        opponent_test_controls.manual_velocity_actuators.size() != 2) {
      std::fprintf(stderr, "Opponent wheel/weapon motor actuators did not bind independently.\n");
      mj_deleteData(test_data); mj_deleteModel(test_model);
      return 1;
    }
    const int opponent_fork_geom = mj_name2id(test_model, mjOBJ_GEOM, "opponent_part_3");
    bool opponent_fork_pair = false;
    for (int pair = 0; pair < test_model->npair; ++pair)
      if ((test_model->pair_geom1[pair] == opponent_fork_geom &&
           test_model->pair_geom2[pair] == opponent_geom) ||
          (test_model->pair_geom2[pair] == opponent_fork_geom &&
           test_model->pair_geom1[pair] == opponent_geom))
        opponent_fork_pair = true;
    if (!opponent_fork_pair) {
      std::fprintf(stderr, "Configured opponent fork does not have its frame collision pair.\n");
      mj_deleteData(test_data); mj_deleteModel(test_model);
      return 1;
    }
    mj_forward(test_model, test_data);
    mjvScene axis_scene;
    mjv_defaultScene(&axis_scene);
    mjv_makeScene(test_model, &axis_scene, 512);
    if (bundle.ShowAxisMarkers())
      bundle.AddAxisMarkers(test_model, test_data, axis_scene);
    const int host_axis_marker_count = axis_scene.ngeom;
    if (opponent_bundle.ShowAxisMarkers())
      opponent_bundle.AddAxisMarkers(test_model, test_data, axis_scene, true);
    const int guest_axis_marker_count = axis_scene.ngeom - host_axis_marker_count;
    mjv_freeScene(&axis_scene);
    if (bundle.ShowAxisMarkers() || !opponent_bundle.ShowAxisMarkers() ||
        host_axis_marker_count != 0 || guest_axis_marker_count != 8) {
      std::fprintf(stderr,
          "LAN client axis visibility did not show only the joining player's bot axes.\n");
      mj_deleteData(test_data); mj_deleteModel(test_model);
      return 1;
    }
    const int target_qpos = test_model->jnt_qposadr[target_joint];
    if (std::abs(test_data->qpos[target_qpos] - 0.55) > 1e-6 ||
        std::abs(test_data->qpos[target_qpos + 1]) > 1e-6 ||
        std::abs(test_data->qpos[target_qpos + 2] - 0.012) > 1e-6) {
      std::fprintf(stderr, "Opponent did not spawn opposite the player.\n");
      mj_deleteData(test_data); mj_deleteModel(test_model);
      return 1;
    }
    const KeyboardControls spawn_ai_command = opponent_ram_ai(
        test_model, test_data, opponent_test_controls, 70.0f);
    if (std::abs(spawn_ai_command.throttle - 0.70f) > 1e-6f ||
        spawn_ai_command.turn != 0.0f ||
        !spawn_ai_command.weapon) {
      std::fprintf(stderr,
          "Opponent ram AI did not apply 70%% aggression toward the player ahead from its configured spawn heading.\n");
      mj_deleteData(test_data); mj_deleteModel(test_model);
      return 1;
    }
    const int player_body = mj_name2id(test_model, mjOBJ_BODY, "Player_Robot");
    const int player_joint = mj_name2id(test_model, mjOBJ_JOINT, "robot_free");
    const int player_geom = mj_name2id(test_model, mjOBJ_GEOM, "cad_part_0");
    if (player_body < 0 || player_joint < 0 || player_geom < 0) {
      std::fprintf(stderr, "Player free body missing while loading opponent.\n");
      mj_deleteData(test_data); mj_deleteModel(test_model);
      return 1;
    }
    const int player_qpos = test_model->jnt_qposadr[player_joint];
    test_data->qpos[player_qpos + 1] += 0.5;
    mj_forward(test_model, test_data);
    const KeyboardControls side_aim_command = opponent_ram_ai(
        test_model, test_data, opponent_test_controls, 70.0f);
    if (side_aim_command.throttle != 0.0f || side_aim_command.turn >= 0.0f) {
      std::fprintf(stderr,
          "Opponent ram AI did not turn toward a player shifted to its left.\n");
      mj_deleteData(test_data); mj_deleteModel(test_model);
      return 1;
    }
    test_data->qpos[player_qpos + 1] -= 0.5;
    mj_forward(test_model, test_data);
    // Keep the earlier assertion that the target initially faces the player,
    // then align its orientation for a direct collision-overlap check.
    test_data->qpos[target_qpos + 6] = 1.0;
    test_data->qpos[target_qpos + 7] = 0.0;
    test_data->qpos[target_qpos + 8] = 0.0;
    test_data->qpos[target_qpos + 9] = 0.0;
    mj_forward(test_model, test_data);
    for (int axis = 0; axis < 3; ++axis)
      test_data->qpos[target_qpos + axis] +=
        test_data->geom_xpos[3 * player_geom + axis] -
        test_data->geom_xpos[3 * opponent_geom + axis];
    test_data->qpos[target_qpos] += 0.02;  // Partial overlap avoids coincident-mesh ambiguity.
    mj_forward(test_model, test_data);
    bool contact_with_player = false;
    for (int i = 0; i < test_data->ncon; ++i) {
      const int body0 = test_model->geom_bodyid[test_data->contact[i].geom[0]];
      const int body1 = test_model->geom_bodyid[test_data->contact[i].geom[1]];
      if ((body0 == target_body && body1 == player_body) ||
          (body1 == target_body && body0 == player_body)) contact_with_player = true;
    }
    mj_deleteData(test_data);
    mj_deleteModel(test_model);
    if (!contact_with_player) {
      std::fprintf(stderr, "Imported opponent did not produce player-to-opponent collision contacts.\n");
      return 1;
    }
    std::puts("Opponent bundle passed: two wheel drives, weapon, and fork load with 0.34 kg total mass; opposite spawn faces the player; guest-only axis visibility; fork/frame contact and player collision work.");
    return 0;
  }
  if (argc >= 3 && std::string(argv[1]) == "--test-transform-gizmo") {
    std::array<char, 8192> diagnostic{};
    mjModel* test_model = mj_loadXML(argv[2], nullptr, diagnostic.data(), diagnostic.size());
    if (!test_model) {
      std::fprintf(stderr, "Transform test model load failed: %s\n", diagnostic.data());
      return 1;
    }
    mjData* test_data = mj_makeData(test_model);
    const int body = mj_name2id(test_model, mjOBJ_BODY, "Player_Robot");
    const int joint = free_joint_for_body(test_model, body);
    if (!test_data || body < 0 || joint < 0) {
      std::fprintf(stderr, "Transform test needs a Player_Robot body with a free joint.\n");
      if (test_data) mj_deleteData(test_data);
      mj_deleteModel(test_model);
      return 1;
    }
    const int qpos = test_model->jnt_qposadr[joint];
    const Vec3 initial{test_data->qpos[qpos], test_data->qpos[qpos + 1],
                       test_data->qpos[qpos + 2]};
    for (int axis = 0; axis < 3; ++axis)
      if (!translate_free_body_axis(test_model, test_data, body, axis, 0.01*(axis + 1))) {
        mj_deleteData(test_data); mj_deleteModel(test_model);
        return 1;
      }
    for (int axis = 0; axis < 3; ++axis)
      if (std::abs(test_data->qpos[qpos + axis] - initial[axis] - 0.01*(axis + 1)) > 1e-9) {
        std::fprintf(stderr, "World-axis translation failed on axis %d.\n", axis);
        mj_deleteData(test_data); mj_deleteModel(test_model);
        return 1;
      }
    const Vec3 pivot{test_data->subtree_com[3*body], test_data->subtree_com[3*body + 1],
                     test_data->subtree_com[3*body + 2]};
    mjtNum offset[3];
    mju_sub3(offset, pivot.data(), test_data->xpos + 3*body);
    Vec3 local_pivot{};
    mju_mulMatTVec(local_pivot.data(), test_data->xmat + 9*body, offset, 3, 3);
    for (int axis = 0; axis < 3; ++axis) {
      if (!rotate_free_body_axis(test_model, test_data, body, axis, 0.2, pivot, local_pivot)) {
        mj_deleteData(test_data); mj_deleteModel(test_model);
        return 1;
      }
      for (int component = 0; component < 3; ++component)
        if (std::abs(test_data->subtree_com[3*body + component] - pivot[component]) > 1e-8) {
          std::fprintf(stderr, "Rotation about axis %d failed to preserve the robot pivot.\n", axis);
          mj_deleteData(test_data); mj_deleteModel(test_model);
          return 1;
        }
    }
    mjvScene test_scene;
    mjv_defaultScene(&test_scene);
    mjv_makeScene(test_model, &test_scene, 2000);
    mjvOption test_options;
    mjv_defaultOption(&test_options);
    mjvCamera test_camera;
    mjv_defaultCamera(&test_camera);
    test_camera.distance = 3.5;
    test_camera.elevation = -55;
    test_camera.azimuth = 90;
    for (int i = 0; i < 3; ++i) test_camera.lookat[i] = pivot[i];
    mjv_updateScene(test_model, test_data, &test_options, nullptr, &test_camera,
                    mjCAT_ALL, &test_scene);
    const int initial_geoms = test_scene.ngeom;
    add_transform_gizmo(test_model, test_data, body, false, -1, 0.35, test_scene);
    if (test_scene.ngeom < initial_geoms + 4) {
      std::fprintf(stderr, "Move gizmo arrows were not added to the viewport scene.\n");
      mjv_freeScene(&test_scene); mj_deleteData(test_data); mj_deleteModel(test_model);
      return 1;
    }
    const Vec3 center{test_data->subtree_com[3*body], test_data->subtree_com[3*body + 1],
                      test_data->subtree_com[3*body + 2]};
    ScreenPoint screen_start;
    if (!project_to_screen(test_model, test_scene, 1280, 800, center, screen_start)) {
      std::fprintf(stderr, "Could not project the transform gizmo into the Drive viewport.\n");
      mjv_freeScene(&test_scene); mj_deleteData(test_data); mj_deleteModel(test_model);
      return 1;
    }
    double longest_axis_pixels = 0.0;
    int projected_axis = -1;
    for (int axis = 0; axis < 3; ++axis) {
      const Vec3 endpoint{center[0] + 0.35*world_axes[axis][0],
                          center[1] + 0.35*world_axes[axis][1],
                          center[2] + 0.35*world_axes[axis][2]};
      ScreenPoint screen_end;
      if (!project_to_screen(test_model, test_scene, 1280, 800, endpoint, screen_end)) continue;
      const double dx = screen_end[0] - screen_start[0], dy = screen_end[1] - screen_start[1];
      const double pixels = std::sqrt(dx*dx + dy*dy);
      if (pixels > longest_axis_pixels) { longest_axis_pixels = pixels; projected_axis = axis; }
    }
    if (projected_axis < 0 || longest_axis_pixels < 25.0) {
      std::fprintf(stderr, "Transform axis handles are not visible at the default Drive camera.\n");
      mjv_freeScene(&test_scene); mj_deleteData(test_data); mj_deleteModel(test_model);
      return 1;
    }
    const Vec3 endpoint{center[0] + 0.35*world_axes[projected_axis][0],
                        center[1] + 0.35*world_axes[projected_axis][1],
                        center[2] + 0.35*world_axes[projected_axis][2]};
    ScreenPoint screen_end;
    project_to_screen(test_model, test_scene, 1280, 800, endpoint, screen_end);
    const ScreenPoint arrow_mouse{screen_start[0] + 0.8*(screen_end[0] - screen_start[0]),
                                  screen_start[1] + 0.8*(screen_end[1] - screen_start[1])};
    const GizmoHit arrow_hit = pick_translation_axis(test_model, test_scene, 1280, 800,
                                                      center, 0.35, arrow_mouse);
    if (arrow_hit.axis != projected_axis || arrow_hit.distance > 1.0) {
      std::fprintf(stderr, "Projected axis arrow did not pick at its visible screen position.\n");
      mjv_freeScene(&test_scene); mj_deleteData(test_data); mj_deleteModel(test_model);
      return 1;
    }
    const int move_scene_geoms = test_scene.ngeom;
    add_transform_gizmo(test_model, test_data, body, true, -1, 0.35, test_scene);
    if (test_scene.ngeom < move_scene_geoms + 3*48) {
      std::fprintf(stderr, "Rotation gizmo rings were not added to the viewport scene.\n");
      mjv_freeScene(&test_scene); mj_deleteData(test_data); mj_deleteModel(test_model);
      return 1;
    }
    for (int axis = 0; axis < 3; ++axis) {
      ScreenPoint ring_screen;
      if (!project_to_screen(test_model, test_scene, 1280, 800,
                             ring_point(center, axis, 0.35*0.82, 1.1), ring_screen)) {
        std::fprintf(stderr, "Could not project rotation ring %d into the Drive viewport.\n", axis);
        mjv_freeScene(&test_scene); mj_deleteData(test_data); mj_deleteModel(test_model);
        return 1;
      }
      const GizmoHit ring_hit = pick_rotation_ring(test_model, test_scene, 1280, 800,
          center, 0.35*0.82, ring_screen, axis);
      if (ring_hit.axis != axis || ring_hit.distance > 1.0) {
        std::fprintf(stderr, "Projected rotation ring %d did not pick at its screen position.\n", axis);
        mjv_freeScene(&test_scene); mj_deleteData(test_data); mj_deleteModel(test_model);
        return 1;
      }
    }
    mjv_freeScene(&test_scene);
    mj_deleteData(test_data);
    mj_deleteModel(test_model);
    std::puts("Transform gizmo passed: X/Y/Z arrows and rotation rings render and pick; translation and rotation preserve the robot pivot.");
    return 0;
  }
  if (argc >= 2 && std::string(argv[1]) == "--test-drive-mix") {
    constexpr double speed = 44.4;
    const auto left_turn = mix_drive(0.0f, 1.0f, speed);
    const auto right_turn = mix_drive(0.0f, -1.0f, speed);
    const auto forward = mix_drive(1.0f, 0.0f, speed);
    const auto reverse = mix_drive(-1.0f, 0.0f, speed);
    if (left_turn[0] != -speed || left_turn[1] != speed ||
        right_turn[0] != speed || right_turn[1] != -speed ||
        forward[0] != speed || forward[1] != speed ||
        reverse[0] != -speed || reverse[1] != -speed) {
      std::fprintf(stderr, "Drive mix failed: zero-throttle turns must counter-rotate both sides.\n");
      return 1;
    }
    std::array<bool, SDL_SCANCODE_COUNT> keys{};
    auto pressed = [&](SDL_Scancode key, bool opponent) {
      keys.fill(false);
      keys[key] = true;
      return read_keyboard_controls(keys.data(), opponent);
    };
    const auto opponent_forward = pressed(SDL_SCANCODE_I, true);
    const auto opponent_reverse = pressed(SDL_SCANCODE_K, true);
    const auto opponent_left = pressed(SDL_SCANCODE_J, true);
    const auto opponent_right = pressed(SDL_SCANCODE_L, true);
    const auto opponent_weapon = pressed(SDL_SCANCODE_M, true);
    const auto player_forward = pressed(SDL_SCANCODE_W, false);
    const auto player_reverse = pressed(SDL_SCANCODE_S, false);
    const auto player_left = pressed(SDL_SCANCODE_A, false);
    const auto player_right = pressed(SDL_SCANCODE_D, false);
    const auto player_weapon = pressed(SDL_SCANCODE_SPACE, false);
    const auto i_not_player = pressed(SDL_SCANCODE_I, false);
    const auto w_not_opponent = pressed(SDL_SCANCODE_W, true);
    const auto m_not_player_weapon = pressed(SDL_SCANCODE_M, false);
    const auto ai_forward = aim_and_ram_command(
        {0, 0, 0}, {2, 0, 0}, {1, 0, 0}, true, 70.0f);
    const auto ai_turn_left = aim_and_ram_command(
        {0, 0, 0}, {0, 2, 0}, {1, 0, 0}, false, 70.0f);
    const auto ai_turn_right = aim_and_ram_command(
        {0, 0, 0}, {0, -2, 0}, {1, 0, 0}, false, 70.0f);
    const auto ai_low_aggression_turn = aim_and_ram_command(
        {0, 0, 0}, {0, 2, 0}, {1, 0, 0}, false, 25.0f);
    const auto ai_full_aggression_turn = aim_and_ram_command(
        {0, 0, 0}, {0, 2, 0}, {1, 0, 0}, false, 100.0f);
    if (opponent_forward.throttle != 1.0f || opponent_reverse.throttle != -1.0f ||
        opponent_left.turn != 1.0f || opponent_right.turn != -1.0f || !opponent_weapon.weapon ||
        player_forward.throttle != 1.0f || player_reverse.throttle != -1.0f ||
        player_left.turn != 1.0f || player_right.turn != -1.0f || !player_weapon.weapon ||
        i_not_player.throttle != 0.0f || w_not_opponent.throttle != 0.0f ||
        m_not_player_weapon.weapon || std::abs(ai_forward.throttle - 0.70f) > 1e-6f ||
        ai_forward.turn != 0.0f || !ai_forward.weapon || ai_turn_left.throttle != 0.0f ||
        ai_turn_left.turn <= 0.0f || ai_turn_right.throttle != 0.0f ||
        ai_turn_right.turn >= 0.0f ||
        std::abs(ai_low_aggression_turn.turn - 0.25f) > 1e-6f ||
        std::abs(ai_full_aggression_turn.turn - 1.0f) > 1e-6f ||
        opponent_ai_delay_elapsed(1.999) || !opponent_ai_delay_elapsed(2.0) ||
        opponent_ai_delay_elapsed(std::numeric_limits<double>::quiet_NaN())) {
      std::fprintf(stderr, "Drive mapping or opponent aim-and-ram command failed.\n");
      return 1;
    }
    std::puts("Drive mixing passed: WASD/Space control the player; IJKL/M control the manual opponent; opponent AI aggression scales throttle and turn, and its simulation-time warm-up is checked.");
    return 0;
  }
  if (argc >= 2 && std::string(argv[1]) == "--test-straight-drive") {
    const bool arena_mode = argc >= 3 && std::string(argv[2]) == "--arena";
    const int model_arg = arena_mode ? 3 : 2;
    const std::filesystem::path test_model = argc > model_arg ? argv[model_arg] :
      stock_model_path();
    std::array<char, 4096> diagnostic{};
    mjModel* checked = mj_loadXML(test_model.string().c_str(), nullptr,
                                  diagnostic.data(), diagnostic.size());
    if (!checked) { std::fprintf(stderr, "%s\n", diagnostic.data()); return 1; }
    mjData* checked_data = mj_makeData(checked);
    const int player_body = mj_name2id(checked, mjOBJ_BODY, "Player_Robot");
    const int player_joint = mj_name2id(checked, mjOBJ_JOINT, "robot_free");
    Controls test_controls;
    bind_stock_actuators(checked, test_controls);
    test_controls.wheel_rpm = 424.0f;
    if (!checked_data || player_body < 0 || player_joint < 0 ||
        test_controls.left.size() != 2 || test_controls.right.size() != 2) {
      if (checked_data) mj_deleteData(checked_data);
      mj_deleteModel(checked);
      std::fprintf(stderr, "Straight-drive test requires the stock Player_Robot and four drive actuators.\n");
      return 1;
    }
    const int root_qpos = checked->jnt_qposadr[player_joint];
    if (!arena_mode) {
      checked_data->qpos[root_qpos] = 0.0;
      checked_data->qpos[root_qpos + 1] = 0.0;
      checked_data->qpos[root_qpos + 2] = 0.065;
      checked_data->qpos[root_qpos + 3] = 1.0;
      checked_data->qpos[root_qpos + 4] = 0.0;
      checked_data->qpos[root_qpos + 5] = 0.0;
      checked_data->qpos[root_qpos + 6] = 0.0;
      for (int geom = 0; geom < checked->ngeom; ++geom) {
        const int body = checked->geom_bodyid[geom];
        if (body == 0 || body_is_in_subtree(checked, body, player_body) ||
            checked->geom_type[geom] == mjGEOM_PLANE) continue;
        checked->geom_contype[geom] = 0;
        checked->geom_conaffinity[geom] = 0;
      }
    }
    mj_forward(checked, checked_data);
    for (int step = 0; step < 500; ++step) {
      apply_wheel_drive_targets(checked, checked_data, test_controls, 0.0, 0.0);
      mj_step(checked, checked_data);
    }
    const double start_x = checked_data->qpos[root_qpos];
    const double start_y = checked_data->qpos[root_qpos + 1];
    const double start_w = checked_data->qpos[root_qpos + 3];
    const double start_qx = checked_data->qpos[root_qpos + 4];
    const double start_qy = checked_data->qpos[root_qpos + 5];
    const double start_qz = checked_data->qpos[root_qpos + 6];
    const double start_yaw = std::atan2(2.0 * (start_w * start_qz + start_qx * start_qy),
      1.0 - 2.0 * (start_qy * start_qy + start_qz * start_qz));
    const auto [left_speed, right_speed] = mix_drive(1.0f, 0.0f,
      test_controls.wheel_rpm * 2.0 * pi / 60.0);
    for (int step = 0; step < 2500; ++step) {
      apply_wheel_drive_targets(checked, checked_data, test_controls, left_speed, right_speed);
      mj_step(checked, checked_data);
    }
    const double end_x = checked_data->qpos[root_qpos];
    const double end_y = checked_data->qpos[root_qpos + 1];
    const double w = checked_data->qpos[root_qpos + 3];
    const double qx = checked_data->qpos[root_qpos + 4];
    const double qy = checked_data->qpos[root_qpos + 5];
    const double qz = checked_data->qpos[root_qpos + 6];
    const double end_yaw = std::atan2(2.0 * (w * qz + qx * qy),
      1.0 - 2.0 * (qy * qy + qz * qz));
    const double yaw_change = std::remainder(end_yaw - start_yaw, 2.0 * pi);
    std::printf("Straight-drive test (%s, %s): commanded left=%.3f right=%.3f rad/s; ",
      arena_mode ? "arena" : "isolated", test_model.string().c_str(), left_speed, right_speed);
    std::printf("chassis dx=%.4f m dy=%.4f m yaw=%.5f rad (%.3f deg); wheel RPM L=[",
      end_x - start_x, end_y - start_y, yaw_change, yaw_change * 180.0 / pi);
    for (size_t i = 0; i < test_controls.left.size(); ++i)
      std::printf("%s%.1f", i ? "," : "", actuator_rpm(checked, checked_data,
        test_controls.left[i].first, test_controls.left[i].second));
    std::printf("] R=[");
    for (size_t i = 0; i < test_controls.right.size(); ++i)
      std::printf("%s%.1f", i ? "," : "", actuator_rpm(checked, checked_data,
        test_controls.right[i].first, test_controls.right[i].second));
    std::printf("]\n");
    const bool passed = std::isfinite(yaw_change) && std::isfinite(end_y - start_y) &&
      (arena_mode || (std::abs(yaw_change) < 0.03 && std::abs(end_y - start_y) < 0.03));
    mj_deleteData(checked_data); mj_deleteModel(checked);
    return passed ? 0 : 1;
  }
  if (argc >= 3 && std::string(argv[1]) == "--test-straight-drive-bundle") {
    if (!bundle.Load(argv[2])) { std::fprintf(stderr, "%s\n", bundle.Error().c_str()); return 1; }
    std::filesystem::path generated;
    if (!bundle.Generate(generated)) { std::fprintf(stderr, "%s\n", bundle.Error().c_str()); return 1; }
    std::array<char, 8192> diagnostic{};
    mjModel* checked = mj_loadXML(generated.string().c_str(), nullptr,
                                  diagnostic.data(), diagnostic.size());
    if (!checked) { std::fprintf(stderr, "%s\n", diagnostic.data()); return 1; }
    mjData* checked_data = mj_makeData(checked);
    const int player_body = mj_name2id(checked, mjOBJ_BODY, "Player_Robot");
    const int player_joint = mj_name2id(checked, mjOBJ_JOINT, "robot_free");
    Controls test_controls;
    bundle.BindControls(checked, test_controls.left, test_controls.right,
                        test_controls.weapon, test_controls.motor_models,
                        test_controls.weapon_motor_models,
                        test_controls.manual_velocity_actuators,
                        test_controls.battery_voltage);
    if (!checked_data || player_body < 0 || player_joint < 0 ||
        test_controls.left.empty() || test_controls.right.empty()) {
      if (checked_data) mj_deleteData(checked_data);
      mj_deleteModel(checked);
      std::fprintf(stderr, "Bundle test needs Player_Robot and assigned left/right drive wheels.\n");
      return 1;
    }
    const int root_qpos = checked->jnt_qposadr[player_joint];
    const int floor_geom = mj_name2id(checked, mjOBJ_GEOM, "floor");
    std::vector<int> scrape_geom_ids;
    for (int part = 0; part < bundle.PartCount(); ++part) {
      const int assembly = bundle.PartAssembly(part);
      const bool fixed_structure = assembly > 0 && assembly < bundle.AssemblyCount() &&
        bundle.Assembly(assembly).kind == AssemblyKind::Fixed;
      if (assembly != 0 && !fixed_structure && !bundle.IsForkPart(part)) continue;
      const std::string geom_name = "cad_part_" + std::to_string(part);
      const int geom = mj_name2id(checked, mjOBJ_GEOM, geom_name.c_str());
      if (geom >= 0) scrape_geom_ids.push_back(geom);
    }
    int scrape_floor_contact_steps = 0;
    int max_scrape_floor_contacts = 0;
    double max_scrape_floor_normal = 0.0;
    auto record_scrape_floor_contacts = [&]() {
      int contact_count = 0;
      for (int contact = 0; contact < checked_data->ncon; ++contact) {
        const mjContact& item = checked_data->contact[contact];
        if (item.geom[0] != floor_geom && item.geom[1] != floor_geom) continue;
        const int other = item.geom[0] == floor_geom ? item.geom[1] : item.geom[0];
        if (std::find(scrape_geom_ids.begin(), scrape_geom_ids.end(), other) == scrape_geom_ids.end()) continue;
        ++contact_count;
        mjtNum wrench[6]{};
        mj_contactForce(checked, checked_data, contact, wrench);
        max_scrape_floor_normal = std::max(max_scrape_floor_normal, double(wrench[0]));
      }
      if (contact_count > 0) ++scrape_floor_contact_steps;
      max_scrape_floor_contacts = std::max(max_scrape_floor_contacts, contact_count);
    };
    for (int geom = 0; geom < checked->ngeom; ++geom) {
      const int body = checked->geom_bodyid[geom];
      if (checked->geom_type[geom] == mjGEOM_PLANE ||
          body_is_in_subtree(checked, body, player_body)) continue;
      checked->geom_contype[geom] = 0;
      checked->geom_conaffinity[geom] = 0;
    }
    mj_forward(checked, checked_data);
    for (int step = 0; step < 1250; ++step) {
      apply_wheel_drive_targets(checked, checked_data, test_controls, 0.0, 0.0);
      mj_step(checked, checked_data);
      record_scrape_floor_contacts();
    }
    const int settle_scrape_contact_steps = scrape_floor_contact_steps;
    const int settle_max_scrape_contacts = max_scrape_floor_contacts;
    const double settle_max_scrape_normal = max_scrape_floor_normal;
    scrape_floor_contact_steps = 0;
    max_scrape_floor_contacts = 0;
    max_scrape_floor_normal = 0.0;
    const double start_x = checked_data->qpos[root_qpos];
    const double start_y = checked_data->qpos[root_qpos + 1];
    const double start_z = checked_data->xipos[3 * player_body + 2];
    double min_com_z = start_z;
    double max_com_z = start_z;
    auto yaw_at = [&](const mjData* sample) {
      const double w = sample->qpos[root_qpos + 3];
      const double qx = sample->qpos[root_qpos + 4];
      const double qy = sample->qpos[root_qpos + 5];
      const double qz = sample->qpos[root_qpos + 6];
      return std::atan2(2.0 * (w * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz));
    };
    const double start_yaw = yaw_at(checked_data);
    const auto left_estimate = side_motor_speed_estimate(test_controls.left,
      test_controls.motor_models, test_controls.battery_voltage);
    const auto right_estimate = side_motor_speed_estimate(test_controls.right,
      test_controls.motor_models, test_controls.battery_voltage);
    const double left_rpm = left_estimate.rpm;
    const double right_rpm = right_estimate.rpm;
    const auto [left_speed, right_speed] = mix_drive(1.0f, 0.0f,
      left_rpm * 2.0 * pi / 60.0, right_rpm * 2.0 * pi / 60.0);
    std::printf("CAD drive mapping left=%d right=%d, motorized wheel assemblies=%zu, battery=%.1f V, no-load estimate L/R=%.1f/%.1f RPM\n",
      static_cast<int>(test_controls.left.size()), static_cast<int>(test_controls.right.size()),
      test_controls.motor_models.size(), test_controls.battery_voltage, left_rpm, right_rpm);
    std::printf("Forward signs L=[");
    for (size_t i = 0; i < test_controls.left.size(); ++i)
      std::printf("%s%.0f", i ? "," : "", test_controls.left[i].second);
    std::printf("] R=[");
    for (size_t i = 0; i < test_controls.right.size(); ++i)
      std::printf("%s%.0f", i ? "," : "", test_controls.right[i].second);
    std::printf("]\n");
    for (int step = 0; step < 2500; ++step) {
      apply_wheel_drive_targets(checked, checked_data, test_controls, left_speed, right_speed);
      mj_step(checked, checked_data);
      record_scrape_floor_contacts();
      const double center_of_mass_z = checked_data->xipos[3 * player_body + 2];
      min_com_z = std::min(min_com_z, center_of_mass_z);
      max_com_z = std::max(max_com_z, center_of_mass_z);
    }
    const double yaw_change = std::remainder(yaw_at(checked_data) - start_yaw, 2.0 * pi);
    std::printf("CAD straight run (floor and bot only, 1.0 s): dx=%.4f m dy=%.4f m yaw=%.5f rad (%.3f deg)\n",
      checked_data->qpos[root_qpos] - start_x, checked_data->qpos[root_qpos + 1] - start_y,
      yaw_change, yaw_change * 180.0 / pi);
    std::printf("Bot COM height during drive: start=%.4f m end=%.4f m range=%.4f..%.4f m (%.1f mm span)\n",
      start_z, checked_data->xipos[3 * player_body + 2], min_com_z, max_com_z,
      (max_com_z - min_com_z) * 1000.0);
    std::printf("Chassis/fork-floor contact during settle: %d steps, up to %d contacts, max normal %.3f N; during drive: %d steps, up to %d contacts, max normal %.3f N\n",
      settle_scrape_contact_steps, settle_max_scrape_contacts, settle_max_scrape_normal,
      scrape_floor_contact_steps, max_scrape_floor_contacts, max_scrape_floor_normal);
    auto print_wheels = [&](const char* side,
                            const std::vector<std::pair<int, double>>& wheels) {
      std::printf("%s wheels:", side);
      for (size_t i = 0; i < wheels.size(); ++i) {
        const int id = wheels[i].first;
        const double rpm = actuator_rpm(checked, checked_data, id, wheels[i].second);
        const WheelSlip slip = wheel_slip(checked, checked_data, id);
        const int joint = checked->actuator_trnid[2 * id];
        const int body = checked->jnt_bodyid[joint];
        std::printf(" %s[z=%.3f m]=%+.1f RPM/%s", mj_id2name(checked, mjOBJ_ACTUATOR, id),
          checked_data->xpos[3 * body + 2], rpm,
          !slip.valid ? "unknown" : !slip.has_ground_contact ? "airborne" :
          slip.speed > 0.18 ? "slipping" : "grip");
        if (slip.valid && slip.has_ground_contact) std::printf("(%.2f m/s)", slip.speed);
      }
      std::printf("\n");
    };
    print_wheels("Left", test_controls.left);
    print_wheels("Right", test_controls.right);
    if (floor_geom >= 0) {
      mj_forward(checked, checked_data);
      std::printf("Floor contacts (body, geom, z, normal N):");
      for (int contact = 0; contact < checked_data->ncon; ++contact) {
        const mjContact& c = checked_data->contact[contact];
        if (c.geom[0] != floor_geom && c.geom[1] != floor_geom) continue;
        const int other = c.geom[0] == floor_geom ? c.geom[1] : c.geom[0];
        mjtNum wrench[6]{};
        mj_contactForce(checked, checked_data, contact, wrench);
        std::printf(" [%s/%s z=%.4f F=%.3f]",
          mj_id2name(checked, mjOBJ_BODY, checked->geom_bodyid[other]),
          mj_id2name(checked, mjOBJ_GEOM, other), c.pos[2], wrench[0]);
      }
      std::printf("\n");
    }
    const bool finite = std::isfinite(yaw_change) &&
      std::isfinite(checked_data->qpos[root_qpos]) &&
      std::isfinite(checked_data->qpos[root_qpos + 1]);
    mj_deleteData(checked_data); mj_deleteModel(checked);
    return finite ? 0 : 1;
  }
  if (argc >= 2 && std::string(argv[1]) == "--test-rpm-telemetry") {
    const std::filesystem::path telemetry_model = argc >= 3 ? argv[2] :
      stock_model_path();
    std::array<char, 4096> diagnostic{};
    mjModel* checked = mj_loadXML(telemetry_model.string().c_str(), nullptr,
                                  diagnostic.data(), diagnostic.size());
    if (!checked) { std::fprintf(stderr, "%s\n", diagnostic.data()); return 1; }
    mjData* checked_data = mj_makeData(checked);
    if (!checked_data) { mj_deleteModel(checked); return 1; }
    Controls telemetry_controls;
    bind_stock_actuators(checked, telemetry_controls);
    auto set_joint_rpm = [&](int actuator_id, double normalized_rpm, double direction) {
      const int joint_id = checked->actuator_trnid[2 * actuator_id];
      const int dof = checked->jnt_dofadr[joint_id];
      checked_data->qvel[dof] = normalized_rpm / direction * 2.0 * pi / 60.0;
    };
    for (size_t i = 0; i < telemetry_controls.left.size(); ++i)
      set_joint_rpm(telemetry_controls.left[i].first, 250.0 + 100.0 * i,
                    telemetry_controls.left[i].second);
    for (size_t i = 0; i < telemetry_controls.right.size(); ++i)
      set_joint_rpm(telemetry_controls.right[i].first, -350.0 - 100.0 * i,
                    telemetry_controls.right[i].second);
    if (telemetry_controls.weapon.empty() || telemetry_controls.left.empty() ||
        telemetry_controls.right.empty()) {
      mj_deleteData(checked_data); mj_deleteModel(checked);
      std::fprintf(stderr, "Stock model is missing a telemetry actuator.\n"); return 1;
    }
    set_joint_rpm(telemetry_controls.weapon.front(), 1800.0, 1.0);
    bool valid = std::abs(actuator_rpm(checked, checked_data,
      telemetry_controls.weapon.front()) - 1800.0) < 1e-6;
    for (size_t i = 0; i < telemetry_controls.left.size(); ++i)
      valid = valid && std::abs(actuator_rpm(checked, checked_data,
        telemetry_controls.left[i].first, telemetry_controls.left[i].second) -
        (250.0 + 100.0 * i)) < 1e-6;
    for (size_t i = 0; i < telemetry_controls.right.size(); ++i)
      valid = valid && std::abs(actuator_rpm(checked, checked_data,
        telemetry_controls.right[i].first, telemetry_controls.right[i].second) -
        (-350.0 - 100.0 * i)) < 1e-6;
    mj_deleteData(checked_data); mj_deleteModel(checked);
    if (!valid) { std::fprintf(stderr, "Actual RPM telemetry mapping failed.\n"); return 1; }
    std::puts("Actual RPM telemetry passed for the weapon and normalized wheel joints.");
    return 0;
  }
  if (argc >= 2 && std::string(argv[1]) == "--test-motor-torque") {
    MotorSpec motor;
    motor.enabled = true; motor.motor_count = 2; motor.kv_rpm_per_volt = 1000.0;
    motor.peak_current_a = 10.0; motor.gear_ratio = 10.0;
    motor.gearbox_efficiency = 0.9;
    constexpr double voltage = 11.1;
    const double kv_rad = motor.kv_rpm_per_volt * 2.0 * pi / 60.0;
    const double free_speed = kv_rad * voltage / motor.gear_ratio;
    const double expected_stall = (1.0 / kv_rad) * motor.peak_current_a *
      motor.gear_ratio * motor.gearbox_efficiency * motor.motor_count;
    const double estimated_no_load_rpm = MotorNoLoadOutputRpm(motor, voltage);
    const double ideal_stall = MotorOutputTorque(motor, voltage, free_speed, 0.0);
    const double ideal_free = MotorOutputTorque(motor, voltage, free_speed, free_speed);
    const double shared_wheel_torque = motor_group_torque_per_wheel(
      motor, voltage, free_speed, 0.0, 2);
    motor.phase_resistance_ohm = voltage / motor.peak_current_a;
    const double resistive_stall = MotorOutputTorque(motor, voltage, free_speed, 0.0);
    const double resistive_free = MotorOutputTorque(motor, voltage, free_speed, free_speed);
    MotorSpec brushed = motor;
    brushed.type = MotorType::Brushed;
    brushed.speed_input = MotorSpeedInput::RatedRpmAtVoltage;
    brushed.rated_rpm = 6200.0;
    brushed.rated_voltage = 12.0;
    brushed.phase_resistance_ohm = 0.0;
    const double brushed_kv = 6200.0 / 12.0;
    const double brushed_free_rpm = 6200.0 * voltage / 12.0 / brushed.gear_ratio;
    const double brushed_free_speed = brushed_free_rpm * 2.0 * pi / 60.0;
    const double brushed_stall_torque = MotorOutputTorque(
      brushed, voltage, brushed_free_speed, 0.0);
    const double expected_brushed_stall = (1.0 / (brushed_kv * 2.0 * pi / 60.0)) *
      brushed.peak_current_a * brushed.gear_ratio * brushed.gearbox_efficiency *
      brushed.motor_count;
    if (std::abs(ideal_stall - expected_stall) > 1e-8 ||
        std::abs(resistive_stall - expected_stall) > 1e-8 ||
        std::abs(2.0 * shared_wheel_torque - expected_stall) > 1e-8 ||
        std::abs(estimated_no_load_rpm - 1110.0) > 1e-8 ||
        std::abs(ideal_free) > 1e-8 || std::abs(resistive_free) > 1e-8 ||
        std::abs(MotorKvRpmPerVolt(brushed) - brushed_kv) > 1e-8 ||
        std::abs(MotorNoLoadOutputRpm(brushed, voltage) - brushed_free_rpm) > 1e-8 ||
        std::abs(brushed_stall_torque - expected_brushed_stall) > 1e-8 ||
        brushed.phase_resistance_ohm != 0.0) {
      std::fprintf(stderr, "Motor torque model failed its stall/free-speed checks.\n");
      return 1;
    }
    brushed.speed_input = MotorSpeedInput::Kv;
    brushed.kv_rpm_per_volt = brushed_kv;
    if (std::abs(MotorNoLoadOutputRpm(brushed, voltage) - brushed_free_rpm) > 1e-8) {
      std::fprintf(stderr, "Brushed Kv and rated-RPM speed inputs disagree.\n");
      return 1;
    }
    std::printf("Motor torque/RPM passed: %.4f Nm brushless stall, %.0f RPM brushless no-load; brushed rated-RPM input and optional resistance passed.\n",
                expected_stall, estimated_no_load_rpm);
    return 0;
  }
  if (argc >= 3 && std::string(argv[1]) == "--test-motor-actuator") {
    if (!bundle.Load(argv[2])) { std::fprintf(stderr, "%s\n", bundle.Error().c_str()); return 1; }
    std::filesystem::path generated;
    if (!bundle.Generate(generated)) { std::fprintf(stderr, "%s\n", bundle.Error().c_str()); return 1; }
    std::array<char, 8192> diagnostic{};
    mjModel* checked = mj_loadXML(generated.string().c_str(), nullptr,
                                  diagnostic.data(), diagnostic.size());
    if (!checked) { std::fprintf(stderr, "%s\n", diagnostic.data()); return 1; }
    std::vector<std::pair<int, double>> left, right;
    std::vector<int> weapons;
    std::map<int, MotorSpec> motors;
    std::map<int, MotorSpec> weapon_motors;
    std::map<int, int> manual_velocity_actuators;
    double voltage = 0.0;
    bundle.BindControls(checked, left, right, weapons, motors,
                        weapon_motors, manual_velocity_actuators, voltage);
    const auto motor_it = std::find_if(motors.begin(), motors.end(), [&](const auto& item) {
      const auto is_wheel = [&](const auto& side) {
        return std::find_if(side.begin(), side.end(), [&](const auto& wheel) {
          return wheel.first == item.first;
        }) != side.end();
      };
      return is_wheel(left) || is_wheel(right);
    });
    mjData* checked_data = mj_makeData(checked);
    if (motor_it == motors.end() || !checked_data) {
      if (checked_data) mj_deleteData(checked_data);
      mj_deleteModel(checked);
      std::fprintf(stderr, "Test bundle has no configured drive wheel motor.\n"); return 1;
    }
    std::vector<int> shared_motor_actuators;
    for (const auto& [id, motor] : motors)
      if (motor.drive_group == motor_it->second.drive_group)
        shared_motor_actuators.push_back(id);
    checked->opt.gravity[0] = checked->opt.gravity[1] = checked->opt.gravity[2] = 0.0;
    for (int geom = 0; geom < checked->ngeom; ++geom) {
      checked->geom_contype[geom] = 0;
      checked->geom_conaffinity[geom] = 0;
    }
    // Explicit MuJoCo pairs bypass geom collision masks. Disable them too in
    // this isolated motor-speed test so CAD floor-contact pairs cannot load a
    // wheel while the test is measuring its free speed.
    checked->npair = 0;
    mj_forward(checked, checked_data);
    const int actuator_id = motor_it->first;
    const int joint_id = checked->actuator_trnid[2 * actuator_id];
    const MotorSpec& motor = motor_it->second;
    const double free_output_speed = MotorNoLoadOutputRpm(motor, voltage) *
      2.0 * pi / 60.0;
    Controls test_controls;
    test_controls.left = left;
    test_controls.right = right;
    test_controls.motor_models = motors;
    test_controls.manual_velocity_actuators = manual_velocity_actuators;
    test_controls.battery_voltage = voltage;
    for (int step = 0; step < 2500; ++step) {
      apply_wheel_drive_targets(checked, checked_data, test_controls,
        free_output_speed, free_output_speed);
      mj_step(checked, checked_data);
    }
    const int dof = checked->jnt_dofadr[joint_id];
    const double final_rpm = std::abs(checked_data->qvel[dof]) * 60.0 / (2.0 * pi);
    const double free_rpm = free_output_speed * 60.0 / (2.0 * pi);
    bool finite = shared_motor_actuators.size() >= 2 && std::isfinite(final_rpm);
    for (const int shared_id : shared_motor_actuators) {
      const int shared_joint = checked->actuator_trnid[2 * shared_id];
      const int shared_dof = checked->jnt_dofadr[shared_joint];
      const double shared_rpm = std::abs(checked_data->qvel[shared_dof]) * 60.0 / (2.0 * pi);
      finite = finite && std::isfinite(shared_rpm) && shared_rpm >= 0.1 * free_rpm &&
        shared_rpm <= 1.05 * free_rpm;
    }
    if (!finite || final_rpm < 0.1 * free_rpm || final_rpm > 1.05 * free_rpm) {
      mj_deleteData(checked_data); mj_deleteModel(checked);
      std::fprintf(stderr, "Motor actuator spin check was %.1f RPM versus %.1f RPM free speed.\n",
                   final_rpm, free_rpm);
      return 1;
    }
    std::printf("Shared motor spin test passed: %d wheel assemblies, %.0f RPM of %.0f RPM no-load.\n",
                static_cast<int>(shared_motor_actuators.size()), final_rpm, free_rpm);
    test_controls.manual_wheel_rpm_override = true;
    constexpr double manual_target_rpm = 4000.0;
    const double manual_target_speed = manual_target_rpm * 2.0 * pi / 60.0;
    mj_resetData(checked, checked_data);
    mj_forward(checked, checked_data);
    for (int step = 0; step < 3000; ++step) {
      apply_wheel_drive_targets(checked, checked_data, test_controls,
        manual_target_speed, manual_target_speed);
      mj_step(checked, checked_data);
    }
    bool manual_rpm_ok = true;
    double manual_actual_rpm = 0.0;
    for (const int shared_id : shared_motor_actuators) {
      const auto left_wheel = std::find_if(left.begin(), left.end(), [&](const auto& item) {
        return item.first == shared_id;
      });
      double forward_sign = 1.0;
      if (left_wheel != left.end()) {
        forward_sign = left_wheel->second;
      } else {
        const auto right_wheel = std::find_if(right.begin(), right.end(), [&](const auto& item) {
          return item.first == shared_id;
        });
        if (right_wheel == right.end()) {
          manual_rpm_ok = false;
          continue;
        }
        forward_sign = right_wheel->second;
      }
      const double rpm = actuator_rpm(checked, checked_data, shared_id, forward_sign);
      manual_actual_rpm += rpm;
      manual_rpm_ok = manual_rpm_ok && std::isfinite(rpm) &&
        std::abs(rpm - manual_target_rpm) < 0.05 * manual_target_rpm;
    }
    manual_actual_rpm /= static_cast<double>(shared_motor_actuators.size());
    mj_deleteData(checked_data); mj_deleteModel(checked);
    if (!manual_rpm_ok) {
      std::fprintf(stderr, "Manual wheel RPM override reached %.0f RPM, expected %.0f.\n",
                   manual_actual_rpm, manual_target_rpm);
      return 1;
    }
    std::printf("Manual speed override passed: %.0f RPM target reached at %.0f RPM average.\n",
                manual_target_rpm, manual_actual_rpm);
    return 0;
  }
  if (argc >= 3 && std::string(argv[1]) == "--test-drive-layout") {
    if (!bundle.Load(argv[2])) {
      std::fprintf(stderr, "%s\n", bundle.Error().c_str());
      return 1;
    }
    auto verify_layout = [&](DriveLayout layout, const char* axle,
                             int expected_left, int expected_right) {
      bundle.SetWheelDriveLayout(layout);
      if (layout == DriveLayout::TwoWheel) bundle.SetTwoWheelDriveAxle(axle);
      std::filesystem::path generated;
      if (!bundle.Generate(generated)) {
        std::fprintf(stderr, "%s\n", bundle.Error().c_str());
        return false;
      }
      std::array<char, 8192> diagnostic{};
      mjModel* checked = mj_loadXML(generated.string().c_str(), nullptr,
                                    diagnostic.data(), diagnostic.size());
      if (!checked) {
        std::fprintf(stderr, "Drive-layout model failed to load: %s\n", diagnostic.data());
        return false;
      }
      std::vector<std::pair<int, double>> left, right;
      std::vector<int> weapon;
      std::map<int, MotorSpec> motors, weapon_motors;
      std::map<int, int> manual_velocity_actuators;
      double voltage = 0.0;
      bundle.BindControls(checked, left, right, weapon, motors, weapon_motors,
                          manual_velocity_actuators, voltage);
      const bool valid = int(left.size()) == expected_left &&
                         int(right.size()) == expected_right;
      mj_deleteModel(checked);
      if (!valid)
        std::fprintf(stderr, "Drive-layout mapping was left=%d/right=%d; expected left=%d/right=%d.\n",
                     int(left.size()), int(right.size()), expected_left, expected_right);
      return valid;
    };
    if (!verify_layout(DriveLayout::TwoWheel, "front", 1, 1) ||
        !verify_layout(DriveLayout::TwoWheel, "rear", 1, 1) ||
        !verify_layout(DriveLayout::FourWheel, "front", 2, 2)) return 1;
    if (!bundle.Save()) {
      std::fprintf(stderr, "%s\n", bundle.Error().c_str());
      return 1;
    }
    std::puts("2WD front/rear axle selection and return to 4WD passed.");
    return 0;
  }
  if (argc >= 3 && std::string(argv[1]) == "--test-weapon-motor") {
    if (!bundle.Load(argv[2])) { std::fprintf(stderr, "%s\n", bundle.Error().c_str()); return 1; }
    std::filesystem::path generated;
    if (!bundle.Generate(generated)) { std::fprintf(stderr, "%s\n", bundle.Error().c_str()); return 1; }
    std::array<char, 8192> diagnostic{};
    mjModel* checked = mj_loadXML(generated.string().c_str(), nullptr,
                                  diagnostic.data(), diagnostic.size());
    if (!checked) { std::fprintf(stderr, "%s\n", diagnostic.data()); return 1; }
    mjData* checked_data = mj_makeData(checked);
    Controls test_controls;
    bundle.BindControls(checked, test_controls.left, test_controls.right,
      test_controls.weapon, test_controls.motor_models, test_controls.weapon_motor_models,
      test_controls.manual_velocity_actuators, test_controls.battery_voltage);
    test_controls.weapon_sign = 1.0;
    if (!checked_data || test_controls.weapon.empty() ||
        !weapon_uses_torque_model(test_controls)) {
      if (checked_data) mj_deleteData(checked_data);
      mj_deleteModel(checked);
      std::fprintf(stderr, "Test bundle needs a motor model on every weapon assembly.\n");
      return 1;
    }
    checked->opt.gravity[0] = checked->opt.gravity[1] = checked->opt.gravity[2] = 0.0;
    for (int geom = 0; geom < checked->ngeom; ++geom) {
      checked->geom_contype[geom] = 0;
      checked->geom_conaffinity[geom] = 0;
    }
    // This measures the motor model in isolation, without any generated
    // floor or fork/frame contact pairs adding a mechanical load.
    checked->npair = 0;
    mj_forward(checked, checked_data);
    const double estimated_spinup_seconds = bundle.EstimatedWeaponSpinupSeconds();
    double early_rpm = 0.0;
    constexpr int early_steps = 250;
    constexpr int total_steps = 2500;
    for (int step = 0; step < total_steps; ++step) {
      apply_weapon_drive_targets(checked, checked_data, test_controls, true);
      mj_step(checked, checked_data);
      if (step == early_steps - 1) {
        for (const int id : test_controls.weapon)
          early_rpm += std::abs(actuator_rpm(checked, checked_data, id, 1.0));
        early_rpm /= static_cast<double>(test_controls.weapon.size());
      }
    }
    double final_rpm = 0.0;
    for (const int id : test_controls.weapon)
      final_rpm += std::abs(actuator_rpm(checked, checked_data, id, 1.0));
    final_rpm /= static_cast<double>(test_controls.weapon.size());
    const double free_rpm = weapon_no_load_rpm(test_controls);
    const bool passed = std::isfinite(estimated_spinup_seconds) &&
      estimated_spinup_seconds > 0.0 && std::isfinite(early_rpm) && std::isfinite(final_rpm) &&
      early_rpm > 0.0 && early_rpm < final_rpm &&
      final_rpm >= 0.1 * free_rpm && final_rpm <= 1.05 * free_rpm;
    mj_deleteData(checked_data); mj_deleteModel(checked);
    if (!passed) {
      std::fprintf(stderr, "Weapon motor spin test failed: %.0f RPM at 0.1 s, %.0f RPM final, %.0f RPM no-load.\n",
                   early_rpm, final_rpm, free_rpm);
      return 1;
    }
    std::printf("Weapon torque spin-up passed: %.0f RPM at 0.1 s, %.0f RPM final of %.0f RPM no-load; estimated 0-to-90%% time %.2f s.\n",
                  early_rpm, final_rpm, free_rpm, estimated_spinup_seconds);
    return 0;
  }
  if (argc >= 2 && std::string(argv[1]) == "--test-wall-impact") {
    const std::filesystem::path wall_model = argc >= 3 ? argv[2] :
      stock_model_path();
    auto run_wall_case = [&](double timestep, double impact_speed, bool vertical = false) {
      std::array<char, 4096> diagnostic{};
      mjModel* checked = mj_loadXML(wall_model.string().c_str(), nullptr,
                                    diagnostic.data(), diagnostic.size());
      if (!checked) { std::fprintf(stderr, "%s\n", diagnostic.data()); return 1e9; }
      checked->opt.timestep = timestep;
      mjData* checked_data = mj_makeData(checked);
      const int player_joint = mj_name2id(checked, mjOBJ_JOINT, "robot_free");
      const int target_joint = mj_name2id(checked, mjOBJ_JOINT, "target_free");
      if (!checked_data || player_joint < 0 || target_joint < 0) {
        if (checked_data) mj_deleteData(checked_data);
        mj_deleteModel(checked);
        return 1e9;
      }
      const int player_qpos = checked->jnt_qposadr[player_joint];
      const int player_dof = checked->jnt_dofadr[player_joint];
      const int target_qpos = checked->jnt_qposadr[target_joint];
      checked_data->qpos[player_qpos] = 0.0;
      checked_data->qpos[player_qpos + 1] = 0.0;
      checked_data->qpos[player_qpos + 2] = 0.065;
      checked_data->qpos[target_qpos] = -0.55;
      checked_data->qpos[target_qpos + 1] = 0.68;
      checked_data->qpos[target_qpos + 2] = 0.065;
      checked_data->qvel[player_dof + (vertical ? 2 : 0)] = impact_speed;
      mj_forward(checked, checked_data);
      double peak_position = checked_data->qpos[player_qpos + (vertical ? 2 : 0)];
      const int steps = int(std::ceil(0.12 / timestep));
      for (int i = 0; i < steps; ++i) {
        mj_step(checked, checked_data);
        peak_position = std::max(peak_position,
          checked_data->qpos[player_qpos + (vertical ? 2 : 0)]);
      }
      const double end_position = checked_data->qpos[player_qpos + (vertical ? 2 : 0)];
      std::printf("%s impact at %.0f Hz and %.1f m/s: peak robot center=%.4f m, final=%.4f m\n",
        vertical ? "Roof" : "Wall", 1.0 / timestep, impact_speed,
        peak_position, end_position);
      mj_deleteData(checked_data); mj_deleteModel(checked);
      return peak_position;
    };
    auto run_corner_case = [&](double timestep) {
      std::array<char, 4096> diagnostic{};
      mjModel* checked = mj_loadXML(wall_model.string().c_str(), nullptr,
                                    diagnostic.data(), diagnostic.size());
      if (!checked) { std::fprintf(stderr, "%s\n", diagnostic.data()); return 1e9; }
      apply_arena_size(checked, 2.0);
      checked->opt.timestep = timestep;
      mjData* checked_data = mj_makeData(checked);
      const int player_joint = mj_name2id(checked, mjOBJ_JOINT, "robot_free");
      const int target_joint = mj_name2id(checked, mjOBJ_JOINT, "target_free");
      if (!checked_data || player_joint < 0 || target_joint < 0) {
        if (checked_data) mj_deleteData(checked_data);
        mj_deleteModel(checked);
        return 1e9;
      }
      const int player_qpos = checked->jnt_qposadr[player_joint];
      const int player_dof = checked->jnt_dofadr[player_joint];
      const int target_qpos = checked->jnt_qposadr[target_joint];
      checked_data->qpos[player_qpos] = 0.45;
      checked_data->qpos[player_qpos + 1] = 0.45;
      checked_data->qpos[player_qpos + 2] = 0.065;
      checked_data->qpos[target_qpos] = -0.55;
      checked_data->qpos[target_qpos + 1] = -0.55;
      checked_data->qpos[target_qpos + 2] = 0.065;
      checked_data->qvel[player_dof] = 180.0;
      checked_data->qvel[player_dof + 1] = 180.0;
      mj_forward(checked, checked_data);
      double peak_extent = std::max(checked_data->qpos[player_qpos],
                                    checked_data->qpos[player_qpos + 1]);
      const int steps = int(std::ceil(0.08 / timestep));
      for (int i = 0; i < steps; ++i) {
        mj_step(checked, checked_data);
        peak_extent = std::max(peak_extent,
          std::max(double(checked_data->qpos[player_qpos]),
                   double(checked_data->qpos[player_qpos + 1])));
      }
      std::printf("Diagonal corner impact at %.0f Hz: peak robot center extent=%.4f m\n",
                  1.0 / timestep, peak_extent);
      mj_deleteData(checked_data);
      mj_deleteModel(checked);
      return peak_extent;
    };
    const double at_2500 = run_wall_case(0.0004, 24.0);
    const double at_10000 = run_wall_case(0.0001, 24.0);
    const double at_2500_fast = run_wall_case(0.0004, 100.0);
    const double at_10000_fast = run_wall_case(0.0001, 100.0);
    const double at_2500_extreme = run_wall_case(0.0004, 180.0);
    const double at_10000_extreme = run_wall_case(0.0001, 180.0);
    const double roof_2500 = run_wall_case(0.0004, 100.0, true);
    const double roof_10000 = run_wall_case(0.0001, 100.0, true);
    const double corner_2500 = run_corner_case(0.0004);
    const double corner_10000 = run_corner_case(0.0001);
    if (at_2500 > 1.25 || at_10000 > 1.25 ||
        at_2500_fast > 0.84 || at_10000_fast > 0.84 ||
        at_2500_extreme > 1.25 || at_10000_extreme > 1.25 ||
        roof_2500 > 1.25 || roof_10000 > 1.25 ||
        corner_2500 > 1.90 || corner_10000 > 1.90) {
      std::fprintf(stderr, "Robot escaped the arena's hidden wall or roof catch volumes.\n");
      return 1;
    }
    std::puts("Wall, roof, and diagonal corner impacts stayed inside the hidden catch volumes at both update rates.");
    return 0;
  }
  if (argc >= 3 && std::string(argv[1]) == "--inspect-model") {
    std::array<char, 8192> diagnostic{};
    mjModel* checked = mj_loadXML(argv[2], nullptr, diagnostic.data(), diagnostic.size());
    if (!checked) { std::fprintf(stderr, "%s\n", diagnostic.data()); return 1; }
    mjData* checked_data = mj_makeData(checked);
    mj_forward(checked, checked_data);
    mjvCamera checked_camera; mjv_defaultCamera(&checked_camera);
    mjvOption checked_options; mjv_defaultOption(&checked_options);
    mjvScene checked_scene; mjv_defaultScene(&checked_scene);
    mjv_makeScene(checked, &checked_scene, 2000);
    mjv_updateScene(checked, checked_data, &checked_options, nullptr, &checked_camera,
                    mjCAT_ALL, &checked_scene);
    std::printf("Model: %d geoms, %d meshes, %d visible scene geoms\n",
                checked->ngeom, checked->nmesh, checked_scene.ngeom);
    if (checked_scene.ngeom) {
      const auto& first = checked_scene.geoms[0];
      std::printf("First scene geom: type=%d rgba=%.2f %.2f %.2f %.2f pos=%.3f %.3f %.3f size=%.3f %.3f %.3f\n",
                  int(first.type), first.rgba[0], first.rgba[1], first.rgba[2], first.rgba[3],
                  first.pos[0], first.pos[1], first.pos[2], first.size[0], first.size[1], first.size[2]);
    }
    mjv_freeScene(&checked_scene); mj_deleteData(checked_data); mj_deleteModel(checked);
    return 0;
  }
  if (argc >= 3 && std::string(argv[1]) == "--test-spin") {
    std::array<char, 8192> diagnostic{};
    mjModel* checked = mj_loadXML(argv[2], nullptr, diagnostic.data(), diagnostic.size());
    if (!checked) { std::fprintf(stderr, "%s\n", diagnostic.data()); return 1; }
    mjData* checked_data = mj_makeData(checked);
    const int root_joint = mj_name2id(checked, mjOBJ_JOINT, "robot_free");
    const int weapon_joint = mj_name2id(checked, mjOBJ_JOINT, "cad_joint_1");
    const int weapon_actuator = mj_name2id(checked, mjOBJ_ACTUATOR, "cad_assembly_1_drive");
    const int weapon_body = mj_name2id(checked, mjOBJ_BODY, "cad_assembly_1");
    if (!checked_data || root_joint < 0 || weapon_joint < 0 || weapon_actuator < 0 || weapon_body < 0) {
      std::fprintf(stderr, "Model needs robot_free, cad_joint_1, and cad_assembly_1_drive.\n");
      mj_deleteData(checked_data); mj_deleteModel(checked); return 1;
    }
    const mjtNum* center = checked->body_ipos + 3 * weapon_body;
    const mjtNum* axis = checked->jnt_axis + 3 * weapon_joint;
    const double axis_length = std::sqrt(axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2]);
    const double center_along_axis = (center[0]*axis[0] + center[1]*axis[1] + center[2]*axis[2]) /
      std::max(1e-12, axis_length);
    const double radial_com_offset = std::sqrt(
      std::pow(center[0] - center_along_axis*axis[0]/std::max(1e-12, axis_length), 2) +
      std::pow(center[1] - center_along_axis*axis[1]/std::max(1e-12, axis_length), 2) +
      std::pow(center[2] - center_along_axis*axis[2]/std::max(1e-12, axis_length), 2));
    mj_resetData(checked, checked_data);
    mj_forward(checked, checked_data);
    const int root_dof = checked->jnt_dofadr[root_joint];
    const int weapon_dof = checked->jnt_dofadr[weapon_joint];
    constexpr int settle_steps = 2500;
    constexpr int spinup_steps = 3750;
    constexpr int hold_steps = 2500;
    constexpr double target_rpm = 30000.0;
    constexpr double target_speed = target_rpm * 2.0 * pi / 60.0;
    double max_linear = 0, max_angular = 0, angular_sq_sum = 0;
    int measured = 0;
    for (int step = 0; step < settle_steps + spinup_steps + hold_steps; ++step) {
      double command = 0.0;
      if (step >= settle_steps)
        command = target_speed * std::clamp(double(step - settle_steps + 1) / spinup_steps, 0.0, 1.0);
      checked_data->ctrl[weapon_actuator] = -command;
      mj_step(checked, checked_data);
      for (int i = 0; i < checked->nq; ++i) if (!std::isfinite(checked_data->qpos[i])) {
        std::fprintf(stderr, "Non-finite qpos during weapon spin test at step %d.\n", step);
        mj_deleteData(checked_data); mj_deleteModel(checked); return 1;
      }
      for (int i = 0; i < checked->nv; ++i) if (!std::isfinite(checked_data->qvel[i])) {
        std::fprintf(stderr, "Non-finite qvel during weapon spin test at step %d.\n", step);
        mj_deleteData(checked_data); mj_deleteModel(checked); return 1;
      }
      if (step >= settle_steps + spinup_steps) {
        const double linear = std::sqrt(checked_data->qvel[root_dof]*checked_data->qvel[root_dof] +
          checked_data->qvel[root_dof+1]*checked_data->qvel[root_dof+1] +
          checked_data->qvel[root_dof+2]*checked_data->qvel[root_dof+2]);
        const double angular = std::sqrt(checked_data->qvel[root_dof+3]*checked_data->qvel[root_dof+3] +
          checked_data->qvel[root_dof+4]*checked_data->qvel[root_dof+4] +
          checked_data->qvel[root_dof+5]*checked_data->qvel[root_dof+5]);
        max_linear = std::max(max_linear, linear);
        max_angular = std::max(max_angular, angular);
        angular_sq_sum += angular * angular;
        ++measured;
      }
    }
    std::printf("Weapon spin: %.0f rpm target; final %.0f rpm; actual radial COM offset %.3f mm; root max linear %.3f m/s, max angular %.3f rad/s, post-spin RMS angular %.3f rad/s\n",
      target_rpm, std::abs(checked_data->qvel[weapon_dof]) * 60.0 / (2.0*pi),
      radial_com_offset * 1000.0, max_linear, max_angular,
      std::sqrt(angular_sq_sum / std::max(1, measured)));
    mj_deleteData(checked_data); mj_deleteModel(checked);
    return 0;
  }
  if (argc >= 4 && std::string(argv[1]) == "--test-clicks") {
    if (!bundle.Load(argv[3])) { std::fprintf(stderr, "%s\n", bundle.Error().c_str()); return 1; }
    std::array<char, 8192> diagnostic{};
    mjModel* checked = mj_loadXML(argv[2], nullptr, diagnostic.data(), diagnostic.size());
    if (!checked) { std::fprintf(stderr, "%s\n", diagnostic.data()); return 1; }
    mjData* checked_data = mj_makeData(checked);
    mj_forward(checked, checked_data);
    mjvCamera checked_camera; mjv_defaultCamera(&checked_camera);
    checked_camera.azimuth = 45; checked_camera.elevation = -45;
    const auto center = bundle.PreviewCenter();
    for (int i = 0; i < 3; ++i) checked_camera.lookat[i] = center[i];
    checked_camera.distance = bundle.PreviewDistance();
    mjvOption checked_options; mjv_defaultOption(&checked_options);
    mjvScene checked_scene; mjv_defaultScene(&checked_scene);
    mjv_makeScene(checked, &checked_scene, 2000);
    mjv_updateScene(checked, checked_data, &checked_options, nullptr, &checked_camera,
                    mjCAT_ALL, &checked_scene);
    bundle.AddSetupAxisMarkers(checked_scene);
    int hits = 0;
    for (int y = 1; y < 20; ++y) for (int x = 1; x < 20; ++x) {
      mjtNum point[3]; int geom = -1, flex = -1, skin = -1;
      mjv_select(checked, checked_data, &checked_options, 1.3,
                 x/20.0, y/20.0, &checked_scene, point, &geom, &flex, &skin);
      if (geom >= 0) {
        ++hits;
        const char* name = mj_id2name(checked, mjOBJ_GEOM, geom);
        if (name && std::string(name).rfind("cad_part_", 0) == 0) {
          const int part = std::stoi(std::string(name).substr(9));
          bundle.SelectPart(part);
          bundle.PickFace(part, {point[0], point[1], point[2]});
          bundle.PickEdge(part, {point[0], point[1], point[2]});
        }
      }
    }
    std::printf("Click-tested %d CAD hits\n", hits);
    for (int i = 0; i < checked->ngeom; ++i) checked->geom_group[i] = 5;
    checked_options.geomgroup[5] = 0;
    mjv_updateScene(checked, checked_data, &checked_options, nullptr, &checked_camera,
                    mjCAT_ALL, &checked_scene);
    for (int y = 1; y < 20; ++y) for (int x = 1; x < 20; ++x) {
      mjtNum point[3]; int geom = -1, flex = -1, skin = -1;
      mjv_select(checked, checked_data, &checked_options, 1.3,
                 x/20.0, y/20.0, &checked_scene, point, &geom, &flex, &skin);
      if (geom >= 0) {
        std::fprintf(stderr, "Hidden CAD part remained pickable.\n");
        mjv_freeScene(&checked_scene); mj_deleteData(checked_data); mj_deleteModel(checked);
        return 1;
      }
    }
    mjv_freeScene(&checked_scene); mj_deleteData(checked_data); mj_deleteModel(checked);
    return 0;
  }
  if (argc >= 3 && std::string(argv[1]) == "--test-hole-axis") {
    if (!bundle.Load(argv[2])) { std::fprintf(stderr, "%s\n", bundle.Error().c_str()); return 1; }
    if (bundle.Assembly(1).kind == AssemblyKind::Fixed) {
      std::fprintf(stderr, "Hole-axis test requires assembly 1 to be a rotating assembly.\n");
      return 1;
    }
    bundle.StartAxis(1);
    bundle.UpdateHover(1, {0.010, 0.0, 0.004});
    if (bundle.HoveredPart() != 1 || bundle.HoveredLoopPoints() < 6 ||
        !bundle.SelectHoveredHole()) {
      std::fprintf(stderr, "Could not use an outside-assembly part as the axis reference: %s\n",
                   bundle.PickMessage().c_str());
      return 1;
    }
    if (bundle.PickMessage().rfind("Fitted Weapon axis at (", 0) != 0) {
      std::fprintf(stderr, "Fitted axis result was not reported to the editor: %s\n",
                   bundle.PickMessage().c_str());
      return 1;
    }
    const auto& axis = bundle.Assembly(1);
    if (std::abs(axis.pivot[0]) > 0.0005 || std::abs(axis.pivot[1]) > 0.0005 ||
        std::abs(axis.pivot[2] - 0.004) > 0.0005 || std::abs(axis.axis[2]) < 0.99) {
      std::fprintf(stderr, "Fitted hole axis does not match the synthetic rim.\n");
      return 1;
    }
    if (!bundle.Save()) { std::fprintf(stderr, "%s\n", bundle.Error().c_str()); return 1; }
    FusionBundle reloaded;
    if (!reloaded.Load(argv[2]) || reloaded.AxisAssembly() != 1 ||
        reloaded.PartAssembly(0) != 1 || reloaded.PartAssembly(1) != 0 ||
        std::abs(reloaded.Assembly(1).pivot[2] - axis.pivot[2]) > 0.0005) {
      std::fprintf(stderr, "Saved assignments changed or target axis did not persist.\n");
      return 1;
    }
    std::printf("Outside-assembly reference part fitted Weapon axis without reassignment: center=(%.4f, %.4f, %.4f) m\n",
                axis.pivot[0], axis.pivot[1], axis.pivot[2]);
    return 0;
  }
  if (argc >= 8 && std::string(argv[1]) == "--test-axis-point") {
    if (!bundle.Load(argv[2])) { std::fprintf(stderr, "%s\n", bundle.Error().c_str()); return 1; }
    int target = -1, part = -1;
    std::array<double, 3> hit{};
    try {
      target = std::stoi(argv[3]); part = std::stoi(argv[4]);
      for (int i = 0; i < 3; ++i) hit[i] = std::stod(argv[5+i]);
    } catch (...) {
      std::fprintf(stderr, "Expected assembly, part, and three world-space hit coordinates.\n");
      return 1;
    }
    bundle.StartAxis(target);
    if (!bundle.AxisPicking()) {
      std::fprintf(stderr, "Could not start axis picking: %s\n", bundle.PickMessage().c_str());
      return 1;
    }
    bundle.UpdateHover(part, hit);
    if (bundle.HoveredLoopPoints() < 6) {
      std::fprintf(stderr, "No round rim highlighted at the supplied point: %s\n",
                   bundle.PickMessage().c_str());
      return 1;
    }
    const int points = int(bundle.HoveredLoopPoints());
    if (!bundle.SelectHoveredHole()) {
      std::fprintf(stderr, "Highlighted rim could not fit an axis: %s\n",
                   bundle.PickMessage().c_str());
      return 1;
    }
    const auto& axis = bundle.Assembly(target);
    std::printf("Axis target '%s': part %d highlighted a %d-point round rim and fitted "
                "center %.5f %.5f %.5f m, direction %.4f %.4f %.4f\n",
                axis.name.c_str(), part, points, axis.pivot[0], axis.pivot[1], axis.pivot[2],
                axis.axis[0], axis.axis[1], axis.axis[2]);
    return 0;
  }
  if (argc >= 4 && std::string(argv[1]) == "--test-axis-hover") {
    if (!bundle.Load(argv[3])) { std::fprintf(stderr, "%s\n", bundle.Error().c_str()); return 1; }
    int target = bundle.AxisAssembly();
    if (argc >= 5) {
      try { target = std::stoi(argv[4]); }
      catch (...) {
        std::fprintf(stderr, "Axis test target must be an assembly index.\n");
        return 1;
      }
    }
    if (target <= 0) {
      std::fprintf(stderr, "Bundle has no rotating assembly selected for the hole test.\n");
      return 1;
    }
    std::array<char, 8192> diagnostic{};
    mjModel* checked = mj_loadXML(argv[2], nullptr, diagnostic.data(), diagnostic.size());
    if (!checked) { std::fprintf(stderr, "%s\n", diagnostic.data()); return 1; }
    mjData* checked_data = mj_makeData(checked);
    mj_forward(checked, checked_data);
    mjvCamera checked_camera; mjv_defaultCamera(&checked_camera);
    const auto pivot = bundle.Assembly(target).pivot;
    for (int i = 0; i < 3; ++i) checked_camera.lookat[i] = pivot[i];
    checked_camera.distance = 0.13;
    mjvOption checked_options; mjv_defaultOption(&checked_options);
    for (int i = 0; i < checked->ngeom; ++i)
      checked->geom_group[i] = bundle.PreviewVisible(i) ? 0 : 5;
    checked_options.geomgroup[5] = 0;
    mjvScene checked_scene; mjv_defaultScene(&checked_scene);
    mjv_makeScene(checked, &checked_scene, 2000);
    const int azimuths[] = {0, 45, 90, 135, 180, 225, 270, 315};
    const int elevations[] = {-65, -35, 0, 35, 65};
    int rays = 0;
    int found_part = -1;
    int highlight_segments = 0;
    bool highlight_render_ready = true;
    bool fit = false;
    bundle.StartAxis(target);
    if (!bundle.AxisPicking()) {
      std::fprintf(stderr, "Could not start axis picking for assembly %d: %s\n",
                   target, bundle.PickMessage().c_str());
      mjv_freeScene(&checked_scene); mj_deleteData(checked_data); mj_deleteModel(checked);
      return 1;
    }
    for (int elevation : elevations) {
      for (int azimuth : azimuths) {
        checked_camera.azimuth = azimuth;
        checked_camera.elevation = elevation;
        mjv_updateScene(checked, checked_data, &checked_options, nullptr, &checked_camera,
                        mjCAT_ALL, &checked_scene);
        for (int y = 2; y < 62 && !fit; ++y) for (int x = 2; x < 62; ++x) {
          mjtNum point[3];
          const int part = SelectPreviewPartForHover(checked, checked_data,
            &checked_options, 1.3, x/64.0, y/64.0, checked_scene, bundle, point);
          ++rays;
          if (part < 0) continue;
          bundle.UpdateHover(part, {point[0], point[1], point[2]});
          if (bundle.HoveredLoopPoints() >= 6) {
            found_part = part;
            highlight_segments = int(bundle.HoveredLoopPoints());
            const int first_highlight_geom = checked_scene.ngeom;
            mjtNum camera_head[3], camera_forward[3], camera_up[3];
            mjv_cameraInModel(camera_head, camera_forward, camera_up, &checked_scene);
            bundle.AddSetupAxisMarkers(checked_scene,
              {camera_head[0], camera_head[1], camera_head[2]});
            for (int i = 0; i < highlight_segments; ++i) {
              const int index = first_highlight_geom + i;
              if (index >= checked_scene.ngeom || checked_scene.geomorder[index] != index ||
                  checked_scene.geoms[index].type != mjGEOM_LINE ||
                  checked_scene.geoms[index].category != mjCAT_DECOR ||
                  checked_scene.geoms[index].objtype != mjOBJ_UNKNOWN) {
                highlight_render_ready = false;
                break;
              }
            }
            fit = bundle.SelectHoveredHole();
            break;
          }
        }
        if (fit) break;
      }
      if (fit) break;
    }
    if (fit) {
      const auto& axis = bundle.Assembly(target);
      if (!highlight_render_ready) {
        std::fprintf(stderr, "Detected a rim, but its hover line geoms were not registered for rendering.\n");
        mjv_freeScene(&checked_scene); mj_deleteData(checked_data); mj_deleteModel(checked);
        return 1;
      }
      std::printf("Axis target '%s': rim hit reference part %d after %d CAD-only preview rays; "
                  "highlight loop %d line segments registered; click-to-axis fit passed; center %.5f %.5f %.5f m\n",
                  axis.name.c_str(), found_part, rays, highlight_segments,
                  axis.pivot[0], axis.pivot[1], axis.pivot[2]);
    } else {
      std::fprintf(stderr, "No complete rim hit on a visible CAD part in %d MuJoCo preview rays. Last status: %s\n",
                   rays, bundle.PickMessage().c_str());
    }
    mjv_freeScene(&checked_scene); mj_deleteData(checked_data); mj_deleteModel(checked);
    return fit ? 0 : 1;
  }
  if (argc >= 2 && std::string(argv[1]) == "--test-arena-spawns") {
    const std::filesystem::path test_path = argc >= 3 ? argv[2] :
      stock_model_path();
    std::array<char, 4096> diagnostic{};
    mjModel* test_model = mj_loadXML(test_path.string().c_str(), nullptr,
                                      diagnostic.data(), int(diagnostic.size()));
    if (!test_model) {
      std::fprintf(stderr, "Could not load arena model: %s\n", diagnostic.data());
      return 1;
    }
    mjData* test_data = mj_makeData(test_model);
    if (!test_data) { mj_deleteModel(test_model); return 1; }
    const int floor = mj_name2id(test_model, mjOBJ_GEOM, "floor");
    const int floor_visual = mj_name2id(test_model, mjOBJ_GEOM, "floor_visual");
    const int floor_material = floor_visual >= 0 ? test_model->geom_matid[floor_visual] : -1;
    const int key = mj_name2id(test_model, mjOBJ_LIGHT, "arena_key_light");
    const int fill = mj_name2id(test_model, mjOBJ_LIGHT, "arena_fill_light");
    AppSettings test_lighting;
    apply_arena_lighting(test_model, test_lighting);
    bool valid = floor >= 0 && test_model->geom_type[floor] == mjGEOM_PLANE &&
      test_model->geom_rgba[4 * floor + 3] == 0.0f &&
      floor_visual >= 0 && test_model->geom_type[floor_visual] == mjGEOM_BOX &&
      test_model->geom_contype[floor_visual] == 0 &&
      test_model->geom_conaffinity[floor_visual] == 0 &&
      test_model->geom_rgba[4 * floor_visual + 3] >= 0.999f &&
      floor_material >= 0 && test_model->mat_rgba[4 * floor_material] >= 0.75f &&
      test_model->mat_rgba[4 * floor_material + 1] >= 0.75f &&
      test_model->mat_rgba[4 * floor_material + 2] >= 0.70f &&
      key >= 0 && fill >= 0 && !test_model->light_castshadow[key] &&
      !test_model->light_castshadow[fill];
    for (const char* name : {"north_wall_acrylic", "south_wall_acrylic",
                             "east_wall_acrylic", "west_wall_acrylic"}) {
      const int panel = mj_name2id(test_model, mjOBJ_GEOM, name);
      if (panel < 0 || test_model->geom_contype[panel] != 0 ||
          test_model->geom_conaffinity[panel] != 0) {
        valid = false;
        continue;
      }
      const int thickness_axis = (std::string(name).find("east") != std::string::npos ||
                                  std::string(name).find("west") != std::string::npos) ? 0 : 1;
      valid = valid && std::abs(test_model->geom_size[3 * panel + thickness_axis] - 0.004) < 1e-6;
    }
    for (const char* name : {"north_wall", "south_wall", "east_wall", "west_wall"}) {
      const int wall = mj_name2id(test_model, mjOBJ_GEOM, name);
      if (wall < 0) { valid = false; continue; }
      const bool x_axis = std::string(name).find("east") != std::string::npos ||
                          std::string(name).find("west") != std::string::npos;
      const int radial_axis = x_axis ? 0 : 1;
      const int tangent_axis = 1 - radial_axis;
      const double inner_face = std::abs(test_model->geom_pos[3 * wall + radial_axis]) -
        test_model->geom_size[3 * wall + radial_axis];
      const double outer_face = inner_face + 2.0 * test_model->geom_size[3 * wall + radial_axis];
      const int roof = mj_name2id(test_model, mjOBJ_GEOM, "roof");
      valid = valid && std::abs(inner_face - 1.0) < 1e-6 &&
        test_model->geom_rgba[4 * wall + 3] == 0.0f &&
        test_model->geom_size[3 * wall + tangent_axis] >= outer_face + 0.019 &&
        roof >= 0 && test_model->geom_size[3 * roof] >= outer_face + 0.019 &&
        test_model->geom_size[3 * roof + 1] >= outer_face + 0.019 &&
        test_model->geom_pos[3 * wall + 2] + test_model->geom_size[3 * wall + 2] >=
          test_model->geom_pos[3 * roof + 2] - test_model->geom_size[3 * roof + 2] + 0.099;
    }
    const int roots[2] = {mj_name2id(test_model, mjOBJ_BODY, "Player_Robot"),
                          mj_name2id(test_model, mjOBJ_BODY, "Target_Robot")};
    mjtNum center_quat[2][4]{};
    for (int i = 0; i < 2; ++i) {
      const int joint = free_joint_for_body(test_model, roots[i]);
      if (joint < 0) { valid = false; continue; }
      const int qpos = test_model->jnt_qposadr[joint];
      apply_arena_spawn_layout(test_model, test_data, 0, 2.0);
      std::copy(test_data->qpos + qpos + 3, test_data->qpos + qpos + 7, center_quat[i]);
    }
    apply_arena_spawn_layout(test_model, test_data, 0, 2.0);
    Vec3 player_center = robot_geometry_center(test_model, test_data, roots[0]);
    Vec3 target_center = robot_geometry_center(test_model, test_data, roots[1]);
    valid = valid && player_center[0] < 0.0 && target_center[0] > 0.0 &&
      std::abs(player_center[0] + target_center[0]) < 1e-4 &&
      std::abs(player_center[1]) < 1e-4 && std::abs(target_center[1]) < 1e-4;
    for (int i = 0; i < 2; ++i) {
      Vec3 low{}, high{};
      robot_geometry_bounds(test_model, test_data, roots[i], low, high);
      valid = valid && low[0] >= -0.9901 && high[0] <= 0.9901 &&
        low[1] >= -0.9901 && high[1] <= 0.9901;
    }
    apply_arena_spawn_layout(test_model, test_data, 1, 2.0);
    player_center = robot_geometry_center(test_model, test_data, roots[0]);
    target_center = robot_geometry_center(test_model, test_data, roots[1]);
    valid = valid && player_center[0] < 0.0 && player_center[1] < 0.0 &&
      target_center[0] > 0.0 && target_center[1] > 0.0 &&
      std::abs(player_center[0] - player_center[1]) < 1e-4 &&
      std::abs(target_center[0] - target_center[1]) < 1e-4 &&
      std::abs(player_center[0] + target_center[0]) < 1e-4 &&
      std::abs(player_center[1] + target_center[1]) < 1e-4;
    const Vec3 diagonal_player = player_center;
    const Vec3 diagonal_target = target_center;
    apply_arena_spawn_layout(test_model, test_data, 1, 2.0);  // Reset must retain selected layout.
    player_center = robot_geometry_center(test_model, test_data, roots[0]);
    target_center = robot_geometry_center(test_model, test_data, roots[1]);
    valid = valid && std::abs(player_center[0] - diagonal_player[0]) < 1e-4 &&
      std::abs(player_center[1] - diagonal_player[1]) < 1e-4 &&
      std::abs(target_center[0] - diagonal_target[0]) < 1e-4 &&
      std::abs(target_center[1] - diagonal_target[1]) < 1e-4;
    for (int i = 0; i < 2; ++i) {
      Vec3 low{}, high{};
      robot_geometry_bounds(test_model, test_data, roots[i], low, high);
      valid = valid && low[0] >= -0.9901 && high[0] <= 0.9901 &&
        low[1] >= -0.9901 && high[1] <= 0.9901;
    }
    for (int i = 0; i < 2; ++i) {
      const int joint = free_joint_for_body(test_model, roots[i]);
      if (joint < 0) { valid = false; continue; }
      const int qpos = test_model->jnt_qposadr[joint];
      mjtNum expected_delta[4] = {std::cos(pi / 8.0), 0.0, 0.0, std::sin(pi / 8.0)};
      mjtNum expected[4];
      mju_mulQuat(expected, expected_delta, center_quat[i]);
      double alignment = 0.0;
      for (int component = 0; component < 4; ++component)
        alignment += expected[component] * test_data->qpos[qpos + 3 + component];
      valid = valid && std::abs(std::abs(alignment) - 1.0) < 1e-6;
    }
    const bool initial_layout_valid = valid;
    constexpr double resized_arena_m = 3.5;
    apply_arena_size(test_model, resized_arena_m);
    apply_arena_spawn_layout(test_model, test_data, 0, resized_arena_m);
    valid = valid && std::abs(test_model->geom_size[3 * floor_visual] - resized_arena_m * 0.5) < 1e-6 &&
      std::abs(test_model->geom_size[3 * floor_visual + 1] - resized_arena_m * 0.5) < 1e-6 &&
      std::abs(test_model->geom_size[3 * floor_visual + 2] - 0.001) < 1e-6 &&
      std::abs(test_model->geom_pos[3 * floor_visual + 2] + 0.001) < 1e-6 &&
      std::abs(test_model->geom_rbound[floor_visual] -
               std::sqrt(2.0 * std::pow(resized_arena_m * 0.5, 2) + 0.001 * 0.001)) < 1e-6;
    for (const char* name : {"north_wall", "south_wall", "east_wall", "west_wall"}) {
      const int wall = mj_name2id(test_model, mjOBJ_GEOM, name);
      if (wall < 0) { valid = false; continue; }
      const bool x_axis = std::string(name).find("east") != std::string::npos ||
                          std::string(name).find("west") != std::string::npos;
      const int radial_axis = x_axis ? 0 : 1;
      const int tangent_axis = 1 - radial_axis;
      const double inner_face = std::abs(test_model->geom_pos[3 * wall + radial_axis]) -
        test_model->geom_size[3 * wall + radial_axis];
      const double outer_face = inner_face + 2.0 * test_model->geom_size[3 * wall + radial_axis];
      const int roof = mj_name2id(test_model, mjOBJ_GEOM, "roof");
      valid = valid && std::abs(inner_face - resized_arena_m * 0.5) < 1e-6 &&
        test_model->geom_size[3 * wall + tangent_axis] >= outer_face + 0.019 &&
        roof >= 0 && test_model->geom_size[3 * roof] >= outer_face + 0.019 &&
        test_model->geom_size[3 * roof + 1] >= outer_face + 0.019 &&
        test_model->geom_pos[3 * wall + 2] + test_model->geom_size[3 * wall + 2] >=
          test_model->geom_pos[3 * roof + 2] - test_model->geom_size[3 * roof + 2] + 0.099;
    }
    for (int i = 0; i < 2; ++i) {
      Vec3 low{}, high{};
      robot_geometry_bounds(test_model, test_data, roots[i], low, high);
      valid = valid && low[0] >= -resized_arena_m * 0.5 - 1e-4 &&
        high[0] <= resized_arena_m * 0.5 + 1e-4 &&
        low[1] >= -resized_arena_m * 0.5 - 1e-4 &&
        high[1] <= resized_arena_m * 0.5 + 1e-4;
    }
    const bool resized_layout_valid = valid;
    const int player_free = free_joint_for_body(test_model, roots[0]);
    if (player_free < 0) {
      valid = false;
    } else {
      const int player_qpos = test_model->jnt_qposadr[player_free];
      const int player_dof = test_model->jnt_dofadr[player_free];
      test_data->qpos[player_qpos] = 1.1;
      test_data->qpos[player_qpos + 1] = 0.0;
      test_data->qpos[player_qpos + 2] = 0.065;
      test_data->qvel[player_dof] = 50.0;
      mj_forward(test_model, test_data);
      double peak_x = test_data->qpos[player_qpos];
      for (int step = 0; step < 300; ++step) {
        mj_step(test_model, test_data);
        peak_x = std::max(peak_x, double(test_data->qpos[player_qpos]));
      }
      valid = valid && peak_x < resized_arena_m * 0.5 + 0.85;
    }
    if (valid)
      std::puts("Arena check passed: adjustable square arena in meters, matte laminate floor, calm lighting, 8 mm acrylic panels, hidden catch walls, safe center-line and diagonal spawns, and selected-layout reset.");
    else
      std::fprintf(stderr, "Arena check failed (initial layout: %s, resized layout: %s): arena dimensions, floor, lights, acrylic panels, wall boundary, safe spawns, or reset layout are incorrect.\n",
                   initial_layout_valid ? "passed" : "failed",
                   resized_layout_valid ? "passed" : "failed");
    mj_deleteData(test_data);
    mj_deleteModel(test_model);
    return valid ? 0 : 1;
  }
  if (argc >= 3 && std::string(argv[1]) == "--validate-bundle") {
    if (!bundle.Load(argv[2])) { std::fprintf(stderr, "%s\n", bundle.Error().c_str()); return 1; }
    std::printf("Assembly axis markers: %s\n",
                bundle.ShowAxisMarkers() ? "shown" : "hidden");
    std::filesystem::path preview_generated;
    if (!bundle.GeneratePreview(preview_generated)) { std::fprintf(stderr, "%s\n", bundle.Error().c_str()); return 1; }
    std::array<char, 8192> preview_diagnostic{};
    mjModel* preview_checked = mj_loadXML(preview_generated.string().c_str(), nullptr,
                                          preview_diagnostic.data(), preview_diagnostic.size());
    if (!preview_checked) {
      std::fprintf(stderr, "MuJoCo rejected CAD setup model: %s\n", preview_diagnostic.data());
      return 1;
    }
    mj_deleteModel(preview_checked);
    std::filesystem::path generated;
    if (!bundle.Generate(generated)) { std::fprintf(stderr, "%s\n", bundle.Error().c_str()); return 1; }
    std::array<char, 8192> diagnostic{};
    mjModel* checked = mj_loadXML(generated.string().c_str(), nullptr, diagnostic.data(), diagnostic.size());
    if (!checked) { std::fprintf(stderr, "MuJoCo rejected generated model: %s\n", diagnostic.data()); return 1; }
    std::vector<std::pair<int, double>> left_wheels, right_wheels;
    std::vector<int> weapon_drives;
    std::map<int, MotorSpec> motor_models;
    std::map<int, MotorSpec> weapon_motor_models;
    std::map<int, int> manual_velocity_actuators;
    double bot_battery_voltage = 0.0;
    bundle.BindControls(checked, left_wheels, right_wheels, weapon_drives,
                        motor_models, weapon_motor_models, manual_velocity_actuators,
                        bot_battery_voltage);
    std::printf("CAD wheel drive mapping: left=%d right=%d; forward signs L=[",
                int(left_wheels.size()), int(right_wheels.size()));
    for (size_t i = 0; i < left_wheels.size(); ++i)
      std::printf("%s%.0f", i ? "," : "", left_wheels[i].second);
    std::printf("] R=[");
    for (size_t i = 0; i < right_wheels.size(); ++i)
      std::printf("%s%.0f", i ? "," : "", right_wheels[i].second);
    std::printf("]\n");
    std::map<int, int> motor_group_members;
    for (const auto& [actuator_id, motor] : motor_models) {
      (void)actuator_id;
      ++motor_group_members[motor.drive_group];
    }
    if (!motor_group_members.empty()) {
      std::printf("CAD shared motor groups:");
      for (const auto& [group_id, member_count] : motor_group_members)
        std::printf(" %c=%d wheels", 'A' + group_id, member_count);
      std::printf("\n");
    }
    mjData* sample = mj_makeData(checked);
    if (!sample) { mj_deleteModel(checked); std::fprintf(stderr, "MuJoCo could not allocate model data.\n"); return 1; }
    const int root_body = mj_name2id(checked, mjOBJ_BODY, "Player_Robot");
    std::vector<int> fork_bodies;
    std::vector<std::pair<int, int>> fork_qpos;
    for (int assembly = 1; assembly < bundle.AssemblyCount(); ++assembly) {
      if (bundle.Assembly(assembly).kind != AssemblyKind::GravityFork) continue;
      const std::string body_name = "cad_assembly_" + std::to_string(assembly);
      const int body = mj_name2id(checked, mjOBJ_BODY, body_name.c_str());
      if (body >= 0) fork_bodies.push_back(body);
      const std::string joint_name = "cad_joint_" + std::to_string(assembly);
      const int joint = mj_name2id(checked, mjOBJ_JOINT, joint_name.c_str());
      if (joint >= 0) fork_qpos.emplace_back(joint, checked->jnt_qposadr[joint]);
    }
    mj_forward(checked, sample);
    int initial_fork_frame_contacts = 0;
    for (int contact = 0; contact < sample->ncon; ++contact) {
      const int body1 = checked->geom_bodyid[sample->contact[contact].geom[0]];
      const int body2 = checked->geom_bodyid[sample->contact[contact].geom[1]];
      const bool root_fork = (body1 == root_body &&
        std::find(fork_bodies.begin(), fork_bodies.end(), body2) != fork_bodies.end()) ||
        (body2 == root_body &&
        std::find(fork_bodies.begin(), fork_bodies.end(), body1) != fork_bodies.end());
      if (root_fork) ++initial_fork_frame_contacts;
    }
    std::vector<double> initial_fork_angles;
    for (const auto& [joint, qpos] : fork_qpos) initial_fork_angles.push_back(sample->qpos[qpos]);
    for (int step = 0; step < 2500; ++step) mj_step(checked, sample);
    double max_fork_rotation = 0.0;
    for (size_t i = 0; i < fork_qpos.size(); ++i)
      max_fork_rotation = std::max(max_fork_rotation,
        std::abs(sample->qpos[fork_qpos[i].second] - initial_fork_angles[i]));
    int explicit_fork_frame_pairs = 0;
    for (int pair = 0; pair < checked->npair; ++pair) {
      const int body1 = checked->geom_bodyid[checked->pair_geom1[pair]];
      const int body2 = checked->geom_bodyid[checked->pair_geom2[pair]];
      const bool root_fork = (body1 == root_body &&
        std::find(fork_bodies.begin(), fork_bodies.end(), body2) != fork_bodies.end()) ||
        (body2 == root_body &&
        std::find(fork_bodies.begin(), fork_bodies.end(), body1) != fork_bodies.end());
      if (root_fork) ++explicit_fork_frame_pairs;
    }
    std::printf("CAD gravity forks: %d; explicit fork-frame pairs: %d; initial fork-frame contacts: %d; max passive fork angle after 1.0 s: %.5f rad\n",
                int(fork_bodies.size()), explicit_fork_frame_pairs, initial_fork_frame_contacts,
                max_fork_rotation);
    bool finite = std::isfinite(sample->time);
    for (int i = 0; i < checked->nq; ++i) finite = finite && std::isfinite(sample->qpos[i]);
    mj_deleteData(sample);
    if (!finite) { mj_deleteModel(checked); std::fprintf(stderr, "MuJoCo state became nonfinite during the one-second step test.\n"); return 1; }
    int axis_test_part = -1;
    for (int part = 0; part < bundle.PartCount(); ++part) {
      if (bundle.PartAssembly(part) == 1) { axis_test_part = part; break; }
    }
    if (axis_test_part < 0) {
      std::printf("CAD axis-fit tests skipped: no body is assigned to rotating assembly 1.\n");
      mj_deleteModel(checked);
      return 0;
    }
    bundle.StartAxis(1);
    for (const auto& point : {std::array<double, 3>{0.12, 0.08, 0.09},
                              {0.10, 0.10, 0.09}, {0.08, 0.08, 0.09},
                              {0.10, 0.06, 0.09}})
      if (!bundle.AddAxisPoint(axis_test_part, point, false)) {
        mj_deleteModel(checked); std::fprintf(stderr, "Could not add axis test point.\n"); return 1;
      }
    if (!bundle.FinishAxis()) {
      mj_deleteModel(checked); std::fprintf(stderr, "Could not fit axis test circle.\n"); return 1;
    }
    const auto& fitted = bundle.Assembly(1);
    if (std::abs(fitted.pivot[0]-0.10) > 1e-6 ||
        std::abs(fitted.pivot[1]-0.08) > 1e-6 ||
        std::abs(fitted.pivot[2]-0.09) > 1e-6 ||
        fitted.axis[2] < 0.999) {
      mj_deleteModel(checked); std::fprintf(stderr, "Axis fit missed the test hole center or plane.\n"); return 1;
    }
    bundle.StartAxis(1);
    const double root2 = std::sqrt(0.5);
    for (double angle : {0.0, 0.8, 2.4, 4.3}) {
      const std::array<double, 3> point{0.03 + 0.02*std::cos(angle),
        -0.04 + 0.02*root2*std::sin(angle),
         0.07 - 0.02*root2*std::sin(angle)};
      if (!bundle.AddAxisPoint(axis_test_part, point, false)) {
        mj_deleteModel(checked); std::fprintf(stderr, "Could not add tilted axis test point.\n"); return 1;
      }
    }
    if (!bundle.FinishAxis()) {
      mj_deleteModel(checked); std::fprintf(stderr, "Could not fit tilted axis test circle.\n"); return 1;
    }
    const auto& tilted = bundle.Assembly(1);
    if (std::abs(tilted.pivot[0]-0.03) > 1e-5 ||
        std::abs(tilted.pivot[1]+0.04) > 1e-5 ||
        std::abs(tilted.pivot[2]-0.07) > 1e-5 ||
        tilted.axis[1] < 0.70 || tilted.axis[2] < 0.70) {
      mj_deleteModel(checked); std::fprintf(stderr, "Tilted hole axis fit failed.\n"); return 1;
    }
    bundle.StartAxis(1);
    if (!bundle.AddAxisPoint(axis_test_part, {0.10, 0.0, 0.0}) || bundle.AxisPointCount() != 1) {
      mj_deleteModel(checked); std::fprintf(stderr, "Could not snap an axis point to a mesh edge.\n"); return 1;
    }
    std::printf("Validated %s: %d bodies, %d geoms, %d actuators\n",
                generated.string().c_str(), checked->nbody, checked->ngeom, checked->nu);
    mj_deleteModel(checked);
    return 0;
  }
#if defined(COMBATSIM_USE_FILAMENT)
  const bool filament_render_smoke = argc >= 2 &&
    std::string(argv[1]) == "--test-filament-render";
  if (filament_render_smoke && argc < 4) {
    std::fprintf(stderr, "Usage: CombatSimStandalone --test-filament-render model.xml output.bmp\n");
    return 2;
  }
#else
  constexpr bool filament_render_smoke = false;
#endif
  const bool renderer_resume_test = argc >= 3 &&
    std::string(argv[1]) == "--test-renderer-resume";
  const bool renderer_resume = argc >= 3 &&
    (std::string(argv[1]) == "--renderer-resume" || renderer_resume_test);
  std::filesystem::path model_path;
  if (renderer_resume) {
    if (!bundle.Load(argv[2])) {
      std::fprintf(stderr, "Could not restore player CAD bundle: %s\n", bundle.Error().c_str());
      return 1;
    }
    if (argc >= 4 && argv[3][0] != '\0' && !opponent_bundle.Load(argv[3])) {
      std::fprintf(stderr, "Could not restore opponent CAD bundle: %s\n",
                   opponent_bundle.Error().c_str());
      return 1;
    }
    if (!bundle.Generate(model_path, opponent_bundle.HasBundle() ? &opponent_bundle : nullptr)) {
      std::fprintf(stderr, "Could not rebuild CAD model after renderer switch: %s\n",
                   bundle.Error().c_str());
      return 1;
    }
  } else if (filament_render_smoke) {
    model_path = argv[2];
  } else {
    model_path = argc > 1 ? argv[1] :
      stock_model_path();
  }
  std::array<char, 4096> load_error{};
  mjModel* model = mj_loadXML(model_path.string().c_str(), nullptr, load_error.data(), load_error.size());
  if (!model) {
    std::fprintf(stderr, "MuJoCo model load failed: %s\n", load_error.data());
    return 1;
  }
  if (renderer_resume_test) {
    const bool restored_player = mj_name2id(model, mjOBJ_BODY, "Player_Robot") > 0;
    const bool restored_opponent = mj_name2id(model, mjOBJ_BODY, "Target_Robot") > 0;
    const bool valid = restored_player && restored_opponent &&
      std::abs(model->opt.timestep - 0.0004) < 1e-12;
    if (valid)
      std::printf("Renderer restart restored player and opponent CAD bundles: %d bodies, %.4f s timestep.\n",
                  model->nbody, model->opt.timestep);
    else
      std::fprintf(stderr, "Renderer restart bundle restore did not produce the expected two-bot model.\n");
    mj_deleteModel(model);
    return valid ? 0 : 1;
  }
#if defined(COMBATSIM_USE_FILAMENT)
  apply_filament_material_defaults(model);
#endif
  Controls controls;
  bind_stock_actuators(model, controls);
  Controls opponent_controls;
  if (bundle.HasBundle()) {
    controls.left.clear(); controls.right.clear(); controls.weapon.clear();
    controls.filtered_wheel_speeds.clear();
    bundle.BindControls(model, controls.left, controls.right, controls.weapon,
      controls.motor_models, controls.weapon_motor_models,
      controls.manual_velocity_actuators, controls.battery_voltage);
  }
  if (opponent_bundle.HasBundle()) {
    opponent_controls.left.clear(); opponent_controls.right.clear();
    opponent_controls.weapon.clear(); opponent_controls.filtered_wheel_speeds.clear();
    opponent_bundle.BindControls(model, opponent_controls.left,
      opponent_controls.right, opponent_controls.weapon,
      opponent_controls.motor_models, opponent_controls.weapon_motor_models,
      opponent_controls.manual_velocity_actuators,
      opponent_controls.battery_voltage, true);
  }
  mjData* data = mj_makeData(model);
  if (!data || !SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMEPAD | SDL_INIT_JOYSTICK)) {
    std::fprintf(stderr, "Initialization failed: %s\n", SDL_GetError());
    if (data) mj_deleteData(data);
    mj_deleteModel(model);
    return 1;
  }
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
  // MuJoCo 3.3.7's mjr renderer needs the OpenGL compatibility profile.
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_COMPATIBILITY);
  SDL_Window* window = SDL_CreateWindow("CombatSim Standalone", 1280, 800,
    SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIGH_PIXEL_DENSITY);
  SDL_GLContext gl = window ? SDL_GL_CreateContext(window) : nullptr;
  if (!gl) {
    std::fprintf(stderr, "OpenGL window failed: %s\n", SDL_GetError());
    if (window) SDL_DestroyWindow(window);
    SDL_Quit(); mj_deleteData(data); mj_deleteModel(model);
    return 1;
  }
  SDL_GL_MakeCurrent(window, gl);
  SDL_GL_SetSwapInterval(1);
  SDL_Gamepad* pad = first_gamepad();
  InputMixer input_mixer;
  std::string active_mixer_profile_key = mixer_model_profile_key(bundle, model_path);
  std::string active_mixer_profile_name = mixer_model_profile_name(bundle, model_path);
  load_mixer_settings(input_mixer, active_mixer_profile_key);
  AppSettings app_settings;
  load_app_settings(app_settings);
  std::string graphics_status;
  if (app_settings.renderer != default_renderer_id) {
    const std::filesystem::path player_manifest = bundle.HasBundle() &&
        !bundle.IsMemoryLoaded() ? bundle.ManifestPath() : std::filesystem::path{};
    const std::filesystem::path opponent_manifest = opponent_bundle.HasBundle() &&
        !opponent_bundle.IsMemoryLoaded() ? opponent_bundle.ManifestPath() : std::filesystem::path{};
    if (launch_renderer_build(app_settings.renderer, model_path, graphics_status,
                              player_manifest, opponent_manifest)) {
      SDL_GL_DestroyContext(gl);
      SDL_DestroyWindow(window);
      SDL_Quit();
      mj_deleteData(data);
      mj_deleteModel(model);
      return 0;
    }
    app_settings.renderer = default_renderer_id;
    save_app_settings(app_settings);
  }
  char arena_preset_name[65]{};
  std::string arena_preset_status;
  apply_arena_size(model, app_settings.arena_size_m);
  apply_arena_spawn_layout(model, data, app_settings.spawn_layout,
                           app_settings.arena_size_m);
  apply_arena_lighting(model, app_settings);
  SDL_Joystick* joystick = nullptr;
  SDL_JoystickID joystick_id = 0;
  const std::string saved_guid = input_mixer.preferred_guid;
  refresh_joystick(joystick, joystick_id, input_mixer);
  if (saved_guid != input_mixer.preferred_guid)
    save_mixer_settings(input_mixer, active_mixer_profile_key);

  mjvCamera camera; mjv_defaultCamera(&camera);
  camera.distance = std::clamp(3.5 * double(app_settings.arena_size_m) / 2.0, 1.5, 15.0);
  camera.elevation = -55; camera.azimuth = 90;
  mjvCamera setup_camera; mjv_defaultCamera(&setup_camera);
  setup_camera.distance = 1.0; setup_camera.elevation = -45; setup_camera.azimuth = 45;
  mjvOption options; mjv_defaultOption(&options);
  mjvScene scene; mjv_defaultScene(&scene); mjv_makeScene(model, &scene, 2000);
  mjModel* setup_model = nullptr;
  mjData* setup_data = nullptr;
  mjvScene setup_scene; mjv_defaultScene(&setup_scene);
  mjrContext setup_context; mjr_defaultContext(&setup_context);
  model->vis.quality.offsamples = 0;
  mjrContext context; mjr_defaultContext(&context); make_render_context(model, &context);
#if defined(COMBATSIM_USE_FILAMENT)
  GLuint main_scene_texture = 0;
  glGenTextures(1, &main_scene_texture);
  int main_scene_width = 0, main_scene_height = 0;
  std::vector<unsigned char> main_scene_pixels;
#endif
  GLuint preview_texture = 0;
  glGenTextures(1, &preview_texture);
  int texture_width = 0, texture_height = 0;
  std::vector<unsigned char> preview_pixels;
#if defined(COMBATSIM_USE_FILAMENT)
  if (filament_render_smoke) {
    constexpr int test_width = 1024, test_height = 768;
    mjv_updateScene(model, data, &options, nullptr, &camera, mjCAT_ALL, &scene);
    std::vector<unsigned char> pixels(size_t(test_width) * test_height * 3);
    const mjrRect test_view{0, 0, test_width, test_height};
    mjr_setBuffer(mjFB_OFFSCREEN, &context);
    mjr_render(test_view, &scene, &context);
    mjr_readPixels(pixels.data(), nullptr, test_view, &context);
    unsigned char low = 255, high = 0;
    for (unsigned char value : pixels) {
      low = std::min(low, value);
      high = std::max(high, value);
    }
    const bool image_written = write_rgb_bitmap(argv[3], test_width, test_height, pixels);
    mjrContext preview_context; mjr_defaultContext(&preview_context);
    mjvScene preview_scene; mjv_defaultScene(&preview_scene);
    mjv_makeScene(model, &preview_scene, 512);
    make_render_context(model, &preview_context);
    constexpr int preview_width = 320, preview_height = 240;
    std::vector<unsigned char> preview_pixels(size_t(preview_width) * preview_height * 3);
    mjv_updateScene(model, data, &options, nullptr, &setup_camera, mjCAT_ALL,
                    &preview_scene);
    mjr_setBuffer(mjFB_OFFSCREEN, &preview_context);
    mjr_render({0, 0, preview_width, preview_height}, &preview_scene, &preview_context);
    mjr_readPixels(preview_pixels.data(), nullptr,
                   {0, 0, preview_width, preview_height}, &preview_context);
    const auto preview_range = std::minmax_element(preview_pixels.begin(), preview_pixels.end());
    const bool preview_rendered = preview_range.first != preview_pixels.end() &&
      int(*preview_range.second) - int(*preview_range.first) >= 8;
    const bool rendered = image_written && (high - low >= 8) && preview_rendered;
    if (rendered)
      std::printf("Filament render smoke passed: %s (%dx%d, pixel range %u-%u)\n",
                  argv[3], test_width, test_height, unsigned(low), unsigned(high));
    else
      std::fprintf(stderr, "Filament render smoke failed (image write=%d, drive range %u-%u, preview=%d).\n",
                   image_written ? 1 : 0, unsigned(low), unsigned(high),
                   preview_rendered ? 1 : 0);
    mjr_freeContext(&preview_context); mjv_freeScene(&preview_scene);
    mjr_freeContext(&context); mjv_freeScene(&scene);
    glDeleteTextures(1, &preview_texture);
    glDeleteTextures(1, &main_scene_texture);
    if (pad) SDL_CloseGamepad(pad);
    if (joystick) SDL_CloseJoystick(joystick);
    SDL_GL_DestroyContext(gl); SDL_DestroyWindow(window); SDL_Quit();
    mj_deleteData(data); mj_deleteModel(model);
    return rendered ? 0 : 1;
  }
#endif
  IMGUI_CHECKVERSION(); ImGui::CreateContext();
  ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  ImGuiStyle& ui = ImGui::GetStyle();
  ImGui::StyleColorsDark(&ui);
  ui.WindowRounding = 8.0f; ui.ChildRounding = 7.0f; ui.FrameRounding = 5.0f;
  ui.PopupRounding = 7.0f; ui.ScrollbarRounding = 7.0f; ui.GrabRounding = 5.0f;
  ui.WindowPadding = ImVec2(18, 16); ui.FramePadding = ImVec2(11, 8);
  ui.ItemSpacing = ImVec2(10, 10); ui.ItemInnerSpacing = ImVec2(8, 6);
  ui.WindowBorderSize = 0.0f; ui.ChildBorderSize = 1.0f; ui.FrameBorderSize = 0.0f;
  ImVec4* colors = ui.Colors;
  colors[ImGuiCol_Text] = ui_ink;
  colors[ImGuiCol_TextDisabled] = ImVec4(0.58f, 0.63f, 0.70f, 1.0f);
  colors[ImGuiCol_WindowBg] = ImVec4(0.055f, 0.062f, 0.075f, 1.0f);
  colors[ImGuiCol_ChildBg] = ImVec4(0.075f, 0.083f, 0.098f, 1.0f);
  colors[ImGuiCol_PopupBg] = ImVec4(0.075f, 0.083f, 0.098f, 0.99f);
  colors[ImGuiCol_Border] = ImVec4(0.20f, 0.22f, 0.25f, 0.82f);
  colors[ImGuiCol_BorderShadow] = ImVec4(0.015f, 0.020f, 0.030f, 0.22f);
  colors[ImGuiCol_FrameBg] = ImVec4(0.13f, 0.145f, 0.17f, 1.0f);
  colors[ImGuiCol_FrameBgHovered] = ImVec4(0.18f, 0.195f, 0.22f, 1.0f);
  colors[ImGuiCol_FrameBgActive] = ImVec4(0.22f, 0.23f, 0.25f, 1.0f);
  colors[ImGuiCol_Button] = ImVec4(0.14f, 0.15f, 0.17f, 1.0f);
  colors[ImGuiCol_ButtonHovered] = ImVec4(0.22f, 0.23f, 0.25f, 1.0f);
  colors[ImGuiCol_ButtonActive] = ImVec4(0.48f, 0.22f, 0.12f, 1.0f);
  colors[ImGuiCol_Header] = ImVec4(0.15f, 0.16f, 0.18f, 1.0f);
  colors[ImGuiCol_HeaderHovered] = ImVec4(0.20f, 0.21f, 0.23f, 1.0f);
  colors[ImGuiCol_HeaderActive] = ImVec4(0.33f, 0.19f, 0.13f, 1.0f);
  colors[ImGuiCol_CheckMark] = ui_accent;
  colors[ImGuiCol_SliderGrab] = ImVec4(0.83f, 0.36f, 0.17f, 1.0f);
  colors[ImGuiCol_SliderGrabActive] = ui_accent;
  colors[ImGuiCol_Separator] = ImVec4(0.22f, 0.23f, 0.25f, 0.82f);
  colors[ImGuiCol_SeparatorHovered] = ImVec4(0.83f, 0.36f, 0.17f, 1.0f);
  colors[ImGuiCol_SeparatorActive] = ui_accent;
  colors[ImGuiCol_ScrollbarBg] = ImVec4(0.055f, 0.062f, 0.075f, 0.72f);
  colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.25f, 0.26f, 0.28f, 1.0f);
  colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.38f, 0.38f, 0.39f, 1.0f);
  colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.68f, 0.32f, 0.17f, 1.0f);
  colors[ImGuiCol_TableHeaderBg] = ImVec4(0.17f, 0.21f, 0.27f, 1.0f);
  colors[ImGuiCol_TableBorderStrong] = ImVec4(0.29f, 0.34f, 0.41f, 1.0f);
  colors[ImGuiCol_TableBorderLight] = ImVec4(0.22f, 0.27f, 0.34f, 1.0f);
  colors[ImGuiCol_TableRowBg] = ImVec4(0.10f, 0.12f, 0.15f, 1.0f);
  colors[ImGuiCol_TableRowBgAlt] = ImVec4(0.13f, 0.155f, 0.195f, 1.0f);
  colors[ImGuiCol_Tab] = ImVec4(0.13f, 0.16f, 0.21f, 1.0f);
  colors[ImGuiCol_TabHovered] = ImVec4(0.22f, 0.32f, 0.39f, 1.0f);
  colors[ImGuiCol_TabSelected] = ImVec4(0.18f, 0.24f, 0.31f, 1.0f);
  colors[ImGuiCol_TabDimmed] = ImVec4(0.10f, 0.125f, 0.16f, 1.0f);
  colors[ImGuiCol_TabDimmedSelected] = ImVec4(0.15f, 0.19f, 0.24f, 1.0f);
  colors[ImGuiCol_PlotHistogram] = ui_teal;
  colors[ImGuiCol_PlotHistogramHovered] = ImVec4(0.22f, 0.69f, 0.70f, 1.0f);
  ImGui_ImplSDL3_InitForOpenGL(window, gl);
  ImGui_ImplOpenGL3_Init("#version 330");

  enum class AppScreen { Home, Drive, Configure, Settings };
  enum class SettingsSection { Arena, Graphics };
  enum class GizmoMode { Move, Rotate };
  AppScreen screen = AppScreen::Home;
  SettingsSection settings_section = SettingsSection::Arena;
  int config_bot = 0;  // 0 = player, 1 = opponent.
  bool opponent_ai_enabled = true;
  float opponent_ai_aggression_percent = 70.0f;
  bool running = true, paused = true;
  bool placement_mode = false;
  GizmoMode gizmo_mode = GizmoMode::Move;
  bool gizmo_drag_active = false;
  int gizmo_drag_axis = -1;
  double gizmo_length = 0.35;
  double ring_previous_angle = 0.0;
  Vec3 rotation_pivot{};
  Vec3 rotation_local_pivot{};
  bool mixer_popup_open = false;
  bool lan_popup_open = false;
  std::string runtime_error;
  char lan_address[256] = "127.0.0.1";
  char lan_port_text[16] = "29811";
  char lan_passphrase[65]{};
  std::string generated_passphrase;
  LanSession::GeneratePassphrase(generated_passphrase);
  std::snprintf(lan_passphrase, sizeof(lan_passphrase), "%s", generated_passphrase.c_str());
  std::filesystem::path previous_player_manifest;
  std::filesystem::path previous_opponent_manifest;
  std::vector<unsigned char> local_client_package;
  bool lan_bundle_swapped = false;
  bool lan_model_installed = false;
  bool lan_match_started = false;
  double accumulator = 0.0;
  auto last = std::chrono::steady_clock::now();
  auto wipe_package = [](std::vector<unsigned char>& bytes) {
    volatile unsigned char* data = bytes.empty() ? nullptr : bytes.data();
    for (size_t i = 0; i < bytes.size(); ++i) data[i] = 0;
    bytes.clear();
    bytes.shrink_to_fit();
  };
  auto activate_mixer_profile_for_current_model = [&] {
    const std::string next_key = mixer_model_profile_key(bundle, model_path);
    active_mixer_profile_name = mixer_model_profile_name(bundle, model_path);
    if (next_key == active_mixer_profile_key) return;

    const std::string selected_device = input_mixer.preferred_guid;
    if (!save_mixer_settings(input_mixer, active_mixer_profile_key))
      runtime_error = "Could not save the joystick setup for the previous model.";
    active_mixer_profile_key = next_key;
    load_mixer_settings(input_mixer, active_mixer_profile_key);
    // The selected physical controller is a computer preference; channel maps
    // and output mixes belong to the model profile.
    input_mixer.preferred_guid = selected_device;
    refresh_joystick(joystick, joystick_id, input_mixer);
    if (!save_mixer_settings(input_mixer, active_mixer_profile_key))
      runtime_error = "Could not save this model's joystick setup.";
  };
  auto install_model = [&](mjModel* next_model) {
    mjData* next_data = next_model ? mj_makeData(next_model) : nullptr;
    if (!next_data) {
      runtime_error = next_model ? "MuJoCo could not allocate model data." :
        "MuJoCo could not compile the generated model.";
      if (next_model) mj_deleteModel(next_model);
      return false;
    }
#if defined(COMBATSIM_USE_FILAMENT)
    apply_filament_material_defaults(next_model);
#endif
    apply_arena_size(next_model, app_settings.arena_size_m);
    apply_arena_spawn_layout(next_model, next_data, app_settings.spawn_layout,
                             app_settings.arena_size_m);
    apply_arena_lighting(next_model, app_settings);
    mjr_freeContext(&context); mjv_freeScene(&scene);
    mj_deleteData(data); mj_deleteModel(model);
    model = next_model; data = next_data;
    gizmo_drag_active = false; gizmo_drag_axis = -1;
    model->vis.quality.offsamples = 0;
    mjv_makeScene(model, &scene, 2000); make_render_context(model, &context);
    controls.left.clear(); controls.right.clear(); controls.weapon.clear();
    controls.filtered_wheel_speeds.clear();
    bundle.BindControls(model, controls.left, controls.right, controls.weapon,
                        controls.motor_models, controls.weapon_motor_models,
                        controls.manual_velocity_actuators, controls.battery_voltage);
    controls.weapon_sign = 1.0; controls.weapon_command = 0.0;
    opponent_controls.left.clear(); opponent_controls.right.clear();
    opponent_controls.weapon.clear(); opponent_controls.filtered_wheel_speeds.clear();
    opponent_bundle.BindControls(model, opponent_controls.left, opponent_controls.right,
      opponent_controls.weapon, opponent_controls.motor_models,
      opponent_controls.weapon_motor_models, opponent_controls.manual_velocity_actuators,
      opponent_controls.battery_voltage, true);
    opponent_controls.weapon_sign = 1.0; opponent_controls.weapon_command = 0.0;
    accumulator = 0.0; paused = true; runtime_error.clear();
    return true;
  };
  auto rebuild_configured_model = [&](bool in_memory) {
    if (in_memory) {
      std::string xml;
      std::vector<MemoryBundleAsset> assets;
      if (!bundle.GenerateInMemory(xml, assets,
            opponent_bundle.HasBundle() ? &opponent_bundle : nullptr)) {
        runtime_error = bundle.Error();
        return false;
      }
      std::array<char, 8192> diagnostic{};
      mjModel* next_model = LoadModelFromMemory(xml, assets, diagnostic.data(),
                                                 int(diagnostic.size()));
      for (auto& asset : assets) {
        volatile unsigned char* bytes = asset.bytes.empty() ? nullptr : asset.bytes.data();
        for (size_t i = 0; i < asset.bytes.size(); ++i) bytes[i] = 0;
      }
      if (!next_model) {
        if (diagnostic[0]) runtime_error = diagnostic.data();
        return false;
      }
      if (!install_model(next_model)) return false;
      activate_mixer_profile_for_current_model();
      return true;
    }
    std::filesystem::path generated;
    if (!bundle.Generate(generated, opponent_bundle.HasBundle() ? &opponent_bundle : nullptr)) {
      runtime_error = bundle.Error();
      return false;
    }
    std::array<char, 8192> diagnostic{};
    mjModel* next_model = mj_loadXML(generated.string().c_str(), nullptr,
                                      diagnostic.data(), int(diagnostic.size()));
    if (!next_model) {
      if (diagnostic[0]) runtime_error = diagnostic.data();
      return false;
    }
    if (!install_model(next_model)) return false;
    activate_mixer_profile_for_current_model();
    return true;
  };
  auto end_lan_session = [&] {
    lan.Stop();
    if (lan_bundle_swapped) {
      bool player_restored = false;
      if (!previous_player_manifest.empty())
        player_restored = bundle.Load(previous_player_manifest);
      else
        bundle.Clear();
      if (!previous_opponent_manifest.empty())
        opponent_bundle.Load(previous_opponent_manifest);
      else
        opponent_bundle.Clear();
      if (player_restored) rebuild_configured_model(false);
      else runtime_error = "LAN ended, but the original player bundle could not be restored.";
    }
    wipe_package(local_client_package);
    previous_player_manifest.clear();
    previous_opponent_manifest.clear();
    lan_bundle_swapped = false;
    lan_model_installed = false;
    lan_match_started = false;
    opponent_ai_enabled = true;
    placement_mode = false;
    screen = AppScreen::Home;
    paused = true;
  };
  // Each frame samples input, applies controls, advances MuJoCo at its fixed
  // timestep, refreshes render state, and draws the current navigation page.
  while (running) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      ImGui_ImplSDL3_ProcessEvent(&event);
      if (event.type == SDL_EVENT_QUIT) running = false;
      if (event.type == SDL_EVENT_GAMEPAD_ADDED && !pad) pad = first_gamepad();
      if (event.type == SDL_EVENT_GAMEPAD_REMOVED && pad &&
          event.gdevice.which == SDL_GetGamepadID(pad)) {
        SDL_CloseGamepad(pad); pad = first_gamepad();
      }
      if (event.type == SDL_EVENT_JOYSTICK_ADDED || event.type == SDL_EVENT_JOYSTICK_REMOVED) {
        if (event.type == SDL_EVENT_JOYSTICK_REMOVED && joystick &&
            event.jdevice.which == joystick_id) {
          SDL_CloseJoystick(joystick);
          joystick = nullptr;
          joystick_id = 0;
        }
        const std::string previous_guid = input_mixer.preferred_guid;
        refresh_joystick(joystick, joystick_id, input_mixer);
        if (previous_guid != input_mixer.preferred_guid)
          save_mixer_settings(input_mixer, active_mixer_profile_key);
      }

      if (event.type == SDL_EVENT_MOUSE_WHEEL && !ImGui::GetIO().WantCaptureMouse)
        camera.distance = std::clamp(camera.distance * std::pow(0.9, event.wheel.y), 0.5, 15.0);
      if (event.type == SDL_EVENT_KEY_DOWN && event.key.key == SDLK_R &&
          event.key.repeat == 0 && screen == AppScreen::Drive && placement_mode &&
          !ImGui::GetIO().WantCaptureKeyboard) {
        gizmo_mode = gizmo_mode == GizmoMode::Move ? GizmoMode::Rotate : GizmoMode::Move;
        gizmo_drag_active = false;
        gizmo_drag_axis = -1;
      }
      if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN && event.button.button == SDL_BUTTON_LEFT &&
          screen == AppScreen::Drive && placement_mode && !ImGui::GetIO().WantCaptureMouse) {
        gizmo_drag_active = false;
        gizmo_drag_axis = -1;
        const int player_root = mj_name2id(model, mjOBJ_BODY, "Player_Robot");
        int window_width = 0, window_height = 0;
        SDL_GetWindowSize(window, &window_width, &window_height);
        if (player_root > 0 && window_width > 0 && window_height > 0) {
          const Vec3 center{data->subtree_com[3*player_root],
                            data->subtree_com[3*player_root + 1],
                            data->subtree_com[3*player_root + 2]};
          gizmo_length = std::clamp(camera.distance * 0.11, 0.18, 0.65);
          const ScreenPoint mouse{event.button.x, event.button.y};
          GizmoHit hit = gizmo_mode == GizmoMode::Move
            ? pick_translation_axis(model, scene, window_width, window_height,
                                    center, gizmo_length, mouse)
            : pick_rotation_ring(model, scene, window_width, window_height,
                                 center, gizmo_length * 0.82, mouse);
          const double pick_tolerance = gizmo_mode == GizmoMode::Move ? 16.0 : 18.0;
          if (hit.axis >= 0 && hit.distance <= pick_tolerance) {
            gizmo_drag_axis = hit.axis;
            gizmo_drag_active = true;
            paused = true;
            const int joint = free_joint_for_body(model, player_root);
            if (joint >= 0) {
              const int dof = model->jnt_dofadr[joint];
              for (int i = 0; i < 6; ++i) data->qvel[dof + i] = 0.0;
            }
            if (gizmo_mode == GizmoMode::Rotate) {
              rotation_pivot = center;
              mjtNum offset[3];
              mju_sub3(offset, rotation_pivot.data(), data->xpos + 3*player_root);
              mju_mulMatTVec(rotation_local_pivot.data(), data->xmat + 9*player_root,
                             offset, 3, 3);
            }
            ring_previous_angle = hit.angle;
          }
        }
      }
      if (event.type == SDL_EVENT_MOUSE_BUTTON_UP && event.button.button == SDL_BUTTON_LEFT) {
        gizmo_drag_active = false;
        gizmo_drag_axis = -1;
      }
      if (event.type == SDL_EVENT_MOUSE_MOTION && !ImGui::GetIO().WantCaptureMouse) {
        int window_width = 0, window_height = 0;
        SDL_GetWindowSize(window, &window_width, &window_height);
        if (window_width > 0 && window_height > 0 && screen == AppScreen::Drive &&
            placement_mode && gizmo_drag_active && gizmo_drag_axis >= 0 &&
            (event.motion.state & SDL_BUTTON_LMASK)) {
          const int player_root = mj_name2id(model, mjOBJ_BODY, "Player_Robot");
          if (player_root > 0 && gizmo_mode == GizmoMode::Move) {
            const Vec3 center{data->subtree_com[3*player_root],
                              data->subtree_com[3*player_root + 1],
                              data->subtree_com[3*player_root + 2]};
            const Vec3 end{center[0] + gizmo_length*world_axes[gizmo_drag_axis][0],
                           center[1] + gizmo_length*world_axes[gizmo_drag_axis][1],
                           center[2] + gizmo_length*world_axes[gizmo_drag_axis][2]};
            ScreenPoint start_screen, end_screen;
            if (project_to_screen(model, scene, window_width, window_height,
                                  center, start_screen) &&
                project_to_screen(model, scene, window_width, window_height,
                                  end, end_screen)) {
              const double dx = end_screen[0] - start_screen[0];
              const double dy = end_screen[1] - start_screen[1];
              const double length2 = dx*dx + dy*dy;
              if (length2 > 25.0) {
                const double distance = gizmo_length *
                  (event.motion.xrel*dx + event.motion.yrel*dy) / length2;
                translate_free_body_axis(model, data, player_root, gizmo_drag_axis, distance);
              }
            }
          } else if (player_root > 0 && gizmo_mode == GizmoMode::Rotate) {
            const GizmoHit hit = pick_rotation_ring(model, scene, window_width, window_height,
                rotation_pivot, gizmo_length*0.82,
                ScreenPoint{event.motion.x, event.motion.y}, gizmo_drag_axis);
            double delta = hit.angle - ring_previous_angle;
            while (delta > pi) delta -= 2.0*pi;
            while (delta < -pi) delta += 2.0*pi;
            if (hit.axis == gizmo_drag_axis && std::abs(delta) > 1e-8)
              rotate_free_body_axis(model, data, player_root, gizmo_drag_axis, delta,
                                    rotation_pivot, rotation_local_pivot);
            ring_previous_angle = hit.angle;
          }
        } else if (window_height > 0 && screen == AppScreen::Drive &&
                   (event.motion.state & SDL_BUTTON_RMASK)) {
          mjv_moveCamera(model, mjMOUSE_MOVE_H,
              event.motion.xrel / static_cast<mjtNum>(window_height),
              -event.motion.yrel / static_cast<mjtNum>(window_height), &scene, &camera);
        } else if ((!placement_mode || screen == AppScreen::Drive) &&
                   (event.motion.state & SDL_BUTTON_LMASK)) {
          camera.azimuth += event.motion.xrel * 0.4;
          camera.elevation = std::clamp(camera.elevation - event.motion.yrel * 0.4, -89.0, 89.0);
        }
      }

    }
    if (lan.GetState() == LanSession::State::PackageReady && !lan_model_installed) {
      std::vector<unsigned char> remote_package;
      bool received = lan.TakeRemotePackage(remote_package);
      if (received && lan.GetRole() == LanSession::Role::Host) {
        if (!opponent_bundle.LoadSessionPackage(remote_package)) {
          runtime_error = "Opponent bundle rejected: " + opponent_bundle.Error();
          received = false;
        } else {
          lan_bundle_swapped = true;
        }
      } else if (received && lan.GetRole() == LanSession::Role::Client) {
        if (!bundle.LoadSessionPackage(remote_package)) {
          runtime_error = "Host bundle rejected: " + bundle.Error();
          received = false;
        } else {
          lan_bundle_swapped = true;
        }
        if (received && !opponent_bundle.LoadSessionPackage(local_client_package)) {
          runtime_error = "Your bot could not be loaded into the LAN model: " +
            opponent_bundle.Error();
          received = false;
        }
        if (received) {
          wipe_package(local_client_package);
        }
      }
      wipe_package(remote_package);
      if (received && rebuild_configured_model(true)) {
        lan_model_installed = true;
        bundle.DiscardSessionMeshBuffers();
        opponent_bundle.DiscardSessionMeshBuffers();
        lan.MarkReady();
      } else if (received) {
        lan.Stop();
      } else {
        lan.Stop();
      }
    }
    if (lan.GetState() == LanSession::State::Connected && !lan_match_started) {
      lan_match_started = true;
      screen = AppScreen::Drive;
      paused = lan.GetRole() == LanSession::Role::Client;
      opponent_ai_enabled = false;
      runtime_error.clear();
    }
    const bool lan_connected = lan.GetState() == LanSession::State::Connected;
    const bool lan_client = lan_connected && lan.GetRole() == LanSession::Role::Client;
    const bool lan_host = lan_connected && lan.GetRole() == LanSession::Role::Host;
    const bool local_bot_is_guest = lan_bundle_swapped &&
      lan.GetRole() == LanSession::Role::Client;
    const Controls& telemetry_controls = local_bot_is_guest ? opponent_controls : controls;
    const FusionBundle& telemetry_bundle = local_bot_is_guest ? opponent_bundle : bundle;
    const bool window_has_input_focus = SDL_GetKeyboardFocus() == window;
    if (lan_client) {
      const bool* keys = SDL_GetKeyboardState(nullptr);
      const bool keyboard_enabled = window_has_input_focus &&
        !ImGui::GetIO().WantCaptureKeyboard;
      KeyboardControls command = keyboard_enabled
        ? read_keyboard_controls(keys) : KeyboardControls{};
      if (pad && window_has_input_focus) {
        const float gamepad_throttle = -axis_with_deadzone(
          pad, SDL_GAMEPAD_AXIS_LEFTY, controls.deadzone);
        const float gamepad_turn = -axis_with_deadzone(
          pad, SDL_GAMEPAD_AXIS_LEFTX, controls.deadzone);
        if (std::abs(gamepad_throttle) > 1e-4f) command.throttle = gamepad_throttle;
        if (std::abs(gamepad_turn) > 1e-4f) command.turn = gamepad_turn;
        command.weapon = command.weapon ||
          SDL_GetGamepadButton(pad, SDL_GAMEPAD_BUTTON_LEFT_SHOULDER) ||
          SDL_GetGamepadButton(pad, SDL_GAMEPAD_BUTTON_SOUTH);
      }
      lan.SendInput({command.throttle, command.turn, command.weapon ? 1.0f : 0.0f});
      LanSession::Snapshot snapshot;
      if (lan.TakeLatestSnapshot(snapshot)) {
        if (snapshot.qpos.size() != size_t(model->nq) ||
            snapshot.qvel.size() != size_t(model->nv)) {
          runtime_error = "LAN model state does not match the host model.";
          paused = true;
          lan.Stop();
        } else {
          std::copy(snapshot.qpos.begin(), snapshot.qpos.end(), data->qpos);
          std::copy(snapshot.qvel.begin(), snapshot.qvel.end(), data->qvel);
          data->time = snapshot.time;
          mj_forward(model, data);
        }
      }
    }
    if (lan.GetState() == LanSession::State::Failed) {
      if (lan_match_started) paused = true;
      if (!lan.Error().empty()) runtime_error = "LAN session: " + lan.Error();
    }

    const auto now = std::chrono::steady_clock::now();
    accumulator += std::min(0.1, std::chrono::duration<double>(now - last).count());
    last = now;
    if (paused || screen != AppScreen::Drive) accumulator = 0.0;
    int steps = 0;
    while (!lan_client && !paused && screen == AppScreen::Drive &&
           accumulator >= model->opt.timestep && steps++ < 250) {
      const bool keyboard_enabled = window_has_input_focus &&
        !ImGui::GetIO().WantCaptureKeyboard;
      const bool* keys = SDL_GetKeyboardState(nullptr);
      const KeyboardControls player_keyboard = keyboard_enabled
        ? read_keyboard_controls(keys) : KeyboardControls{};
      const KeyboardControls opponent_keyboard = keyboard_enabled
        ? read_keyboard_controls(keys, true) : KeyboardControls{};
      drive(model, data, controls,
            window_has_input_focus ? pad : nullptr,
            window_has_input_focus ? joystick : nullptr,
            input_mixer, player_keyboard, true);
      const bool opponent_ai_ready = opponent_ai_delay_elapsed(data->time);
      const LanSession::Input remote = lan_host ? lan.LatestRemoteInput() :
        LanSession::Input{};
      const KeyboardControls opponent_command = lan_host
        ? KeyboardControls{remote.throttle, remote.turn, remote.weapon > 0.5f}
        : opponent_ai_enabled
          ? (opponent_ai_ready
              ? opponent_ram_ai(model, data, opponent_controls,
                                opponent_ai_aggression_percent)
              : KeyboardControls{})
          : opponent_keyboard;
      drive(model, data, opponent_controls, nullptr, nullptr, input_mixer,
            opponent_command, false);
      mj_step(model, data);
      accumulator -= model->opt.timestep;
    }
    if (steps >= 250) accumulator = 0.0;
    if (lan_host)
      lan.SendSnapshot(data->time, data->qpos, model->nq, data->qvel, model->nv);

    int width = 0, height = 0;
    SDL_GetWindowSizeInPixels(window, &width, &height);
    mjrRect viewport{0, 0, width, height};
#if !defined(COMBATSIM_USE_FILAMENT)
    mjr_setBuffer(mjFB_WINDOW, &context);
#endif
    apply_arena_lighting(model, app_settings);
    mjv_updateScene(model, data, &options, nullptr, &camera, mjCAT_ALL, &scene);
    if (local_bot_is_guest) {
      if (opponent_bundle.ShowAxisMarkers())
        opponent_bundle.AddAxisMarkers(model, data, scene, true);
    } else if (bundle.ShowAxisMarkers()) {
      bundle.AddAxisMarkers(model, data, scene);
    }
    if (screen == AppScreen::Drive && placement_mode) {
      const int player_root = mj_name2id(model, mjOBJ_BODY, "Player_Robot");
      gizmo_length = std::clamp(camera.distance * 0.11, 0.18, 0.65);
      add_transform_gizmo(model, data, player_root, gizmo_mode == GizmoMode::Rotate,
                          gizmo_drag_axis, gizmo_length, scene);
    }
#if defined(COMBATSIM_USE_FILAMENT)
    if (width != main_scene_width || height != main_scene_height) {
      if (main_scene_width > 0) mjr_setBuffer(mjFB_WINDOW, &context);
      main_scene_width = width;
      main_scene_height = height;
      main_scene_pixels.resize(size_t(width) * height * 3);
      glBindTexture(GL_TEXTURE_2D, main_scene_texture);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
      glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0,
                   GL_RGB, GL_UNSIGNED_BYTE, nullptr);
    }
    mjr_setBuffer(mjFB_OFFSCREEN, &context);
    mjr_render(viewport, &scene, &context);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    mjr_readPixels(main_scene_pixels.data(), nullptr, viewport, &context);
    glBindTexture(GL_TEXTURE_2D, main_scene_texture);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height,
                    GL_RGB, GL_UNSIGNED_BYTE, main_scene_pixels.data());
#else
    mjr_render(viewport, &scene, &context);
#endif
    ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplSDL3_NewFrame(); ImGui::NewFrame();
#if defined(COMBATSIM_USE_FILAMENT)
    // Filament's readPixels path already flips OpenGL's bottom-up rows into
    // top-left image order, so the ImGui texture must not be V-flipped again.
    ImGui::GetBackgroundDrawList()->AddImage(
      (ImTextureID)(intptr_t)main_scene_texture, ImVec2(0, 0),
      ImGui::GetIO().DisplaySize, ImVec2(0, 0), ImVec2(1, 1));
#endif
    bool load_requested = false, apply_requested = false;
    FusionBundle* setup_bundle = &bundle;
    bool mixer_popup_requested = false;
    bool lan_popup_requested = false;
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    constexpr ImGuiWindowFlags fixed_window = ImGuiWindowFlags_NoTitleBar |
      ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
      ImGuiWindowFlags_NoDocking;

    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(display.x, 62));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.035f, 0.040f, 0.050f, 0.97f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.035f, 0.040f, 0.050f, 0.97f));
    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.95f, 0.97f, 1.0f, 1.0f));
    // The navigation bar stays in one place; each page below owns its controls.
    ImGui::Begin("##main_navigation", nullptr,
      fixed_window | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    ImDrawList* nav_draw = ImGui::GetWindowDrawList();
    nav_draw->AddRectFilled(ImVec2(0.0f, 61.0f), ImVec2(display.x, 62.0f),
                            IM_COL32(255, 255, 255, 24));
    nav_draw->AddRectFilled(ImVec2(21.0f, 15.0f), ImVec2(25.0f, 47.0f),
                            ImGui::GetColorU32(ui_accent), 2.0f);
    ImGui::SetCursorScreenPos(ImVec2(37.0f, 13.0f));
    ImGui::TextColored(ui_ink, "COMBAT");
    ImGui::SetCursorScreenPos(ImVec2(37.0f, 34.0f));
    ImGui::TextColored(ImVec4(0.63f, 0.65f, 0.68f, 1.0f), "ROBOT ARENA SIM");
    float nav_x = 184.0f;
    auto nav_button = [&](const char* label, AppScreen target) {
      const bool active = screen == target;
      const bool disabled = target == AppScreen::Configure &&
        (lan_bundle_swapped || (lan.GetState() != LanSession::State::Idle &&
         lan.GetState() != LanSession::State::Stopped &&
         lan.GetState() != LanSession::State::Failed));
      if (disabled) ImGui::BeginDisabled();
      ImGui::SetCursorScreenPos(ImVec2(nav_x, 10.0f));
      const bool clicked = ImGui::InvisibleButton(label, ImVec2(94.0f, 42.0f));
      const bool hovered = ImGui::IsItemHovered();
      const ImVec2 button_min = ImGui::GetItemRectMin();
      const ImVec2 button_max = ImGui::GetItemRectMax();
      if (active || hovered)
        nav_draw->AddRectFilled(button_min, button_max,
          active ? IM_COL32(255, 255, 255, 14) : IM_COL32(255, 255, 255, 8), 5.0f);
      if (active)
        nav_draw->AddRectFilled(ImVec2(button_min.x + 16.0f, button_max.y - 2.0f),
          ImVec2(button_max.x - 16.0f, button_max.y), ImGui::GetColorU32(ui_accent), 1.0f);
      const ImVec2 label_size = ImGui::CalcTextSize(label);
      nav_draw->AddText(ImVec2(button_min.x + (94.0f - label_size.x) * 0.5f,
                               button_min.y + (42.0f - label_size.y) * 0.5f),
        ImGui::GetColorU32(disabled ? ImVec4(0.38f, 0.39f, 0.41f, 1.0f) :
          active ? ui_ink : ImVec4(0.72f, 0.73f, 0.75f, 1.0f)), label);
      if (clicked) {
        screen = target;
        paused = target != AppScreen::Drive;
      }
      if (disabled) ImGui::EndDisabled();
      nav_x += 100.0f;
    };
    nav_button("Home", AppScreen::Home);
    nav_button("Arena", AppScreen::Drive);
    nav_button("Workshop", AppScreen::Configure);
    nav_button("Settings", AppScreen::Settings);
    const bool input_ready = joystick || pad;
    const ImVec2 status_pos(std::max(nav_x + 8.0f, display.x - 246.0f), 23.0f);
    nav_draw->AddCircleFilled(ImVec2(status_pos.x, status_pos.y + 6.0f), 4.0f,
      ImGui::GetColorU32(input_ready ? ui_green : ImVec4(0.62f, 0.64f, 0.67f, 1.0f)));
    ImGui::SetCursorScreenPos(ImVec2(status_pos.x + 13.0f, status_pos.y));
    ImGui::TextColored(input_ready ? ui_green : ImVec4(0.67f, 0.68f, 0.70f, 1.0f),
      "%s", joystick ? "JOYSTICK CONNECTED" :
        pad ? "GAMEPAD CONNECTED" : "KEYBOARD READY");
    ImGui::End();
    ImGui::PopStyleColor(3);

    if (screen == AppScreen::Home) {
      ImGui::SetNextWindowPos(ImVec2(0, 62), ImGuiCond_Always);
      ImGui::SetNextWindowSize(ImVec2(display.x, std::max(1.0f, display.y - 62.0f)),
                               ImGuiCond_Always);
      ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0, 0, 0, 0));
      ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0, 0, 0, 0));
      ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
      // Home contains launch actions, opponent selection, and the LAN entry point.
      ImGui::Begin("##home_screen", nullptr, fixed_window | ImGuiWindowFlags_NoBackground |
          ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
      ImDrawList* menu_draw = ImGui::GetWindowDrawList();
      const ImVec2 home_min = ImGui::GetWindowPos();
      const ImVec2 home_max(home_min.x + display.x, home_min.y + display.y - 62.0f);
      menu_draw->AddRectFilled(home_min, home_max, IM_COL32(5, 7, 10, 92));
      menu_draw->AddRectFilledMultiColor(
          home_min, ImVec2(home_min.x + display.x * 0.76f, home_max.y),
          IM_COL32(7, 8, 10, 246), IM_COL32(8, 9, 11, 68),
          IM_COL32(8, 9, 11, 68), IM_COL32(7, 8, 10, 246));
      menu_draw->AddRectFilledMultiColor(
          ImVec2(home_min.x, home_min.y + display.y * 0.59f), home_max,
          IM_COL32(5, 7, 9, 0), IM_COL32(5, 7, 9, 0),
          IM_COL32(5, 7, 9, 178), IM_COL32(5, 7, 9, 178));

      const float margin = std::clamp(display.x * 0.055f, 28.0f, 76.0f);
      const float menu_width = std::max(230.0f,
          std::min(480.0f, display.x - margin * 2.0f - 20.0f));
      const bool compact_home = display.x < 980.0f;
      const float title_size = display.y < 700.0f
          ? (display.y < 620.0f ? 31.0f : 34.0f) : 49.0f;
      const float hero_y = home_min.y + (display.y < 700.0f ? 24.0f : 46.0f);
      const ImU32 white = IM_COL32(240, 245, 250, 255);
      const ImU32 muted = IM_COL32(164, 177, 193, 255);
      const ImU32 accent = ImGui::GetColorU32(ui_accent);
      menu_draw->AddRectFilled(ImVec2(margin, hero_y), ImVec2(margin + 42.0f, hero_y + 3.0f),
                               accent, 1.5f);
      menu_draw->AddText(ImGui::GetFont(), 13.0f, ImVec2(margin, hero_y + 13.0f),
                         accent, "ROBOT COMBAT SIMULATOR  /  PRACTICE MODE");
      menu_draw->AddText(ImGui::GetFont(), title_size, ImVec2(margin, hero_y + 37.0f),
                         white, "COMBAT");
      menu_draw->AddText(ImGui::GetFont(), title_size, ImVec2(margin, hero_y + 37.0f + title_size),
                         white, "ARENA");
      const float hero_bottom = hero_y + 43.0f + 2.0f * title_size;
      menu_draw->AddText(ImGui::GetFont(), 16.0f, ImVec2(margin + 2.0f, hero_bottom + 4.0f),
                         muted, "Tune your machine. Take it into the arena.");
      const float status_y = hero_bottom + 37.0f;
      menu_draw->AddRectFilled(ImVec2(margin, status_y - 3.0f),
          ImVec2(margin + 184.0f, status_y + 20.0f), IM_COL32(12, 18, 16, 210), 11.0f);
      menu_draw->AddRect(ImVec2(margin, status_y - 3.0f),
          ImVec2(margin + 184.0f, status_y + 20.0f), IM_COL32(92, 143, 105, 95), 11.0f);
      menu_draw->AddCircleFilled(ImVec2(margin + 5.0f, status_y + 8.0f), 4.0f,
                                 ImGui::GetColorU32(ui_green));
      menu_draw->AddText(ImGui::GetFont(), 12.0f, ImVec2(margin + 17.0f, status_y),
                         ImGui::GetColorU32(ui_green), "LOCAL SIMULATION READY");

      const float item_height = std::clamp(display.y * 0.082f, 48.0f, 64.0f);
      const float menu_stack_height = 5.0f * item_height + 4.0f * 7.0f;
      const float menu_y = std::min(home_min.y + 285.0f,
          std::max(status_y + 27.0f, display.y - 72.0f - menu_stack_height));
      float menu_item_y = menu_y;
      auto menu_item = [&](const char* id, const char* index, const char* title,
                           const char* subtitle, ImVec4 item_accent, bool primary,
                           auto&& on_click) {
        ImGui::SetCursorScreenPos(ImVec2(margin, menu_item_y));
        const bool pressed = ImGui::InvisibleButton(id, ImVec2(menu_width, item_height));
        const bool hovered = ImGui::IsItemHovered();
        const ImVec2 item_min = ImGui::GetItemRectMin();
        const ImVec2 item_max = ImGui::GetItemRectMax();
        const ImU32 fill = primary
            ? (hovered ? IM_COL32(181, 70, 34, 242) : IM_COL32(133, 52, 29, 228))
            : (hovered ? IM_COL32(37, 40, 44, 236) : IM_COL32(15, 17, 20, 212));
        menu_draw->AddRectFilled(item_min, item_max, fill, 7.0f);
        menu_draw->AddRect(item_min, item_max,
          primary ? IM_COL32(248, 142, 91, 145) : IM_COL32(205, 211, 219, hovered ? 62 : 25), 7.0f);
        menu_draw->AddRectFilled(ImVec2(item_min.x, item_min.y + 8.0f),
            ImVec2(item_min.x + (primary ? 4.0f : 2.0f), item_max.y - 8.0f),
            primary ? accent : ImGui::GetColorU32(item_accent), 2.0f);
        const ImU32 index_color = ImGui::GetColorU32(item_accent);
        const ImU32 title_color = IM_COL32(246, 247, 249, 255);
        menu_draw->AddText(ImGui::GetFont(), 12.0f,
                           ImVec2(item_min.x + 16.0f, item_min.y + 13.0f), index_color, index);
        menu_draw->AddText(ImGui::GetFont(), 19.0f,
                           ImVec2(item_min.x + 54.0f, item_min.y + 9.0f), title_color, title);
        menu_draw->AddText(ImGui::GetFont(), 12.0f,
                           ImVec2(item_min.x + 54.0f, item_min.y + 36.0f), muted, subtitle);
        menu_draw->AddText(ImGui::GetFont(), 21.0f,
                           ImVec2(item_max.x - 30.0f, item_min.y + 20.0f), index_color, "> ");
        menu_item_y += item_height + 7.0f;
        if (pressed) on_click();
      };
      menu_item("##menu_drive", "01", "ENTER THE ARENA", "Practice driving and test your setup",
          ui_teal, true, [&] { screen = AppScreen::Drive; paused = false; });
      menu_item("##menu_configure", "02", "BOT WORKSHOP", "Import CAD, tune motors and configure parts",
          ui_blue, false, [&] { screen = AppScreen::Configure; paused = true; });
      menu_item("##menu_mixer", "03", "CONTROLLER SETUP", "Map joystick channels and response curves",
          ui_violet, false, [&] { mixer_popup_requested = true; });
      menu_item("##menu_settings", "04", "SETTINGS", "Arena layout, lighting and graphics",
          ui_amber, false, [&] { screen = AppScreen::Settings; paused = true; });
      menu_item("##menu_lan", "05", "LAN MULTIPLAYER", "Host a match or join another bot on your network",
          ui_coral, false, [&] { lan_popup_requested = true; });

      if (!compact_home) {
        const float info_x = display.x - margin - 292.0f;
        const float info_y = home_min.y + std::max(112.0f, display.y * 0.20f);
        const ImVec2 card_min(info_x - 18.0f, info_y - 18.0f);
        const ImVec2 card_max(display.x - margin, info_y + 356.0f);
        menu_draw->AddRectFilled(card_min, card_max, IM_COL32(11, 13, 16, 222), 9.0f);
        menu_draw->AddRect(card_min, card_max, IM_COL32(220, 224, 229, 46), 9.0f);
        menu_draw->AddRectFilled(ImVec2(card_min.x, card_min.y + 10.0f),
          ImVec2(card_min.x + 3.0f, card_max.y - 10.0f), accent, 2.0f);
        menu_draw->AddText(ImGui::GetFont(), 12.0f, ImVec2(info_x, info_y), accent,
                           "PRACTICE SESSION");
        menu_draw->AddText(ImGui::GetFont(), 20.0f, ImVec2(info_x, info_y + 22.0f),
                           white, "LOCAL ARENA");
        char rate_text[64];
        std::snprintf(rate_text, sizeof(rate_text), "%.0f HZ SOLVER", 1.0 / model->opt.timestep);
        menu_draw->AddCircleFilled(ImVec2(info_x + 5.0f, info_y + 67.0f), 3.5f,
                                   ImGui::GetColorU32(ui_green));
        menu_draw->AddText(ImGui::GetFont(), 12.0f, ImVec2(info_x + 15.0f, info_y + 60.0f),
                           ImGui::GetColorU32(ui_green), rate_text);
        menu_draw->AddLine(ImVec2(info_x, info_y + 86.0f),
                           ImVec2(card_max.x - 18.0f, info_y + 86.0f),
                           IM_COL32(213, 218, 226, 40), 1.0f);
        menu_draw->AddText(ImGui::GetFont(), 11.0f, ImVec2(info_x, info_y + 102.0f),
                           muted, "ACTIVE MODEL");
        const std::string active_model = model_path.filename().string();
        menu_draw->PushClipRect(ImVec2(info_x, info_y + 121.0f),
                                ImVec2(card_max.x - 18.0f, info_y + 143.0f), true);
        menu_draw->AddText(ImGui::GetFont(), 15.0f, ImVec2(info_x, info_y + 121.0f),
                           white, active_model.c_str());
        menu_draw->PopClipRect();
        menu_draw->AddLine(ImVec2(info_x, info_y + 158.0f),
                           ImVec2(card_max.x - 18.0f, info_y + 158.0f),
                           IM_COL32(213, 218, 226, 40), 1.0f);
        char arena_text[64];
        std::snprintf(arena_text, sizeof(arena_text), "%.2f m  |  %s",
                      app_settings.arena_size_m,
                      app_settings.spawn_layout == 0 ? "CENTER LINE" : "DIAGONAL SPAWN");
        menu_draw->AddText(ImGui::GetFont(), 11.0f, ImVec2(info_x, info_y + 174.0f),
                           muted, "ARENA CONFIGURATION");
        menu_draw->AddText(ImGui::GetFont(), 14.0f, ImVec2(info_x, info_y + 194.0f),
                           white, arena_text);
        menu_draw->AddLine(ImVec2(info_x, info_y + 222.0f),
                           ImVec2(card_max.x - 18.0f, info_y + 222.0f),
                           IM_COL32(213, 218, 226, 40), 1.0f);
        menu_draw->AddText(ImGui::GetFont(), 11.0f, ImVec2(info_x, info_y + 238.0f),
                           muted, "CONTROLS");
        const char* input_label = joystick ? "USB JOYSTICK" :
          pad ? "GAMEPAD" : "KEYBOARD";
        menu_draw->AddText(ImGui::GetFont(), 14.0f, ImVec2(info_x, info_y + 258.0f),
                           white, input_label);
        menu_draw->AddLine(ImVec2(info_x, info_y + 284.0f),
                           ImVec2(card_max.x - 18.0f, info_y + 284.0f),
                           IM_COL32(213, 218, 226, 40), 1.0f);
        menu_draw->AddText(ImGui::GetFont(), 11.0f, ImVec2(info_x, info_y + 299.0f),
                           muted, "IMPORTED BOT");
        menu_draw->AddCircleFilled(ImVec2(info_x + 5.0f, info_y + 321.0f), 4.0f,
            ImGui::GetColorU32(bundle.HasBundle() ? ui_green : ui_amber));
        menu_draw->AddText(ImGui::GetFont(), 13.0f, ImVec2(info_x + 17.0f, info_y + 313.0f),
            white, bundle.HasBundle() ? "READY" : "SAMPLE MODEL");
        if (bundle.HasBundle()) {
          char parts_text[64];
          std::snprintf(parts_text, sizeof(parts_text), "%d imported bodies", bundle.PartCount());
          menu_draw->AddText(ImGui::GetFont(), 11.0f, ImVec2(info_x, info_y + 337.0f),
                             muted, parts_text);
        }
      }

      if (!runtime_error.empty()) {
        ImGui::SetCursorScreenPos(ImVec2(margin, menu_y - 24.0f));
        ImGui::TextColored(ui_red, "%s", runtime_error.c_str());
      }
      const float footer_y = display.y - 47.0f;
      menu_draw->AddLine(ImVec2(margin, footer_y - 12.0f),
          ImVec2(display.x - margin, footer_y - 12.0f), IM_COL32(213, 218, 226, 42), 1.0f);
      ImGui::SetCursorScreenPos(ImVec2(margin, footer_y));
      ImGui::TextColored(ImVec4(0.69f, 0.70f, 0.72f, 1.0f),
                         "W / S DRIVE    A / D STEER    SPACE WEAPON");
      ImGui::SameLine(display.x - margin - 104.0f);
      push_button_palette(ImVec4(0.12f, 0.13f, 0.15f, 0.92f),
                          ImVec4(0.23f, 0.13f, 0.10f, 1.0f),
                          ImVec4(0.36f, 0.16f, 0.10f, 1.0f));
      if (ImGui::Button("QUIT GAME", ImVec2(104.0f, 28.0f))) running = false;
      pop_button_palette();
      ImGui::End();
      ImGui::PopStyleVar();
      ImGui::PopStyleColor(2);
    } else if (screen == AppScreen::Settings) {
      ImGui::SetNextWindowPos(ImVec2(0, 62), ImGuiCond_Always);
      ImGui::SetNextWindowSize(ImVec2(display.x, std::max(1.0f, display.y - 62.0f)),
                               ImGuiCond_Always);
      ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.055f, 0.075f, 0.105f, 0.96f));
      ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0, 0, 0, 0));
      // Arena and renderer preferences are separate from bot setup files.
      ImGui::Begin("##settings", nullptr, fixed_window);
      const float panel_width = std::min(760.0f, std::max(360.0f, display.x - 48.0f));
      ImGui::SetCursorPosX(std::max(24.0f, (display.x - panel_width) * 0.5f));
      ImGui::BeginChild("Settings panel", ImVec2(panel_width, display.y - 110.0f),
                        ImGuiChildFlags_Borders);
      ImGui::TextColored(ui_amber, "SETTINGS");
      ImGui::TextDisabled("Choose an arena setup or adjust how the simulator looks.");
      push_button_palette(settings_section == SettingsSection::Arena
                              ? ImVec4(0.24f, 0.43f, 0.64f, 1.0f)
                              : ImVec4(0.16f, 0.20f, 0.26f, 1.0f),
                          ImVec4(0.30f, 0.48f, 0.68f, 1.0f),
                          ImVec4(0.20f, 0.39f, 0.47f, 1.0f));
      if (ImGui::Button("Arena settings", ImVec2(150.0f, 36.0f)))
        settings_section = SettingsSection::Arena;
      pop_button_palette();
      ImGui::SameLine();
      push_button_palette(settings_section == SettingsSection::Graphics
                              ? ImVec4(0.24f, 0.43f, 0.64f, 1.0f)
                              : ImVec4(0.16f, 0.20f, 0.26f, 1.0f),
                          ImVec4(0.30f, 0.48f, 0.68f, 1.0f),
                          ImVec4(0.20f, 0.39f, 0.47f, 1.0f));
      if (ImGui::Button("Graphics settings", ImVec2(170.0f, 36.0f)))
        settings_section = SettingsSection::Graphics;
      pop_button_palette();
      ImGui::Separator();
      if (settings_section == SettingsSection::Arena) {
      ImGui::TextColored(ui_amber, "ARENA SETTINGS");
      ImGui::TextDisabled("Changes are saved on this computer and applied to the current arena.");
      const bool spawn_locked = lan_bundle_swapped || lan_connected;
      ImGui::SeparatorText("Arena size");
      ImGui::TextWrapped("The arena is square. This sets the inside floor and wall-to-wall side length in meters.");
      const float previous_arena_size_m = app_settings.arena_size_m;
      bool arena_size_changed = false;
      char current_arena_label[48]{};
      std::snprintf(current_arena_label, sizeof(current_arena_label), "%.2f m",
                    app_settings.arena_size_m);
      const std::array<ArenaSizePreset, 3> built_in_presets{{
        {"Small", 1.0f}, {"Standard", 2.0f}, {"Large", 3.5f}}};
      if (spawn_locked) ImGui::BeginDisabled();
      ImGui::SetNextItemWidth(std::min(480.0f, panel_width - 40.0f));
      if (ImGui::BeginCombo("Arena preset", current_arena_label)) {
        auto choose_preset = [&](const ArenaSizePreset& preset, const std::string& label) {
          if (ImGui::Selectable(label.c_str())) {
            const float selected_size = float(clamp_arena_size_m(preset.size_m));
            arena_size_changed |= std::abs(app_settings.arena_size_m - selected_size) > 1e-5f;
            app_settings.arena_size_m = selected_size;
            arena_preset_status.clear();
          }
        };
        for (const auto& preset : built_in_presets) {
          char label[96]{};
          std::snprintf(label, sizeof(label), "%s (%.2f m)", preset.name.c_str(), preset.size_m);
          choose_preset(preset, label);
        }
        if (!app_settings.arena_presets.empty()) ImGui::Separator();
        for (const auto& preset : app_settings.arena_presets) {
          char label[128]{};
          std::snprintf(label, sizeof(label), "%s (%.2f m)", preset.name.c_str(), preset.size_m);
          choose_preset(preset, label);
        }
        ImGui::EndCombo();
      }
      ImGui::SetNextItemWidth(std::min(480.0f, panel_width - 40.0f));
      arena_size_changed |= ImGui::SliderFloat("Arena side length (m)",
        &app_settings.arena_size_m, min_arena_size_m, max_arena_size_m, "%.2f m");
      ImGui::SetNextItemWidth(std::min(480.0f, panel_width - 40.0f));
      ImGui::InputText("New preset name", arena_preset_name, sizeof(arena_preset_name));
      std::string new_preset_name(arena_preset_name);
      const size_t name_start = new_preset_name.find_first_not_of(" \t\r\n");
      if (name_start == std::string::npos) new_preset_name.clear();
      else {
        const size_t name_end = new_preset_name.find_last_not_of(" \t\r\n");
        new_preset_name = new_preset_name.substr(name_start, name_end - name_start + 1);
      }
      ImGui::BeginDisabled(new_preset_name.empty());
      if (ImGui::Button("Save current size as preset", ImVec2(220.0f, 34.0f))) {
        auto existing = std::find_if(app_settings.arena_presets.begin(),
          app_settings.arena_presets.end(), [&](const ArenaSizePreset& preset) {
            return preset.name == new_preset_name;
          });
        if (existing == app_settings.arena_presets.end()) {
          app_settings.arena_presets.push_back({new_preset_name, app_settings.arena_size_m});
          arena_preset_status = "Saved preset: " + new_preset_name;
        } else {
          existing->size_m = app_settings.arena_size_m;
          arena_preset_status = "Updated preset: " + new_preset_name;
        }
        arena_preset_name[0] = '\0';
        save_app_settings(app_settings);
      }
      ImGui::EndDisabled();
      if (!arena_preset_status.empty()) ImGui::TextColored(ui_green, "%s", arena_preset_status.c_str());
      if (spawn_locked) ImGui::EndDisabled();
      if (spawn_locked)
        ImGui::TextDisabled("Arena settings are locked while a LAN match is connected.");
      if (arena_size_changed) {
        app_settings.arena_size_m = float(clamp_arena_size_m(app_settings.arena_size_m));
        apply_arena_size(model, app_settings.arena_size_m);
        // The visible floor and arena boundaries use primitive boxes, so the next
        // scene update picks up their new dimensions without rebuilding renderer resources.
        apply_arena_spawn_layout(model, data, app_settings.spawn_layout,
                                 app_settings.arena_size_m);
        if (previous_arena_size_m > 0.0f)
          camera.distance = std::clamp(camera.distance * app_settings.arena_size_m /
                                        previous_arena_size_m, 0.5, 15.0);
        paused = true;
        controls.weapon_command = 0.0;
        opponent_controls.weapon_command = 0.0;
        controls.filtered_wheel_speeds.clear();
        opponent_controls.filtered_wheel_speeds.clear();
        save_app_settings(app_settings);
      }
      ImGui::SeparatorText("Bot starting positions");
      const char* spawn_label = app_settings.spawn_layout == 0
        ? "Opposite ends of center line" : "Opposite arena diagonals";
      ImGui::SetNextItemWidth(std::min(480.0f, panel_width - 40.0f));
      bool spawn_changed = false;
      if (spawn_locked) ImGui::BeginDisabled();
      if (ImGui::BeginCombo("Spawn layout", spawn_label)) {
        if (ImGui::Selectable("Opposite ends of center line", app_settings.spawn_layout == 0)) {
          spawn_changed = app_settings.spawn_layout != 0;
          app_settings.spawn_layout = 0;
        }
        if (ImGui::Selectable("Opposite arena diagonals", app_settings.spawn_layout == 1)) {
          spawn_changed = app_settings.spawn_layout != 1;
          app_settings.spawn_layout = 1;
        }
        ImGui::EndCombo();
      }
      if (spawn_locked) ImGui::EndDisabled();
      if (spawn_locked)
        ImGui::TextDisabled("Spawn layout is locked while a LAN model is loaded.");
      if (spawn_changed) {
        save_app_settings(app_settings);
        apply_arena_spawn_layout(model, data, app_settings.spawn_layout,
                                 app_settings.arena_size_m);
        controls.weapon_command = 0.0;
        opponent_controls.weapon_command = 0.0;
        controls.filtered_wheel_speeds.clear();
        opponent_controls.filtered_wheel_speeds.clear();
      }
      ImGui::TextWrapped(app_settings.spawn_layout == 0
        ? "Bots start on opposite ends of the arena's center line, facing each other."
        : "Bots start at opposite diagonal positions and turn together to face across the arena.");
      } else {
      ImGui::TextColored(ui_blue, "GRAPHICS SETTINGS");
      ImGui::TextDisabled("Renderer changes restart CombatSim and reopen the current model.");
      ImGui::SeparatorText("Renderer");
      ImGui::Text("Active renderer: %s", default_renderer_id == 1
          ? "Filament PBR" : "MuJoCo classic");
      ImGui::TextWrapped("Filament adds physically based materials and image-based reflections. It is an experimental real-time renderer.");
      const int other_renderer_id = default_renderer_id == 1 ? 0 : 1;
      const auto other_renderer_exe = renderer_executable(other_renderer_id);
      const bool other_renderer_available = !other_renderer_exe.empty() &&
        std::filesystem::exists(other_renderer_exe);
      const bool renderer_switch_locked = lan_connected || lan_bundle_swapped ||
        !other_renderer_available;
      ImGui::BeginDisabled(renderer_switch_locked);
      bool requested_filament = default_renderer_id == 1;
      if (ImGui::Checkbox("Use Filament PBR renderer", &requested_filament) &&
          (requested_filament ? 1 : 0) != default_renderer_id) {
        graphics_status.clear();
        std::filesystem::path player_manifest;
        std::filesystem::path opponent_manifest;
        bool bundles_saved = true;
        auto save_bundle_for_switch = [&](FusionBundle& saved_bundle,
                                          std::filesystem::path& manifest,
                                          const char* label) {
          if (!saved_bundle.HasBundle()) return true;
          if (saved_bundle.IsMemoryLoaded() || saved_bundle.ManifestPath().empty()) {
            graphics_status = std::string(label) +
              " bot is only in memory and cannot be restored after switching.";
            return false;
          }
          if (!saved_bundle.Save()) {
            graphics_status = std::string("Could not save ") + label +
              " bot setup: " + saved_bundle.Error();
            return false;
          }
          manifest = saved_bundle.ManifestPath();
          return true;
        };
        bundles_saved &= save_bundle_for_switch(bundle, player_manifest, "Player");
        bundles_saved &= save_bundle_for_switch(opponent_bundle, opponent_manifest,
                                                 "Opponent");
        const int desired_renderer = requested_filament ? 1 : 0;
        if (bundles_saved) {
          app_settings.renderer = desired_renderer;
          if (!save_app_settings(app_settings)) {
            graphics_status = "Could not save the renderer preference.";
            app_settings.renderer = default_renderer_id;
          } else if (!launch_renderer_build(app_settings.renderer, model_path,
                                             graphics_status, player_manifest,
                                             opponent_manifest)) {
            app_settings.renderer = default_renderer_id;
            save_app_settings(app_settings);
          } else {
            running = false;
          }
        } else {
          app_settings.renderer = default_renderer_id;
        }
      }
      ImGui::EndDisabled();
      if (!other_renderer_available) {
        ImGui::TextDisabled(default_renderer_id == 1
          ? "Classic build is missing. Run Standalone\\setup.ps1 first."
          : "Filament build is missing. Run Standalone\\build_filament.ps1 first.");
      }
      if (lan_connected || lan_bundle_swapped)
        ImGui::TextDisabled("Renderer switching is unavailable during a LAN match.");
      if (!graphics_status.empty()) ImGui::TextColored(ui_red, "%s", graphics_status.c_str());
      ImGui::SeparatorText("Arena lighting");
      bool lighting_changed = false;
      ImGui::SetNextItemWidth(std::min(480.0f, panel_width - 40.0f));
      lighting_changed |= ImGui::SliderFloat("Ambient fill", &app_settings.ambient_light,
                                               0.0f, 0.8f, "%.2f");
      ImGui::SetNextItemWidth(std::min(480.0f, panel_width - 40.0f));
      lighting_changed |= ImGui::SliderFloat("Main arena light", &app_settings.key_light,
                                               0.0f, 2.0f, "%.2f x");
      ImGui::SetNextItemWidth(std::min(480.0f, panel_width - 40.0f));
      lighting_changed |= ImGui::SliderFloat("Soft fill light", &app_settings.fill_light,
                                               0.0f, 2.0f, "%.2f x");
      ImGui::SetNextItemWidth(std::min(480.0f, panel_width - 40.0f));
      lighting_changed |= ImGui::SliderFloat("Camera fill", &app_settings.camera_fill,
                                               0.0f, 1.0f, "%.2f");
      if (lighting_changed) save_app_settings(app_settings);
      if (ImGui::Button("Reset lighting", ImVec2(150.0f, 36.0f))) {
        app_settings.ambient_light = 0.12f;
        app_settings.key_light = 0.68f;
        app_settings.fill_light = 0.24f;
        app_settings.camera_fill = 0.07f;
        save_app_settings(app_settings);
      }
      ImGui::Separator();
      ImGui::TextColored(ui_green, "Arena floor opacity: 100%%");
      }
      if (ImGui::Button("Back to Home", ImVec2(150.0f, 36.0f))) {
        screen = AppScreen::Home;
        paused = true;
      }
      ImGui::EndChild();
      ImGui::End();
      ImGui::PopStyleColor(2);
    } else if (screen == AppScreen::Drive) {
      static float drive_sidebar_width = 380.0f;
      static float requested_drive_sidebar_width = 380.0f;
      static bool drive_sidebar_dragging = false;
      const float max_drive_sidebar_width = std::max(280.0f, std::min(760.0f, display.x - 32.0f));
      const float min_drive_sidebar_width = std::min(360.0f, max_drive_sidebar_width);
      drive_sidebar_width = std::clamp(drive_sidebar_width, min_drive_sidebar_width,
                                       max_drive_sidebar_width);
      if (!drive_sidebar_dragging) requested_drive_sidebar_width = drive_sidebar_width;
      ImGui::SetNextWindowPos(ImVec2(16, 76), ImGuiCond_Always);
      ImGui::SetNextWindowSize(ImVec2(drive_sidebar_width, display.y - 92), ImGuiCond_Always);
      ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.045f, 0.050f, 0.060f, 0.96f));
      ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.20f, 0.21f, 0.23f, 0.95f));
      // Drive owns match controls, controller telemetry, and manual robot gizmos.
      ImGui::Begin("##drive_dashboard", nullptr, fixed_window);
      const float drive_content_height = ImGui::GetContentRegionAvail().y;
      ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.070f, 0.077f, 0.088f, 0.98f));
      ImGui::BeginChild("Drive sidebar contents", ImVec2(-8.0f, 0.0f),
                        ImGuiChildFlags_None, ImGuiWindowFlags_AlwaysVerticalScrollbar);
      ImGui::PushTextWrapPos(520.0f);
      ImGui::TextColored(ui_accent, "PRACTICE DRIVE");
      ImGui::TextDisabled("MuJoCo simulation  |  %.0f Hz", 1.0 / model->opt.timestep);
      ImGui::Spacing();
      if (lan_client) {
        ImGui::BeginDisabled();
        ImGui::Button("Host controls simulation", ImVec2(190, 40));
        ImGui::EndDisabled();
      } else if (placement_mode) {
        ImGui::BeginDisabled();
        ImGui::Button("Paused for placement", ImVec2(166, 40));
        ImGui::EndDisabled();
      } else {
        push_button_palette(paused ? ImVec4(0.13f, 0.29f, 0.24f, 1.0f) :
                                      ImVec4(0.16f, 0.20f, 0.26f, 1.0f),
                            paused ? ImVec4(0.17f, 0.39f, 0.30f, 1.0f) :
                                      ImVec4(0.23f, 0.30f, 0.38f, 1.0f),
                            paused ? ImVec4(0.22f, 0.49f, 0.36f, 1.0f) :
                                      ImVec4(0.20f, 0.39f, 0.47f, 1.0f));
        if (ImGui::Button(paused ? "Resume simulation" : "Pause simulation", ImVec2(166, 40)))
          paused = !paused;
        pop_button_palette();
      }
      ImGui::SameLine();
      if (lan_client) ImGui::BeginDisabled();
      if (ImGui::Button("Reset robot", ImVec2(132, 40))) {
        apply_arena_spawn_layout(model, data, app_settings.spawn_layout,
                                 app_settings.arena_size_m);
        controls.weapon_command = 0.0;
        controls.filtered_wheel_speeds.clear();
        opponent_controls.filtered_wheel_speeds.clear();
        gizmo_drag_active = false; gizmo_drag_axis = -1;
      }
      if (lan_client) ImGui::EndDisabled();
      if (lan_connected) ImGui::BeginDisabled();
      if (ImGui::Button(placement_mode ? "Finish moving robot" : "Move", ImVec2(190, 40))) {
        placement_mode = !placement_mode;
        gizmo_drag_active = false;
        gizmo_drag_axis = -1;
        gizmo_mode = GizmoMode::Move;
        if (placement_mode) {
          paused = true;
          controls.weapon_command = 0.0;
          controls.filtered_wheel_speeds.clear();
          opponent_controls.filtered_wheel_speeds.clear();
          const int player_root = mj_name2id(model, mjOBJ_BODY, "Player_Robot");
          const int joint = free_joint_for_body(model, player_root);
          if (joint >= 0) {
            const int dof = model->jnt_dofadr[joint];
            for (int i = 0; i < 6; ++i) data->qvel[dof + i] = 0.0;
            mj_forward(model, data);
          }
        }
      }
      if (lan_connected) ImGui::EndDisabled();
      if (placement_mode) {
        ImGui::TextColored(gizmo_mode == GizmoMode::Move ? ui_teal : ui_amber,
            "%s MODE  |  R toggles Move / Rotate",
            gizmo_mode == GizmoMode::Move ? "MOVE" : "ROTATE");
        if (gizmo_mode == GizmoMode::Move)
          ImGui::TextWrapped("Drag the red X, green Y, or blue Z arrow to move only along that axis. Press R to show rotation rings. Right-drag pans; left-drag empty arena orbits.");
        else
          ImGui::TextWrapped("Drag a colored ring to rotate around its X, Y, or Z axis. Press R to return to the move arrows. Right-drag pans; left-drag empty arena orbits.");
      } else {
        ImGui::TextWrapped("Right-drag to pan the arena view, left-drag to orbit, and scroll to zoom.");
      }
      if (ImGui::Button("Controller mixer", ImVec2(280.0f, 38.0f))) {
        mixer_popup_requested = true;
      }
      if (lan_match_started) {
        ImGui::TextColored(ui_green, "%s multiplayer match",
          lan.GetRole() == LanSession::Role::Host ? "Hosting" : "Joined");
        if (lan.GetRole() == LanSession::Role::Host)
          ImGui::TextDisabled("You drive your bot with WASD; guest uses WASD for theirs.");
        else
          ImGui::TextDisabled("You drive your bot with WASD and Space. Host runs physics.");
        if (ImGui::Button("End LAN match", ImVec2(280.0f, 34.0f)))
          end_lan_session();
      } else if (ImGui::Button("LAN multiplayer", ImVec2(280.0f, 38.0f))) {
        lan_popup_requested = true;
      }
      ImGui::Text("Simulation time  %.2f s", data->time);
      ImGui::SeparatorText("Opponent control");
      if (lan_connected) {
        ImGui::TextColored(ui_green, "Remote player controls the opponent bot.");
      } else {
        ImGui::Checkbox("Aim and ram AI", &opponent_ai_enabled);
        ImGui::SetNextItemWidth(280.0f);
        ImGui::SliderFloat("Aggression", &opponent_ai_aggression_percent,
                           25.0f, 100.0f, "%.0f%%", ImGuiSliderFlags_AlwaysClamp);
        ImGui::TextDisabled("Scales the opponent's drive speed and turn rate.");
      }
      if (!lan_connected && opponent_ai_enabled && (opponent_controls.left.empty() ||
                                  opponent_controls.right.empty())) {
        ImGui::TextColored(ui_amber,
          "AI is waiting for an opponent bundle with left and right wheel drives.");
      } else if (!lan_connected && opponent_ai_enabled && !opponent_ai_delay_elapsed(data->time)) {
        ImGui::TextColored(ui_amber, "AI starts in %.1f s",
            std::max(0.0, opponent_ai_start_delay_seconds - data->time));
      } else if (!lan_connected && opponent_ai_enabled) {
        ImGui::TextColored(ui_green, "AI active: turns toward the player and rams forward");
        if (!opponent_controls.weapon.empty())
          ImGui::TextDisabled("Opponent weapon spins while AI is active.");
      } else if (!lan_connected) {
        ImGui::TextDisabled("Manual opponent: I/K drive, J/L turn, M weapon.");
      }
      const bool weapon_torque_enabled = weapon_uses_torque_model(telemetry_controls);
      const double weapon_rpm_limit = weapon_torque_enabled
        ? weapon_no_load_rpm(telemetry_controls) : telemetry_controls.weapon_rpm;
      if (local_bot_is_guest) {
        ImGui::TextDisabled("Host runs physics; these live RPMs are for your bot.");
        if (weapon_torque_enabled) {
          ImGui::Text("Weapon motor no-load estimate: %.0f RPM", weapon_rpm_limit);
          const double spinup_seconds = telemetry_bundle.EstimatedWeaponSpinupSeconds();
          if (spinup_seconds > 0.0 && std::isfinite(spinup_seconds))
            ImGui::Text("Estimated weapon spin-up: %.2f s", spinup_seconds);
        }
      } else {
        ImGui::SeparatorText("Drive settings");
        ImGui::Checkbox("Manual wheel RPM override", &controls.manual_wheel_rpm_override);
        if (controls.manual_wheel_rpm_override) {
          ImGui::SetNextItemWidth(280.0f);
          ImGui::InputFloat("Forced wheel RPM (0-5000)", &controls.wheel_rpm,
                            50.0f, 250.0f, "%.0f");
          if (!std::isfinite(controls.wheel_rpm)) controls.wheel_rpm = 0.0f;
          controls.wheel_rpm = std::clamp(controls.wheel_rpm, 0.0f, 5000.0f);
          ImGui::TextWrapped("Override uses a speed servo and bypasses drive motor torque. Set 4000 RPM here for a forced-speed check.");
        } else {
          const auto left_estimate = side_motor_speed_estimate(telemetry_controls.left,
            telemetry_controls.motor_models, telemetry_controls.battery_voltage);
          const auto right_estimate = side_motor_speed_estimate(telemetry_controls.right,
            telemetry_controls.motor_models, telemetry_controls.battery_voltage);
          ImGui::Text("Motor no-load estimate  L %.0f | R %.0f RPM",
                      left_estimate.rpm, right_estimate.rpm);
          if (left_estimate.modeled_wheels == 0 || right_estimate.modeled_wheels == 0)
            ImGui::TextWrapped("A side without motor data uses the 424 RPM fallback. Configure its motor in Configure for a motor-based estimate.");
          else
            ImGui::TextWrapped("Full throttle targets the motor speed estimate after pack-voltage and gear-ratio scaling. Loaded RPM comes from motor torque and simulated wheel load.");
        }
        if (weapon_torque_enabled) {
          ImGui::TextWrapped("Weapon motor no-load estimate: %.0f RPM", weapon_rpm_limit);
          const double weapon_spinup_seconds = telemetry_bundle.EstimatedWeaponSpinupSeconds();
          if (weapon_spinup_seconds > 0.0 && std::isfinite(weapon_spinup_seconds))
            ImGui::TextWrapped("Estimated 0-to-90%% no-load spin-up: %.2f s", weapon_spinup_seconds);
          else
            ImGui::TextWrapped("Estimated weapon spin-up is unavailable until its motor data and inertia are valid.");
          ImGui::TextWrapped("Estimate uses the motor torque curve and modeled weapon inertia; battery sag, impacts, and thermal derating are not included.");
          if (input_mixer.enabled && joystick)
            ImGui::TextWrapped("The controller mixer's Weapon output applies motor torque. Zero output lets the weapon coast down.");
          else
            ImGui::TextWrapped("Hold Space or the gamepad weapon button to apply player weapon motor torque. Opponent weapon uses M. Releasing lets each weapon coast under the modeled losses.");
        } else {
          ImGui::SetNextItemWidth(280.0f);
          ImGui::SliderFloat("Weapon target RPM", &controls.weapon_rpm, 0, 30000, "%.0f RPM");
          ImGui::SetNextItemWidth(280.0f);
          ImGui::SliderFloat("Weapon spin-up", &controls.spinup_seconds, 0.1f, 5.0f, "%.1f s");
        }
      }
      ImGui::SeparatorText(local_bot_is_guest ? "Your bot telemetry" : "Player bot telemetry");
      ImGui::Text(local_bot_is_guest ? "Your weapon actual" : "Weapon actual");
      double weapon_actual = 0.0;
      int weapon_readings = 0;
      for (int id : telemetry_controls.weapon) {
        const double rpm = actuator_rpm(model, data, id);
        if (std::isfinite(rpm)) { weapon_actual += std::abs(rpm); ++weapon_readings; }
      }
      if (weapon_readings) weapon_actual /= weapon_readings;
      ImGui::SameLine();
      if (weapon_readings) ImGui::TextColored(ui_teal,
                                               "%.0f RPM", weapon_actual);
      else ImGui::TextDisabled("No weapon drive assigned");
      ImGui::ProgressBar(weapon_rpm_limit > 0.0 ?
        float(std::clamp(weapon_actual / weapon_rpm_limit, 0.0, 1.0)) : 0.0f,
        ImVec2(-1, 12), "");
      auto draw_wheel_telemetry = [&](const char* side,
                                      const std::vector<std::pair<int, double>>& wheels) {
        ImGui::TextColored(std::string(side) == "Left" ? ui_blue : ui_coral,
                           "%s wheels  (%d)", side, int(wheels.size()));
        if (wheels.empty()) { ImGui::TextDisabled("No wheel drives assigned"); return; }
        double sum = 0.0; int valid = 0;
        for (size_t i = 0; i < wheels.size(); ++i) {
          const double rpm = actuator_rpm(model, data, wheels[i].first, wheels[i].second);
          if (!std::isfinite(rpm)) continue;
          sum += rpm; ++valid;
          ImGui::Text("  %s %d: %+.0f RPM", side, int(i + 1), rpm);
          ImGui::SameLine();
          const WheelSlip slip = wheel_slip(model, data, wheels[i].first);
          if (!slip.valid) {
            ImGui::TextDisabled("SLIP UNKNOWN");
          } else if (!slip.has_ground_contact) {
            ImGui::TextColored(ui_amber, "AIRBORNE");
          } else if (slip.speed > 0.18) {
            ImGui::TextColored(ui_red, "SLIPPING  %.2f m/s", slip.speed);
          } else {
            ImGui::TextColored(ui_green, "GRIP  %.2f m/s", slip.speed);
          }
        }
        if (valid) ImGui::TextColored(ui_ink,
                                      "  Average: %+.0f RPM", sum / valid);
      };
      draw_wheel_telemetry("Left", telemetry_controls.left);
      draw_wheel_telemetry("Right", telemetry_controls.right);
      ImGui::TextWrapped("Wheel slip compares tread motion with the floor at each contact patch. Slip is flagged above 0.18 m/s.");
      ImGui::SeparatorText("Controls");
      ImGui::TextWrapped("W / S drive forward and reverse. A / D steer; with no throttle, both sides counter-rotate for a spot turn. Hold Space to spin the weapon.");
      ImGui::TextWrapped("Open Controller mixer to map USB joystick channels and tune motor/weapon mixes.");
      ImGui::Text("Drive joints: left %d  |  right %d",
                  int(telemetry_controls.left.size()), int(telemetry_controls.right.size()));
      if (telemetry_controls.left.empty() || telemetry_controls.right.empty())
        ImGui::TextWrapped("Assign at least one wheel assembly to each side in Configure, then Apply and preview.");
      if (!runtime_error.empty()) {
        ImGui::PushStyleColor(ImGuiCol_Text, ui_red);
        ImGui::TextWrapped("Model message: %s", runtime_error.c_str());
        ImGui::PopStyleColor();
      }
      ImGui::PopTextWrapPos();
      ImGui::EndChild();
      ImGui::PopStyleColor();
      ImGui::SameLine(0.0f, 0.0f);
      ImGui::InvisibleButton("##drive_sidebar_splitter",
                             ImVec2(8.0f, drive_content_height));
      const bool drive_splitter_hovered = ImGui::IsItemHovered();
      const bool drive_splitter_active = ImGui::IsItemActive();
      if (drive_splitter_hovered || drive_splitter_active)
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
      if (drive_splitter_hovered && !drive_sidebar_dragging)
        ImGui::SetTooltip("Drag to reveal more of the fixed-size panel content.");
      if (drive_splitter_active) {
        if (!drive_sidebar_dragging) requested_drive_sidebar_width = drive_sidebar_width;
        drive_sidebar_dragging = true;
        requested_drive_sidebar_width = std::clamp(
          requested_drive_sidebar_width + ImGui::GetIO().MouseDelta.x,
          min_drive_sidebar_width, max_drive_sidebar_width);
      } else if (drive_sidebar_dragging) {
        drive_sidebar_width = requested_drive_sidebar_width;
        drive_sidebar_dragging = false;
      }
      const ImVec2 drive_splitter_min = ImGui::GetItemRectMin();
      const ImVec2 drive_splitter_max = ImGui::GetItemRectMax();
      const ImU32 drive_splitter_color = ImGui::GetColorU32(
          drive_splitter_active ? ImGuiCol_SeparatorActive :
          drive_splitter_hovered ? ImGuiCol_SeparatorHovered : ImGuiCol_Separator);
      ImGui::GetWindowDrawList()->AddRectFilled(
          ImVec2(drive_splitter_min.x + 3.0f, drive_splitter_min.y + 4.0f),
          ImVec2(drive_splitter_min.x + 5.0f, drive_splitter_max.y - 4.0f),
          drive_splitter_color, 1.0f);
      if (drive_sidebar_dragging) {
        const float ghost_x = drive_splitter_min.x +
          requested_drive_sidebar_width - drive_sidebar_width;
        ImGui::GetForegroundDrawList()->AddLine(
          ImVec2(ghost_x, 76.0f), ImVec2(ghost_x, display.y - 16.0f),
          drive_splitter_color, 2.0f);
      }
      ImGui::End();
      ImGui::PopStyleColor(2);
    } else {
      ImGui::SetNextWindowPos(ImVec2(12, 72), ImGuiCond_Always);
      ImGui::SetNextWindowSize(ImVec2(display.x - 24, display.y - 84), ImGuiCond_Always);
      ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.075f, 0.09f, 0.12f, 0.98f));
      ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.24f, 0.29f, 0.36f, 0.95f));
      // Bot workshop selects a local player/opponent bundle and edits its setup.
      ImGui::Begin("##configure_bots", nullptr, fixed_window);
      const float panel_width = ImGui::GetContentRegionAvail().x;
      const float splitter_width = 10.0f;
      const float column_spacing = ImGui::GetStyle().ItemSpacing.x;
      const float min_editor_width = std::min(340.0f, std::max(180.0f, panel_width * 0.35f));
      const float min_preview_width = std::min(300.0f, std::max(180.0f, panel_width * 0.30f));
      const float max_editor_width = std::max(min_editor_width,
          panel_width - min_preview_width - splitter_width - 2.0f * column_spacing);
      static float editor_width = 0.0f;
      static float requested_editor_width = 0.0f;
      static bool editor_dragging = false;
      if (editor_width <= 0.0f) {
        editor_width = std::min(480.0f, std::max(340.0f, display.x * 0.39f));
        requested_editor_width = editor_width;
      }
      editor_width = std::clamp(editor_width, min_editor_width, max_editor_width);
      if (!editor_dragging) requested_editor_width = editor_width;
      const float splitter_height = ImGui::GetContentRegionAvail().y;
      ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.115f, 0.145f, 0.19f, 0.98f));
      ImGui::BeginChild("Bot configuration", ImVec2(editor_width, 0),
                        ImGuiChildFlags_Borders, ImGuiWindowFlags_AlwaysVerticalScrollbar);
      ImGui::PushTextWrapPos(520.0f);
      ImGui::TextColored(ui_blue, "BOT CONFIGURATION");
      ImGui::TextDisabled("Configure imported bodies, wheels, weapons, and forks");
      const bool player_tab_active = config_bot == 0;
      if (player_tab_active)
        push_button_palette(ImVec4(0.16f, 0.27f, 0.39f, 1.0f),
                            ImVec4(0.21f, 0.37f, 0.53f, 1.0f),
                            ImVec4(0.25f, 0.46f, 0.65f, 1.0f));
      if (ImGui::Button("Player bot", ImVec2(132, 32))) {
        if (config_bot != 0) {
          config_bot = 0;
          load_requested = bundle.HasBundle();
        }
      }
      if (player_tab_active) pop_button_palette();
      ImGui::SameLine();
      const bool opponent_tab_active = config_bot == 1;
      if (opponent_tab_active)
        push_button_palette(ImVec4(0.39f, 0.22f, 0.20f, 1.0f),
                            ImVec4(0.53f, 0.29f, 0.24f, 1.0f),
                            ImVec4(0.65f, 0.34f, 0.27f, 1.0f));
      if (ImGui::Button("Opponent bot", ImVec2(132, 32))) {
        if (config_bot != 1) {
          config_bot = 1;
          load_requested = opponent_bundle.HasBundle();
        }
      }
      if (opponent_tab_active) pop_button_palette();
      setup_bundle = config_bot == 1 ? &opponent_bundle : &bundle;
      if (config_bot == 1 && opponent_bundle.HasBundle()) {
        ImGui::SameLine();
        if (ImGui::SmallButton("Remove opponent")) {
          opponent_bundle.Clear();
          config_bot = 0;
          setup_bundle = &bundle;
          load_requested = bundle.HasBundle();
          apply_requested = true;
        }
        ImGui::TextWrapped("Opponent modules use their own saved setup and actuators. In Drive, I/K move it forward/back, J/L turn it, and M spins its weapon.");
      }
      ImGui::Separator();
      setup_bundle->Draw(window, load_requested, apply_requested);
      if (config_bot == 1 && load_requested && opponent_bundle.HasBundle())
        apply_requested = true;
      if (!runtime_error.empty()) ImGui::TextColored(ui_red,
                                                       "Model message: %s", runtime_error.c_str());
      ImGui::PopTextWrapPos();
      ImGui::EndChild();
      ImGui::PopStyleColor();
      ImGui::SameLine();
      ImGui::InvisibleButton("##bot_config_splitter", ImVec2(splitter_width, splitter_height));
      const bool splitter_hovered = ImGui::IsItemHovered();
      const bool splitter_active = ImGui::IsItemActive();
      if (splitter_hovered || splitter_active)
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
      if (splitter_hovered && !editor_dragging)
        ImGui::SetTooltip("Drag to reveal more of the fixed-size panel content.");
      if (splitter_active) {
        if (!editor_dragging) requested_editor_width = editor_width;
        editor_dragging = true;
        requested_editor_width = std::clamp(
          requested_editor_width + ImGui::GetIO().MouseDelta.x,
          min_editor_width, max_editor_width);
      } else if (editor_dragging) {
        editor_width = requested_editor_width;
        editor_dragging = false;
      }
      const ImVec2 splitter_min = ImGui::GetItemRectMin();
      const ImVec2 splitter_max = ImGui::GetItemRectMax();
      const ImU32 splitter_color = ImGui::GetColorU32(splitter_active ? ImGuiCol_SeparatorActive :
          splitter_hovered ? ImGuiCol_SeparatorHovered : ImGuiCol_Separator);
      ImGui::GetWindowDrawList()->AddRectFilled(
          ImVec2(splitter_min.x + splitter_width * 0.5f - 1.0f, splitter_min.y + 4.0f),
          ImVec2(splitter_min.x + splitter_width * 0.5f + 1.0f, splitter_max.y - 4.0f),
          splitter_color, 1.0f);
      if (editor_dragging) {
        const float ghost_x = splitter_min.x + requested_editor_width - editor_width;
        ImGui::GetForegroundDrawList()->AddLine(
          ImVec2(ghost_x, 72.0f), ImVec2(ghost_x, display.y - 12.0f),
          splitter_color, 2.0f);
      }
      ImGui::SameLine();
      ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.10f, 0.13f, 0.17f, 1.0f));
      ImGui::BeginChild("Robot preview panel", ImVec2(0, 0), ImGuiChildFlags_Borders);
      ImGui::TextColored(config_bot == 1 ? ui_coral : ui_blue,
                         config_bot == 1 ? "OPPONENT PREVIEW" : "PLAYER PREVIEW");
      setup_bundle->DrawAxisSelectionControls();
      ImGui::TextWrapped("Hover to highlight a part; click it to select it and jump to its settings. In axis mode, hover a hole rim until its edge loop highlights, then click. Left drag orbits; right drag pans; scroll zooms.");
      if (setup_model && setup_bundle->HasBundle()) {
        if (ImGui::SmallButton("Reset view")) {
          const auto center = setup_bundle->PreviewCenter();
          for (int i = 0; i < 3; ++i) setup_camera.lookat[i] = center[i];
          setup_camera.distance = setup_bundle->PreviewDistance();
          setup_camera.azimuth = 45; setup_camera.elevation = -45;
        }
        ImGui::SameLine();
        const double min_tilt = -89, max_tilt = 89;
        ImGui::SetNextItemWidth(180);
        ImGui::SliderScalar("Tilt", ImGuiDataType_Double, &setup_camera.elevation,
                            &min_tilt, &max_tilt, "%.0f deg");
      }
      if (setup_model && setup_bundle->HasBundle()) {
        const ImVec2 available = ImGui::GetContentRegionAvail();
        const int preview_width = std::max(1, int(available.x));
        const int preview_height = std::max(1, int(available.y));
#if !defined(COMBATSIM_USE_FILAMENT)
        if (preview_width != setup_context.offWidth || preview_height != setup_context.offHeight)
          mjr_resizeOffscreen(preview_width, preview_height, &setup_context);
#endif
        if (preview_width != texture_width || preview_height != texture_height) {
#if defined(COMBATSIM_USE_FILAMENT)
          if (texture_width > 0) mjr_setBuffer(mjFB_WINDOW, &setup_context);
#endif
          texture_width = preview_width; texture_height = preview_height;
          preview_pixels.resize(size_t(preview_width) * preview_height * 3);
          glBindTexture(GL_TEXTURE_2D, preview_texture);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
          glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, preview_width, preview_height, 0,
                       GL_RGB, GL_UNSIGNED_BYTE, nullptr);
        }
        for (int i = 0; i < setup_model->ngeom; ++i)
          setup_model->geom_group[i] = setup_bundle->PreviewVisible(i) ? 0 : 5;
        options.geomgroup[5] = 0;
        apply_arena_lighting(setup_model, app_settings);
        mjv_updateScene(setup_model, setup_data, &options, nullptr, &setup_camera, mjCAT_ALL, &setup_scene);
        const ImVec2 canvas_min = ImGui::GetCursorScreenPos();
        const ImVec2 mouse = ImGui::GetIO().MousePos;
        const bool mouse_on_canvas = ImGui::IsWindowHovered() &&
          mouse.x >= canvas_min.x && mouse.y >= canvas_min.y &&
          mouse.x < canvas_min.x + preview_width && mouse.y < canvas_min.y + preview_height;
        if (mouse_on_canvas && !ImGui::IsMouseDragging(ImGuiMouseButton_Left) &&
            !ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
          mjtNum hit[3];
          const int part = SelectPreviewPartForHover(setup_model, setup_data, &options,
            double(preview_width)/preview_height,
            (mouse.x-canvas_min.x)/preview_width,
            1.0-(mouse.y-canvas_min.y)/preview_height,
            setup_scene, *setup_bundle, hit);
          setup_bundle->UpdateHover(part, {hit[0], hit[1], hit[2]});
        } else setup_bundle->UpdateHover(-1, {});
        if (!setup_bundle->AxisPicking()) {
          const int selected = setup_bundle->SelectedPart();
          for (int highlight : {setup_bundle->HoveredPart(), selected}) if (highlight >= 0) {
            const std::string name = "cad_part_" + std::to_string(highlight);
            const int geom_id = mj_name2id(setup_model, mjOBJ_GEOM, name.c_str());
            for (int i = 0; i < setup_scene.ngeom; ++i)
              if (setup_scene.geoms[i].objtype == mjOBJ_GEOM && setup_scene.geoms[i].objid == geom_id) {
                setup_scene.geoms[i].rgba[0] = highlight == selected ? 1.0f : 0.2f;
                setup_scene.geoms[i].rgba[1] = highlight == selected ? 0.8f : 0.9f;
                setup_scene.geoms[i].rgba[2] = highlight == selected ? 0.1f : 1.0f;
              }
          }
        }
        mjtNum camera_head[3], camera_forward[3], camera_up[3];
        mjv_cameraInModel(camera_head, camera_forward, camera_up, &setup_scene);
        setup_bundle->AddSetupAxisMarkers(setup_scene,
          {camera_head[0], camera_head[1], camera_head[2]},
          setup_bundle->ShowAxisMarkers());
        mjr_setBuffer(mjFB_OFFSCREEN, &setup_context);
        mjr_render({0, 0, preview_width, preview_height}, &setup_scene, &setup_context);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        mjr_readPixels(preview_pixels.data(), nullptr,
                       {0, 0, preview_width, preview_height}, &setup_context);
#if !defined(COMBATSIM_USE_FILAMENT)
        mjr_setBuffer(mjFB_WINDOW, &setup_context);
#endif
        glBindTexture(GL_TEXTURE_2D, preview_texture);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, preview_width, preview_height,
                        GL_RGB, GL_UNSIGNED_BYTE, preview_pixels.data());
        ImGui::InvisibleButton("CAD canvas", ImVec2(float(preview_width), float(preview_height)),
                               ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
        const ImVec2 image_min = ImGui::GetItemRectMin();
        const ImVec2 image_max = ImGui::GetItemRectMax();
#if defined(COMBATSIM_USE_FILAMENT)
        ImGui::GetWindowDrawList()->AddImage((ImTextureID)(intptr_t)preview_texture,
                                             image_min, image_max, ImVec2(0, 0), ImVec2(1, 1));
#else
        ImGui::GetWindowDrawList()->AddImage((ImTextureID)(intptr_t)preview_texture,
                                             image_min, image_max, ImVec2(0, 1), ImVec2(1, 0));
#endif
        const bool canvas_click = mouse.x >= image_min.x && mouse.y >= image_min.y &&
          mouse.x < image_max.x && mouse.y < image_max.y &&
          ImGui::IsMouseReleased(ImGuiMouseButton_Left) &&
          ImGui::GetIO().MouseDragMaxDistanceSqr[ImGuiMouseButton_Left] <= 16.0f;
        if (ImGui::IsItemHovered()) {
          if (ImGui::GetIO().MouseWheel != 0)
            setup_camera.distance = std::clamp(setup_camera.distance * std::pow(0.9, ImGui::GetIO().MouseWheel), 0.05, 15.0);
          if (ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
            setup_camera.azimuth += ImGui::GetIO().MouseDelta.x * 0.4;
            setup_camera.elevation = std::clamp(setup_camera.elevation - ImGui::GetIO().MouseDelta.y * 0.4, -89.0, 89.0);
          }
          if (ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
            mjtNum head[3], forward[3], up[3], right[3];
            mjv_cameraInModel(head, forward, up, &setup_scene);
            mju_cross(right, forward, up);
            const double scale = 2.0 * setup_camera.distance * std::tan(22.5*pi/180.0) / preview_height;
            for (int i = 0; i < 3; ++i)
              setup_camera.lookat[i] += (-right[i]*ImGui::GetIO().MouseDelta.x +
                                          up[i]*ImGui::GetIO().MouseDelta.y) * scale;
          }
        }
        // Re-pick at release using the displayed image's exact rectangle. This
        // keeps the fitted loop synchronized with the click even if the hover
        // ray was stale for one frame while crossing into the preview canvas.
        if (canvas_click) {
          if (setup_bundle->AxisPicking()) {
            mjtNum hit[3];
            const int part = SelectPreviewPartForHover(setup_model, setup_data,
              &options, double(preview_width)/preview_height,
              (mouse.x-image_min.x)/preview_width,
              1.0-(mouse.y-image_min.y)/preview_height,
              setup_scene, *setup_bundle, hit);
            setup_bundle->UpdateHover(part, {hit[0], hit[1], hit[2]});
            setup_bundle->SelectHoveredHole();
          } else if (setup_bundle->PrintBedFacePicking()) {
            mjtNum hit[3];
            const int part = SelectPreviewPartForHover(setup_model, setup_data,
              &options, double(preview_width)/preview_height,
              (mouse.x-image_min.x)/preview_width,
              1.0-(mouse.y-image_min.y)/preview_height,
              setup_scene, *setup_bundle, hit);
            setup_bundle->UpdateHover(part, {hit[0], hit[1], hit[2]});
            setup_bundle->SelectHoveredPrintBedFace();
          } else {
            mjtNum hit[3];
            const int part = SelectPreviewPartForHover(setup_model, setup_data,
              &options, double(preview_width)/preview_height,
              (mouse.x-image_min.x)/preview_width,
              1.0-(mouse.y-image_min.y)/preview_height,
              setup_scene, *setup_bundle, hit);
            setup_bundle->UpdateHover(part, {hit[0], hit[1], hit[2]});
            if (part >= 0) setup_bundle->SelectPart(part);
          }
        }
      } else ImGui::TextWrapped("Load a Fusion manifest in this bot's configuration to preview and select its parts.");
      ImGui::EndChild();
      ImGui::PopStyleColor();
      ImGui::End();
      ImGui::PopStyleColor(2);
    }
    if (lan_popup_requested) {
      lan_popup_open = true;
      ImGui::OpenPopup("LAN multiplayer");
    }
    if (lan_popup_open) {
      ImGui::SetNextWindowPos(ImVec2(display.x * 0.5f, display.y * 0.5f),
                              ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
      ImGui::SetNextWindowSize(ImVec2(std::min(660.0f, display.x - 36.0f),
                                      std::min(590.0f, display.y - 36.0f)),
                               ImGuiCond_Appearing);
      ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.075f, 0.09f, 0.12f, 0.99f));
      ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.30f, 0.39f, 0.47f, 1.0f));
      if (ImGui::BeginPopupModal("LAN multiplayer", &lan_popup_open,
          ImGuiWindowFlags_NoCollapse)) {
        ImGui::TextColored(ui_coral, "PEER-TO-PEER LAN MATCH");
        ImGui::TextWrapped("Both players load their own configured Fusion bot. Each player uses W/A/S/D and Space for their own bot. The host runs MuJoCo; the joining player sends controls and receives live physics state.");
        ImGui::Separator();
        const LanSession::State session_state = lan.GetState();
        const bool can_start = session_state == LanSession::State::Idle ||
          session_state == LanSession::State::Stopped ||
          session_state == LanSession::State::Failed;
        if (!bundle.HasBundle()) {
          ImGui::TextColored(ui_amber, "Load and apply a player bot in Configure before starting a match.");
        } else if (bundle.ManifestPath().empty()) {
          ImGui::TextColored(ui_amber, "For this first version, start from a bot loaded from a local Fusion manifest.");
        } else {
          ImGui::TextColored(ui_green, "Local bot ready: %d CAD bodies", bundle.PartCount());
        }
        ImGui::SetNextItemWidth(300.0f);
        ImGui::InputText("Host IPv4 / name", lan_address, sizeof(lan_address));
        ImGui::SetNextItemWidth(140.0f);
        ImGui::InputText("TCP port", lan_port_text, sizeof(lan_port_text));
        ImGui::SetNextItemWidth(350.0f);
        ImGui::InputText("Session key (share privately)", lan_passphrase,
                         sizeof(lan_passphrase));
        if (lan.GetRole() == LanSession::Role::Host) {
          ImGui::SameLine();
          if (ImGui::SmallButton("Copy key")) SDL_SetClipboardText(lan_passphrase);
        }
        const unsigned long requested_port = std::strtoul(lan_port_text, nullptr, 10);
        const bool valid_port = requested_port > 0 && requested_port <= 65535;
        const bool local_ready = bundle.HasBundle() && !bundle.ManifestPath().empty() &&
          !bundle.IsMemoryLoaded() && !lan_bundle_swapped;
        ImGui::BeginDisabled(!can_start || !local_ready || !valid_port);
        if (ImGui::Button("Host match", ImVec2(150.0f, 38.0f))) {
          std::vector<unsigned char> package;
          if (!bundle.ExportSessionPackage(package)) {
            runtime_error = bundle.Error();
          } else {
            previous_player_manifest = bundle.ManifestPath();
            previous_opponent_manifest = opponent_bundle.ManifestPath();
            std::string error;
            if (!lan.StartHost(uint16_t(requested_port), std::move(package),
                               lan_passphrase, error)) runtime_error = error;
            else {
              lan_bundle_swapped = false;
              lan_model_installed = false;
              lan_match_started = false;
              runtime_error.clear();
            }
            wipe_package(package);
          }
        }
        ImGui::SameLine();
        if (ImGui::Button("Join match", ImVec2(150.0f, 38.0f))) {
          std::vector<unsigned char> package;
          if (!bundle.ExportSessionPackage(package)) {
            runtime_error = bundle.Error();
          } else {
            wipe_package(local_client_package);
            local_client_package = package;
            previous_player_manifest = bundle.ManifestPath();
            previous_opponent_manifest = opponent_bundle.ManifestPath();
            std::string error;
            if (!lan.StartClient(lan_address, uint16_t(requested_port),
                                 std::move(package), lan_passphrase, error)) {
              runtime_error = error;
              wipe_package(local_client_package);
            } else {
              lan_bundle_swapped = false;
              lan_model_installed = false;
              lan_match_started = false;
              runtime_error.clear();
            }
            wipe_package(package);
          }
        }
        ImGui::EndDisabled();

        ImGui::Spacing();
        switch (session_state) {
          case LanSession::State::Starting:
            ImGui::TextColored(ui_amber, "Starting LAN transport..."); break;
          case LanSession::State::Waiting:
            ImGui::TextColored(ui_green, "Hosting on port %u; waiting for one player.",
                               unsigned(lan.BoundPort()));
            ImGui::TextWrapped("Share your PC's local IPv4 address, this port, and the session key. On Windows, allow CombatSim through the private-network firewall prompt.");
            break;
          case LanSession::State::PackageReady:
            ImGui::TextColored(ui_amber, "Bot received. Validating CAD and building the shared MuJoCo model..."); break;
          case LanSession::State::Ready:
            ImGui::TextColored(ui_amber, "Your model is ready; waiting for the other PC to finish loading..."); break;
          case LanSession::State::Connected:
            ImGui::TextColored(ui_green, lan.GetRole() == LanSession::Role::Host
              ? "Connected. You host physics and drive your bot with WASD; guest uses WASD for theirs."
              : "Connected. Host runs physics; you drive your bot with WASD and Space.");
            break;
          case LanSession::State::Failed:
            ImGui::TextColored(ui_red, "LAN session failed: %s", lan.Error().c_str()); break;
          default:
            ImGui::TextDisabled("No LAN session is active."); break;
        }
        if (lan_bundle_swapped || (session_state != LanSession::State::Idle &&
            session_state != LanSession::State::Stopped)) {
          if (ImGui::Button("End / reset LAN session", ImVec2(210.0f, 34.0f)))
            end_lan_session();
        }
        ImGui::Separator();
        ImGui::TextWrapped("CAD packages are encrypted in transit and received meshes are loaded in RAM without extracting files. This prevents ordinary bundle-file browsing; a determined peer can still inspect geometry from process memory or the rendered model, so it is not copy-proof or anti-cheat.");
        ImGui::TextDisabled("For the first LAN version, both PCs must be on a reachable network. It does not relay through the internet or configure router port forwarding.");
        if (!runtime_error.empty())
          ImGui::TextColored(ui_red, "%s", runtime_error.c_str());
        ImGui::SetCursorPosX(std::max(16.0f, ImGui::GetWindowWidth() - 112.0f));
        if (ImGui::Button("Close", ImVec2(88.0f, 34.0f))) {
          lan_popup_open = false;
          ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
      }
      ImGui::PopStyleColor(2);
    }
    if (mixer_popup_requested) {
      mixer_popup_open = true;
      ImGui::OpenPopup("Controller mixer setup");
    }
    if (mixer_popup_open) {
      const ImVec2 mixer_window_size(
        std::min(1120.0f, std::max(640.0f, display.x - 48.0f)),
        std::min(720.0f, std::max(480.0f, display.y - 90.0f)));
      ImGui::SetNextWindowPos(ImVec2(display.x * 0.5f, display.y * 0.5f),
                              ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
      ImGui::SetNextWindowSize(mixer_window_size, ImGuiCond_Appearing);
      ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.055f, 0.060f, 0.070f, 0.99f));
      ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.25f, 0.26f, 0.28f, 1.0f));
      if (ImGui::BeginPopupModal("Controller mixer setup", &mixer_popup_open,
          ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse)) {
        ImGui::TextColored(ui_accent, "CONTROLLER SETUP");
        ImGui::TextDisabled("Connect a controller, detect its sticks and buttons, then map inputs to this bot.");
        ImGui::Separator();
        draw_mixer_dialog(input_mixer, joystick, joystick_id, controls,
                          active_mixer_profile_key, active_mixer_profile_name);
        ImGui::Separator();
        ImGui::TextDisabled("Controller map is saved for %s; other bots keep their own setup.",
                            active_mixer_profile_name.c_str());
        ImGui::SameLine(ImGui::GetWindowWidth() - 118.0f);
        if (ImGui::Button("Close", ImVec2(96.0f, 34.0f))) {
          mixer_popup_open = false;
          ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
      }
      ImGui::PopStyleColor(2);
    }
    if (load_requested) {
      std::filesystem::path preview_path;
      if (setup_bundle->HasBundle() && setup_bundle->GeneratePreview(preview_path)) {
        std::array<char, 8192> diagnostic{};
        mjModel* next = mj_loadXML(preview_path.string().c_str(), nullptr, diagnostic.data(), diagnostic.size());
        mjData* next_data = next ? mj_makeData(next) : nullptr;
        if (next_data) {
          if (setup_model) {
            mjr_freeContext(&setup_context); mjv_freeScene(&setup_scene);
            mj_deleteData(setup_data); mj_deleteModel(setup_model);
          }
          setup_model = next; setup_data = next_data;
#if defined(COMBATSIM_USE_FILAMENT)
          apply_filament_material_defaults(setup_model);
#endif
          mj_forward(setup_model, setup_data);
          setup_model->vis.quality.offsamples = 0;
          mjv_makeScene(setup_model, &setup_scene, 2000);
          make_render_context(setup_model, &setup_context);
          const auto center = setup_bundle->PreviewCenter();
          for (int i = 0; i < 3; ++i) setup_camera.lookat[i] = center[i];
          setup_camera.distance = setup_bundle->PreviewDistance();
          runtime_error.clear();
        } else {
          runtime_error = next ? "Could not allocate CAD setup model." : diagnostic.data();
          if (next) mj_deleteModel(next);
        }
      } else if (setup_bundle->HasBundle()) runtime_error = setup_bundle->Error();
    }
    if (apply_requested) {
      if (lan_bundle_swapped)
        runtime_error = "End the LAN match before changing the shared bot model.";
      else
        rebuild_configured_model(false);
    }
    ImGui::Render(); ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    SDL_GL_SwapWindow(window);
  }

  if (!save_mixer_settings(input_mixer, active_mixer_profile_key))
    std::fprintf(stderr, "Could not save the active model's USB mixer settings on exit.\n");
  if (pad) SDL_CloseGamepad(pad);
  if (joystick) SDL_CloseJoystick(joystick);
  ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplSDL3_Shutdown(); ImGui::DestroyContext();
  glDeleteTextures(1, &preview_texture);
#if defined(COMBATSIM_USE_FILAMENT)
  glDeleteTextures(1, &main_scene_texture);
#endif
  if (setup_model) {
    mjr_freeContext(&setup_context); mjv_freeScene(&setup_scene);
    mj_deleteData(setup_data); mj_deleteModel(setup_model);
  }
  mjr_freeContext(&context); mjv_freeScene(&scene);
  SDL_GL_DestroyContext(gl); SDL_DestroyWindow(window); SDL_Quit();
  mj_deleteData(data); mj_deleteModel(model);
  return 0;
}


