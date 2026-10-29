#include "FusionBundle.h"

#include "imgui.h"
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <locale>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>

namespace {
using json = nlohmann::json;
using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>;
using Mat3 = std::array<double, 9>;

bool RefreshPrintEstimate(CadPart& part);

void Wipe(std::vector<unsigned char>& bytes) {
  volatile unsigned char* data = bytes.empty() ? nullptr : bytes.data();
  for (size_t i = 0; i < bytes.size(); ++i) data[i] = 0;
  bytes.clear();
  bytes.shrink_to_fit();
}

void Wipe(std::string& value) {
  volatile char* data = value.empty() ? nullptr : value.data();
  for (size_t i = 0; i < value.size(); ++i) data[i] = 0;
  value.clear();
  value.shrink_to_fit();
}

void WipeParts(std::vector<CadPart>& parts) {
  for (auto& part : parts) {
    Wipe(part.mesh_data);
    Wipe(part.render_mesh_data);
    for (auto& texture : part.textures) Wipe(texture.data);
  }
  parts.clear();
}

void WipeMeshes(std::map<std::string, std::vector<unsigned char>>& meshes) {
  for (auto& [name, bytes] : meshes) Wipe(bytes);
  meshes.clear();
}

std::filesystem::path NativePath(const std::filesystem::path& value) {
#if defined(_WIN32)
  const auto original = value.native();
  if (original.rfind(L"\\\\?\\", 0) == 0) return value.lexically_normal();
  auto absolute = std::filesystem::absolute(value).lexically_normal();
  const auto native = absolute.native();
  if (native.rfind(L"\\\\?\\", 0) == 0) return absolute;
  if (native.rfind(L"\\\\", 0) == 0)
    return std::filesystem::path(L"\\\\?\\UNC\\" + native.substr(2));
  return std::filesystem::path(L"\\\\?\\" + native);
#else
  return value;
#endif
}

std::string AssetPath(const std::filesystem::path& value) {
#if defined(_WIN32)
  return value.u8string();
#else
  return value.generic_string();
#endif
}

std::string AssetReference(const std::filesystem::path& value,
                           const std::filesystem::path& bundle_root) {
  const auto relative = value.lexically_relative(bundle_root);
  if (relative.empty() || *relative.begin() == "..") return AssetPath(value);
  return relative.generic_u8string();
}

bool IsBundleAssetPath(const std::filesystem::path& value) {
  if (value.empty() || value.is_absolute() || value.has_root_name() ||
      value.has_root_directory()) return false;
  const auto normalized = value.lexically_normal();
  if (normalized.empty() || *normalized.begin() == "..") return false;
  std::string extension = normalized.extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return extension == ".stl" || extension == ".obj" || extension == ".png" ||
         extension == ".jpg" || extension == ".jpeg";
}

bool IsTextureRole(const std::string& role) {
  return role == "rgb" || role == "normal" || role == "roughness";
}

bool HasTransparentAppearanceHint(const CadPart& part) {
  std::string label = part.appearance + " " + part.material;
  std::transform(label.begin(), label.end(), label.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return label.find("transparent") != std::string::npos ||
         label.find("translucent") != std::string::npos ||
         label.find("glass") != std::string::npos ||
         label.find("clear") != std::string::npos;
}

void RepairInvisibleOpaquePartColor(CadPart& part) {
  if (part.color[3] <= 0.0 && part.mass > 0.0 &&
      !HasTransparentAppearanceHint(part))
    part.color[3] = 1.0;
}

const CadPart::Texture* FindTexture(const CadPart& part, const std::string& role) {
  const auto found = std::find_if(part.textures.begin(), part.textures.end(),
    [&](const CadPart::Texture& texture) { return texture.role == role; });
  return found == part.textures.end() ? nullptr : &*found;
}

std::vector<std::string> ManifestAssetPaths(const json& parts) {
  std::set<std::string> unique;
  for (const auto& item : parts) {
    unique.insert(item.at("file").get<std::string>());
    if (item.contains("renderFile") && item.at("renderFile").is_string())
      unique.insert(item.at("renderFile").get<std::string>());
    if (item.contains("textures") && item.at("textures").is_object()) {
      for (auto it = item.at("textures").begin(); it != item.at("textures").end(); ++it)
        if (it.value().is_string()) unique.insert(it.value().get<std::string>());
    }
  }
  return {unique.begin(), unique.end()};
}

std::string MeshCacheId(const std::filesystem::path& manifest) {
  std::ostringstream bundle_id;
  bundle_id << std::hex << std::hash<std::string>{}(manifest.u8string());
  return bundle_id.str();
}

std::filesystem::path ShortMeshCachePath(const std::filesystem::path& cache_root,
                                         size_t index) {
  return cache_root / ("mesh_" + std::to_string(index) + ".stl");
}

Vec3 Add(Vec3 a, Vec3 b) { return {a[0]+b[0], a[1]+b[1], a[2]+b[2]}; }
Vec3 Sub(Vec3 a, Vec3 b) { return {a[0]-b[0], a[1]-b[1], a[2]-b[2]}; }
Vec3 Mul(Vec3 a, double s) { return {a[0]*s, a[1]*s, a[2]*s}; }
double Length(Vec3 a) { return std::sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]); }
double Dot(Vec3 a, Vec3 b) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
Vec3 Cross(Vec3 a, Vec3 b) {
  return {a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]};
}
bool Solve3(double a[3][4], double result[3]) {
  for (int col = 0; col < 3; ++col) {
    int pivot = col;
    for (int row = col+1; row < 3; ++row)
      if (std::abs(a[row][col]) > std::abs(a[pivot][col])) pivot = row;
    if (std::abs(a[pivot][col]) < 1e-10) return false;
    for (int k = col; k < 4; ++k) std::swap(a[pivot][k], a[col][k]);
    const double divisor = a[col][col];
    for (int k = col; k < 4; ++k) a[col][k] /= divisor;
    for (int row = 0; row < 3; ++row) if (row != col) {
      const double factor = a[row][col];
      for (int k = col; k < 4; ++k) a[row][k] -= factor*a[col][k];
    }
  }
  for (int i = 0; i < 3; ++i) result[i] = a[i][3];
  return true;
}
Vec3 Rotate(Quat q, Vec3 v) {
  const Vec3 u{q[1], q[2], q[3]};
  const Vec3 cross{u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]};
  const Vec3 twice = Mul(cross, 2.0);
  const Vec3 cross2{u[1]*twice[2]-u[2]*twice[1], u[2]*twice[0]-u[0]*twice[2], u[0]*twice[1]-u[1]*twice[0]};
  return Add(Add(v, Mul(twice, q[0])), cross2);
}
template <size_t N> std::array<double, N> ReadArray(const json& node, const char* key,
                                                     std::array<double, N> fallback) {
  if (!node.contains(key) || !node.at(key).is_array() || node.at(key).size() != N) return fallback;
  for (size_t i = 0; i < N; ++i) fallback[i] = node.at(key).at(i).get<double>();
  return fallback;
}
std::string Escape(const std::string& s) {
  std::string out;
  for (char c : s) {
    switch (c) {
      case '&': out += "&amp;"; break;
      case '"': out += "&quot;"; break;
      case '<': out += "&lt;"; break;
      case '>': out += "&gt;"; break;
      default: out += c;
    }
  }
  return out;
}
std::string Text(Vec3 v) {
  std::ostringstream s; s.imbue(std::locale::classic()); s << std::setprecision(12);
  s << v[0] << ' ' << v[1] << ' ' << v[2]; return s.str();
}
std::string Text4(std::array<double, 4> v) {
  std::ostringstream s; s.imbue(std::locale::classic()); s << std::setprecision(12);
  s << v[0] << ' ' << v[1] << ' ' << v[2] << ' ' << v[3]; return s.str();
}
std::string Number(double value) {
  std::ostringstream s; s.imbue(std::locale::classic()); s << std::setprecision(12) << value;
  return s.str();
}
std::string FormatGearRatio(double ratio) {
  if (!std::isfinite(ratio) || ratio <= 0.0) return "1:1";
  if (ratio > 1e6) return Number(ratio) + ":1";
  for (int denominator = 1; denominator <= 64; ++denominator) {
    const int numerator = static_cast<int>(std::round(ratio * denominator));
    if (numerator > 0 && std::abs(ratio -
        static_cast<double>(numerator) / denominator) <= 1e-7 * std::max(1.0, ratio))
      return std::to_string(numerator) + ":" + std::to_string(denominator);
  }
  return Number(ratio) + ":1";
}
bool ParseGearRatio(const std::string& text, double& ratio) {
  const size_t separator = text.find(':');
  if (separator == std::string::npos || text.find(':', separator + 1) != std::string::npos)
    return false;
  auto parse_positive = [](std::string value, double& parsed) {
    const size_t begin = value.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) return false;
    const size_t end = value.find_last_not_of(" \t\r\n");
    value = value.substr(begin, end - begin + 1);
    try {
      size_t consumed = 0;
      parsed = std::stod(value, &consumed);
      return consumed == value.size() && std::isfinite(parsed) && parsed > 0.0;
    } catch (...) { return false; }
  };
  double input = 0.0, output = 0.0;
  if (!parse_positive(text.substr(0, separator), input) ||
      !parse_positive(text.substr(separator + 1), output)) return false;
  ratio = input / output;
  return std::isfinite(ratio) && ratio > 0.0;
}
void DrawGearRatioInput(int key, MotorSpec& motor) {
  static std::map<int, std::array<char, 64>> buffers;
  static std::map<int, std::string> last_valid_text;
  auto& buffer = buffers[key];
  auto& last_valid = last_valid_text[key];
  if (last_valid != motor.gear_ratio_text) {
    std::snprintf(buffer.data(), buffer.size(), "%s", motor.gear_ratio_text.c_str());
    last_valid = motor.gear_ratio_text;
  }
  ImGui::TextWrapped("Gear ratio (input:output)");
  ImGui::SetNextItemWidth(std::min(240.0f, ImGui::GetContentRegionAvail().x));
  ImGui::InputText("##gear_ratio", buffer.data(), buffer.size());
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("Enter motor input RPM : output RPM, for example 16:1 or 3:2.");
  double ratio = 0.0;
  if (ParseGearRatio(buffer.data(), ratio)) {
    motor.gear_ratio = ratio;
    motor.gear_ratio_text = buffer.data();
    last_valid = motor.gear_ratio_text;
  } else {
    ImGui::TextColored(ImVec4(1.00f, 0.42f, 0.44f, 1.0f),
                       "Use positive input:output values, such as 16:1 or 3:2.");
  }
}
void DrawMotorIntInput(const char* label, int* value) {
  ImGui::TextWrapped("%s", label);
  ImGui::SetNextItemWidth(std::min(180.0f, ImGui::GetContentRegionAvail().x));
  const std::string id = std::string("##") + label;
  ImGui::InputInt(id.c_str(), value);
}
void DrawMotorDoubleInput(const char* label, double* value, double step,
                          double step_fast, const char* format) {
  ImGui::TextWrapped("%s", label);
  ImGui::SetNextItemWidth(std::min(180.0f, ImGui::GetContentRegionAvail().x));
  const std::string id = std::string("##") + label;
  ImGui::InputDouble(id.c_str(), value, step, step_fast, format);
}

const char* MassModeLabel(PartMassMode mode) {
  switch (mode) {
    case PartMassMode::Fusion: return "Fusion material mass";
    case PartMassMode::Manual: return "Manual weight";
    case PartMassMode::Printed: return "3D print estimate";
  }
  return "Fusion material mass";
}

const char* InfillPatternLabel(PrintInfillPattern pattern) {
  switch (pattern) {
    case PrintInfillPattern::Grid: return "Grid";
    case PrintInfillPattern::Rectilinear: return "Rectilinear";
    case PrintInfillPattern::Triangles: return "Triangles";
    case PrintInfillPattern::Gyroid: return "Gyroid";
    case PrintInfillPattern::Honeycomb: return "Honeycomb";
    case PrintInfillPattern::Cubic: return "Cubic";
    case PrintInfillPattern::Lightning: return "Lightning";
  }
  return "Grid";
}

const char* MassModeKey(PartMassMode mode) {
  switch (mode) {
    case PartMassMode::Fusion: return "fusion";
    case PartMassMode::Manual: return "manual";
    case PartMassMode::Printed: return "printed";
  }
  return "fusion";
}

PartMassMode ParseMassMode(const std::string& value) {
  if (value == "manual") return PartMassMode::Manual;
  if (value == "printed") return PartMassMode::Printed;
  return PartMassMode::Fusion;
}

json PrintProfileJson(const PrintProfile& profile) {
  return {{"name", profile.name},
          {"layerHeightMm", profile.layer_height_mm},
          {"firstLayerHeightMm", profile.first_layer_height_mm},
          {"lineWidthMm", profile.line_width_mm},
          {"wallLoops", profile.wall_loops},
          {"topLayers", profile.top_layers},
          {"bottomLayers", profile.bottom_layers},
          {"topSurfaceDensityPercent", profile.top_surface_density_percent},
          {"bottomSurfaceDensityPercent", profile.bottom_surface_density_percent},
          {"infillDensityPercent", profile.infill_density_percent},
          {"infillPattern", InfillPatternLabel(profile.infill_pattern)},
          {"infillUsageMultiplier", profile.infill_usage_multiplier},
          {"flowRatioPercent", profile.flow_ratio_percent}};
}

PrintProfile ReadPrintProfile(const json& node, PrintProfile fallback = {}) {
  if (!node.is_object()) return fallback;
  fallback.name = node.value("name", fallback.name);
  fallback.layer_height_mm = node.value("layerHeightMm", fallback.layer_height_mm);
  fallback.first_layer_height_mm = node.value(
    "firstLayerHeightMm", fallback.first_layer_height_mm);
  fallback.line_width_mm = node.value("lineWidthMm", fallback.line_width_mm);
  fallback.wall_loops = node.value("wallLoops", fallback.wall_loops);
  fallback.top_layers = node.value("topLayers", fallback.top_layers);
  fallback.bottom_layers = node.value("bottomLayers", fallback.bottom_layers);
  fallback.top_surface_density_percent = node.value(
    "topSurfaceDensityPercent", fallback.top_surface_density_percent);
  fallback.bottom_surface_density_percent = node.value(
    "bottomSurfaceDensityPercent", fallback.bottom_surface_density_percent);
  fallback.infill_density_percent = node.value(
    "infillDensityPercent", fallback.infill_density_percent);
  fallback.infill_usage_multiplier = node.value(
    "infillUsageMultiplier", fallback.infill_usage_multiplier);
  fallback.flow_ratio_percent = node.value("flowRatioPercent", fallback.flow_ratio_percent);
  const std::string pattern = node.value("infillPattern", std::string("Grid"));
  if (pattern == "Rectilinear") fallback.infill_pattern = PrintInfillPattern::Rectilinear;
  else if (pattern == "Triangles") fallback.infill_pattern = PrintInfillPattern::Triangles;
  else if (pattern == "Gyroid") fallback.infill_pattern = PrintInfillPattern::Gyroid;
  else if (pattern == "Honeycomb") fallback.infill_pattern = PrintInfillPattern::Honeycomb;
  else if (pattern == "Cubic") fallback.infill_pattern = PrintInfillPattern::Cubic;
  else if (pattern == "Lightning") fallback.infill_pattern = PrintInfillPattern::Lightning;
  else fallback.infill_pattern = PrintInfillPattern::Grid;
  fallback.layer_height_mm = std::clamp(fallback.layer_height_mm, 0.04, 2.0);
  fallback.first_layer_height_mm = std::clamp(fallback.first_layer_height_mm, 0.04, 2.0);
  fallback.line_width_mm = std::clamp(fallback.line_width_mm, 0.1, 4.0);
  fallback.wall_loops = std::clamp(fallback.wall_loops, 0, 20);
  fallback.top_layers = std::clamp(fallback.top_layers, 0, 100);
  fallback.bottom_layers = std::clamp(fallback.bottom_layers, 0, 100);
  fallback.top_surface_density_percent = std::clamp(
    fallback.top_surface_density_percent, 0.0, 100.0);
  fallback.bottom_surface_density_percent = std::clamp(
    fallback.bottom_surface_density_percent, 0.0, 100.0);
  fallback.infill_density_percent = std::clamp(fallback.infill_density_percent, 0.0, 100.0);
  fallback.infill_usage_multiplier = std::clamp(fallback.infill_usage_multiplier, 0.1, 3.0);
  fallback.flow_ratio_percent = std::clamp(fallback.flow_ratio_percent, 50.0, 150.0);
  return fallback;
}

std::filesystem::path PrintProfileLibraryPath() {
  char* preference_path = SDL_GetPrefPath("CombatSim", "CombatSim");
  if (!preference_path) return {};
  const std::filesystem::path result =
    std::filesystem::path(preference_path) / "print_profiles.json";
  SDL_free(preference_path);
  return result;
}

std::vector<PrintProfile>& PrintProfiles() {
  static std::vector<PrintProfile> profiles;
  static bool loaded = false;
  if (loaded) return profiles;
  loaded = true;
  profiles.push_back(PrintProfile{});
  try {
    const auto path = PrintProfileLibraryPath();
    std::ifstream input(path);
    if (!input) return profiles;
    const json saved = json::parse(input);
    if (!saved.contains("profiles") || !saved.at("profiles").is_array()) return profiles;
    std::vector<PrintProfile> parsed;
    for (const auto& node : saved.at("profiles")) {
      PrintProfile profile = ReadPrintProfile(node);
      if (!profile.name.empty()) parsed.push_back(std::move(profile));
    }
    if (!parsed.empty()) profiles = std::move(parsed);
  } catch (...) {
    // Keep the built-in PLA profile if the settings file is absent or damaged.
  }
  return profiles;
}

bool SavePrintProfiles(std::string* error = nullptr) {
  try {
    const auto path = PrintProfileLibraryPath();
    if (path.empty()) throw std::runtime_error("Could not locate the CombatSim preferences folder.");
    std::filesystem::create_directories(path.parent_path());
    json saved;
    saved["schemaVersion"] = 1;
    saved["profiles"] = json::array();
    for (const auto& profile : PrintProfiles()) saved["profiles"].push_back(PrintProfileJson(profile));
    std::ofstream output(path, std::ios::trunc);
    if (!output) throw std::runtime_error("Could not save print profiles.");
    output << saved.dump(2);
    if (!output) throw std::runtime_error("Failed writing print profiles.");
    return true;
  } catch (const std::exception& e) {
    if (error) *error = e.what();
    return false;
  }
}

PrintProfile* FindPrintProfile(const std::string& name) {
  auto& profiles = PrintProfiles();
  const auto found = std::find_if(profiles.begin(), profiles.end(),
    [&](const PrintProfile& profile) { return profile.name == name; });
  return found == profiles.end() ? nullptr : &*found;
}

bool DrawProfileDouble(const char* label, double* value, double step,
                       const char* format) {
  ImGui::TextUnformatted(label);
  ImGui::SetNextItemWidth(std::min(180.0f, ImGui::GetContentRegionAvail().x));
  return ImGui::InputDouble((std::string("##") + label).c_str(), value, step,
                            step * 10.0, format);
}

bool DrawProfileInt(const char* label, int* value) {
  ImGui::TextUnformatted(label);
  ImGui::SetNextItemWidth(std::min(180.0f, ImGui::GetContentRegionAvail().x));
  return ImGui::InputInt((std::string("##") + label).c_str(), value);
}

void DrawPrintProfileManager() {
  if (!ImGui::CollapsingHeader("3D print profile library")) return;
  ImGui::PushID("print_profile_manager");
  auto& profiles = PrintProfiles();
  static int selected = 0;
  static std::array<char, 128> name_buffer{};
  static std::string last_name;
  static std::string save_message;
  if (profiles.empty()) profiles.push_back(PrintProfile{});
  selected = std::clamp(selected, 0, int(profiles.size()) - 1);
  if (ImGui::BeginCombo("Profile", profiles[selected].name.c_str())) {
    for (size_t index = 0; index < profiles.size(); ++index)
      if (ImGui::Selectable(profiles[index].name.c_str(), selected == int(index))) {
        selected = int(index);
        last_name.clear();
      }
    ImGui::EndCombo();
  }
  if (ImGui::Button("New profile")) {
    PrintProfile profile;
    profile.name = "New print profile";
    int suffix = 2;
    while (FindPrintProfile(profile.name))
      profile.name = "New print profile " + std::to_string(suffix++);
    profiles.push_back(profile);
    selected = int(profiles.size()) - 1;
    last_name.clear();
    SavePrintProfiles(&save_message);
  }
  ImGui::SameLine();
  if (ImGui::Button("Duplicate profile")) {
    PrintProfile duplicate = profiles[selected];
    duplicate.name += " copy";
    int suffix = 2;
    const std::string base = duplicate.name;
    while (FindPrintProfile(duplicate.name))
      duplicate.name = base + " " + std::to_string(suffix++);
    profiles.push_back(duplicate);
    selected = int(profiles.size()) - 1;
    last_name.clear();
    SavePrintProfiles(&save_message);
  }
  ImGui::SameLine();
  if (ImGui::Button("Delete profile") && profiles.size() > 1) {
    profiles.erase(profiles.begin() + selected);
    selected = std::clamp(selected, 0, int(profiles.size()) - 1);
    last_name.clear();
    SavePrintProfiles(&save_message);
  }
  PrintProfile& profile = profiles[selected];
  if (last_name != profile.name) {
    std::snprintf(name_buffer.data(), name_buffer.size(), "%s", profile.name.c_str());
    last_name = profile.name;
  }
  bool changed = false;
  if (ImGui::InputText("Profile name", name_buffer.data(), name_buffer.size())) {
    const std::string edited = name_buffer.data();
    if (!edited.empty() && edited != profile.name) {
      const bool duplicate = std::any_of(profiles.begin(), profiles.end(),
        [&](const PrintProfile& other) {
          return &other != &profile && other.name == edited;
        });
      if (!duplicate) {
        profile.name = edited;
        changed = true;
      } else {
        save_message = "Profile names must be unique.";
      }
    }
  }
  changed |= DrawProfileDouble("Layer height (mm)", &profile.layer_height_mm,
    0.01, "%.3f");
  changed |= DrawProfileDouble("First layer height (mm)",
    &profile.first_layer_height_mm, 0.01, "%.3f");
  changed |= DrawProfileDouble("Line width (mm)", &profile.line_width_mm,
    0.01, "%.3f");
  changed |= DrawProfileInt("Wall loops", &profile.wall_loops);
  changed |= DrawProfileInt("Top solid layers", &profile.top_layers);
  changed |= DrawProfileInt("Bottom solid layers", &profile.bottom_layers);
  changed |= DrawProfileDouble("Top surface density (%)",
    &profile.top_surface_density_percent, 1.0, "%.1f");
  changed |= DrawProfileDouble("Bottom surface density (%)",
    &profile.bottom_surface_density_percent, 1.0, "%.1f");
  changed |= DrawProfileDouble("Sparse infill density (%)",
    &profile.infill_density_percent, 1.0, "%.1f");
  const char* pattern_label = InfillPatternLabel(profile.infill_pattern);
  if (ImGui::BeginCombo("Infill pattern", pattern_label)) {
    for (const auto pattern : {PrintInfillPattern::Grid,
         PrintInfillPattern::Rectilinear, PrintInfillPattern::Triangles,
         PrintInfillPattern::Gyroid, PrintInfillPattern::Honeycomb,
         PrintInfillPattern::Cubic, PrintInfillPattern::Lightning})
      if (ImGui::Selectable(InfillPatternLabel(pattern),
                            pattern == profile.infill_pattern)) {
        profile.infill_pattern = pattern;
        changed = true;
      }
    ImGui::EndCombo();
  }
  double usage_percent = profile.infill_usage_multiplier * 100.0;
  if (DrawProfileDouble("Infill usage correction (%)", &usage_percent,
                        1.0, "%.0f")) {
    profile.infill_usage_multiplier = usage_percent / 100.0;
    changed = true;
  }
  changed |= DrawProfileDouble("Extrusion flow ratio (%)",
    &profile.flow_ratio_percent, 1.0, "%.1f");
  profile.layer_height_mm = std::clamp(profile.layer_height_mm, 0.04, 2.0);
  profile.first_layer_height_mm = std::clamp(profile.first_layer_height_mm, 0.04, 2.0);
  profile.line_width_mm = std::clamp(profile.line_width_mm, 0.1, 4.0);
  profile.wall_loops = std::clamp(profile.wall_loops, 0, 20);
  profile.top_layers = std::clamp(profile.top_layers, 0, 100);
  profile.bottom_layers = std::clamp(profile.bottom_layers, 0, 100);
  profile.top_surface_density_percent = std::clamp(
    profile.top_surface_density_percent, 0.0, 100.0);
  profile.bottom_surface_density_percent = std::clamp(
    profile.bottom_surface_density_percent, 0.0, 100.0);
  profile.infill_density_percent = std::clamp(profile.infill_density_percent, 0.0, 100.0);
  profile.infill_usage_multiplier = std::clamp(profile.infill_usage_multiplier, 0.1, 3.0);
  profile.flow_ratio_percent = std::clamp(profile.flow_ratio_percent, 50.0, 150.0);
  if (changed) last_name = profile.name;
  if (ImGui::Button("Save print profile")) {
    if (SavePrintProfiles(&save_message)) save_message = "Profile saved.";
  }
  ImGui::SameLine();
  ImGui::TextDisabled("Shared by both bots. Editing a preset does not change a part until you apply it.");
  ImGui::TextDisabled("Material density comes from each body's Fusion export; presets change print structure only.");
  if (!save_message.empty()) ImGui::TextWrapped("%s", save_message.c_str());
  ImGui::TextWrapped("Printed mass uses an area-based shell estimate plus sparse infill. Pattern multipliers are approximate; the CAD collision surface stays unchanged.");
  ImGui::PopID();
}
const char* KindLabel(AssemblyKind kind) {
  switch (kind) {
    case AssemblyKind::Frame: return "Frame";
    case AssemblyKind::Fixed: return "Fixed";
    case AssemblyKind::LeftWheel: return "Left wheel";
    case AssemblyKind::RightWheel: return "Right wheel";
    case AssemblyKind::Weapon: return "Weapon";
    case AssemblyKind::GravityFork: return "Gravity fork";
  }
  return "Fixed";
}
AssemblyKind ParseKind(const std::string& kind) {
  if (kind == "Left wheel") return AssemblyKind::LeftWheel;
  if (kind == "Right wheel") return AssemblyKind::RightWheel;
  if (kind == "Weapon") return AssemblyKind::Weapon;
  if (kind == "Gravity fork") return AssemblyKind::GravityFork;
  return AssemblyKind::Fixed;
}
bool IsWheelKind(AssemblyKind kind) {
  return kind == AssemblyKind::LeftWheel || kind == AssemblyKind::RightWheel;
}
const char* MotorTypeLabel(MotorType type) {
  return type == MotorType::Brushed ? "Brushed DC" : "Brushless (BLDC)";
}
MotorType ParseMotorType(const std::string& type) {
  return type == "brushed" ? MotorType::Brushed : MotorType::Brushless;
}
const char* MotorTypeKey(MotorType type) {
  return type == MotorType::Brushed ? "brushed" : "brushless";
}
const char* MotorSpeedInputLabel(MotorSpeedInput input) {
  return input == MotorSpeedInput::RatedRpmAtVoltage ? "Rated RPM at voltage" : "Kv (RPM/V)";
}
MotorSpeedInput ParseMotorSpeedInput(const std::string& input) {
  return input == "ratedRpmAtVoltage" ? MotorSpeedInput::RatedRpmAtVoltage : MotorSpeedInput::Kv;
}
const char* MotorSpeedInputKey(MotorSpeedInput input) {
  return input == MotorSpeedInput::RatedRpmAtVoltage ? "ratedRpmAtVoltage" : "kv";
}
void DrawMotorSpeedInputs(int id, MotorSpec& motor) {
  ImGui::PushID(id);
  if (ImGui::BeginCombo("Motor type", MotorTypeLabel(motor.type))) {
    for (const auto type : {MotorType::Brushless, MotorType::Brushed})
      if (ImGui::Selectable(MotorTypeLabel(type), motor.type == type)) motor.type = type;
    ImGui::EndCombo();
  }
  if (ImGui::BeginCombo("Speed specification", MotorSpeedInputLabel(motor.speed_input))) {
    for (const auto input : {MotorSpeedInput::Kv, MotorSpeedInput::RatedRpmAtVoltage})
      if (ImGui::Selectable(MotorSpeedInputLabel(input), motor.speed_input == input))
        motor.speed_input = input;
    ImGui::EndCombo();
  }
  if (motor.speed_input == MotorSpeedInput::RatedRpmAtVoltage) {
    DrawMotorDoubleInput("Motor speed (RPM)", &motor.rated_rpm, 10.0, 100.0, "%.0f");
    DrawMotorDoubleInput("Speed rating voltage (V)", &motor.rated_voltage,
                         0.1, 1.0, "%.2f");
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("Enter the motor's published speed and its test voltage. The simulator treats this as no-load speed and scales it with pack voltage.");
  } else {
    DrawMotorDoubleInput("Motor Kv (RPM/V)", &motor.kv_rpm_per_volt,
                         1.0, 10.0, "%.1f");
  }
  ImGui::PopID();
}
const char* MotorResistanceLabel(MotorType type) {
  return type == MotorType::Brushed ? "Armature resistance (mOhm)" :
    "Phase-to-phase resistance (mOhm)";
}
const char* MotorCurrentLimitLabel(MotorType type) {
  return type == MotorType::Brushed ? "Motor controller current limit per motor (A)" :
    "ESC phase-current limit per motor (A)";
}
std::string Lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}
struct ClassicSurface {
  double specular;
  double shininess;
};
ClassicSurface SurfaceForPart(const CadPart& part) {
  const std::string label = Lower(part.material + " " + part.appearance);
  ClassicSurface surface{0.38, 0.28};
  auto has = [&](const char* token) { return label.find(token) != std::string::npos; };
  if (has("silicone") || has("tpu") || has("rubber") || has("elastomer")) {
    surface = {0.18, 0.12};
  } else if (has("carbon") || has("fiber")) {
    surface = {0.56, 0.42};
  } else if (has("steel") || has("aluminum") || has("aluminium") || has("metal") ||
             has("titanium") || has("copper") || has("brass") || has("bronze") ||
             has("iron") || has("7075")) {
    surface = {0.72, 0.58};
  } else if (has("uhmw") || has("polyethylene") || has("nylon") || has("pa12") ||
             has("abs") || has("plastic") || has("polymer") || has("acetal") ||
             has("delrin") || has("polycarbonate")) {
    surface = {0.34, 0.24};
  } else if (has("wood") || has("mdf") || has("chipboard")) {
    surface = {0.22, 0.14};
  }

  // Fusion's normalized surface roughness is useful to shape the classic
  // renderer's Phong highlight, even though that renderer is not PBR.
  if (std::isfinite(part.roughness) && part.roughness >= 0.0 && part.roughness <= 1.0) {
    surface.specular *= 1.0 - 0.55 * part.roughness;
    surface.shininess = std::clamp(0.88 - 0.82 * part.roughness, 0.06, 0.88);
  }
  return surface;
}

#if defined(COMBATSIM_USE_FILAMENT)
struct PbrSurface {
  double metallic = 0.0;
  double roughness = 0.62;
};

PbrSurface PbrSurfaceForPart(const CadPart& part) {
  const std::string label = Lower(part.material + " " + part.appearance);
  PbrSurface surface;
  auto has = [&](const char* token) { return label.find(token) != std::string::npos; };
  if (has("steel") || has("aluminum") || has("aluminium") || has("metal") ||
      has("titanium") || has("copper") || has("brass") || has("bronze") ||
      has("iron") || has("7075")) {
    surface = {0.82, 0.28};
  } else if (has("silicone") || has("tpu") || has("rubber") || has("elastomer")) {
    surface = {0.0, 0.82};
  } else if (has("carbon") || has("fiber")) {
    surface = {0.0, 0.48};
  } else if (has("uhmw") || has("polyethylene") || has("nylon") || has("pa12") ||
             has("abs") || has("plastic") || has("polymer") || has("acetal") ||
             has("delrin") || has("polycarbonate")) {
    surface = {0.0, 0.38};
  } else if (has("wood") || has("mdf") || has("chipboard")) {
    surface = {0.0, 0.78};
  }
  if (std::isfinite(part.roughness) && part.roughness >= 0.0 && part.roughness <= 1.0)
    surface.roughness = part.roughness;
  return surface;
}
#endif

void Include(Vec3 point, Vec3& low, Vec3& high) {
  for (int i = 0; i < 3; ++i) { low[i] = std::min(low[i], point[i]); high[i] = std::max(high[i], point[i]); }
}
bool ValidMass(double value) { return std::isfinite(value) && value > 0.0; }
bool FlexiblePartPath(const CadPart& part) {
  auto has_flexible_name = [](const std::string& name) {
    std::string lower = name;
    std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    return lower.find("wire") != std::string::npos ||
      lower.find("cable") != std::string::npos;
  };
  if (has_flexible_name(part.name)) return true;
  return std::any_of(part.occurrence_path.begin(), part.occurrence_path.end(), has_flexible_name);
}
bool ForkPartPath(const CadPart& part) {
  auto has_fork_name = [](const std::string& name) {
    std::string lower = name;
    std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    return lower.find("fork") != std::string::npos;
  };
  if (has_fork_name(part.name)) return true;
  return std::any_of(part.occurrence_path.begin(), part.occurrence_path.end(), has_fork_name);
}
Mat3 UnpackTensor(const std::array<double, 6>& p) {
  return {p[0], p[3], p[4], p[3], p[1], p[5], p[4], p[5], p[2]};
}
bool ValidInertia(const std::array<double, 6>& p) {
  if (!std::all_of(p.begin(), p.end(), [](double value) { return std::isfinite(value); })) return false;
  // Cholesky test for a positive-definite symmetric inertia tensor.
  if (p[0] <= 0.0) return false;
  const double l00 = std::sqrt(p[0]);
  const double l10 = p[3] / l00, l20 = p[4] / l00;
  const double d11 = p[1] - l10*l10;
  if (d11 <= 0.0) return false;
  const double l11 = std::sqrt(d11);
  const double l21 = (p[5] - l20*l10) / l11;
  return p[2] - l20*l20 - l21*l21 > 0.0;
}
Mat3 RotateTensor(const Mat3& tensor, Quat q) {
  const Vec3 axes[3] = {Rotate(q, {1, 0, 0}), Rotate(q, {0, 1, 0}), Rotate(q, {0, 0, 1})};
  double rotation[3][3]{};
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 3; ++col) rotation[row][col] = axes[col][row];
  Mat3 result{};
  for (int row = 0; row < 3; ++row) for (int col = 0; col < 3; ++col)
    for (int k = 0; k < 3; ++k) for (int l = 0; l < 3; ++l)
      result[row*3+col] += rotation[row][k] * tensor[k*3+l] * rotation[col][l];
  return result;
}
Mat3 ParallelAxis(double mass, Vec3 offset) {
  Mat3 result{};
  const double distance2 = Dot(offset, offset);
  for (int row = 0; row < 3; ++row) for (int col = 0; col < 3; ++col)
    result[row*3+col] = mass * ((row == col ? distance2 : 0.0) - offset[row]*offset[col]);
  return result;
}
void AddTensor(Mat3& target, const Mat3& value) {
  for (size_t i = 0; i < target.size(); ++i) target[i] += value[i];
}
std::string TextInertia(const Mat3& value) {
  return Text({value[0], value[4], value[8]}) + " " +
         Number(value[1]) + " " + Number(value[2]) + " " + Number(value[5]);
}

void ReadStlBoundsStream(std::istream& file, const std::string& label,
                         double scale, Vec3 position, Quat rotation,
                         Vec3& low, Vec3& high) {
  file.seekg(0, std::ios::end);
  const auto bytes = file.tellg();
  file.seekg(80);
  uint32_t count = 0;
  file.read(reinterpret_cast<char*>(&count), 4);
  if (!file || count == 0 || bytes != std::streamoff(84) + std::streamoff(count) * 50)
    throw std::runtime_error("Expected a nonempty binary STL: " + label);
  low.fill(std::numeric_limits<double>::infinity());
  high.fill(-std::numeric_limits<double>::infinity());
  for (uint32_t t = 0; t < count; ++t) {
    char triangle[50];
    file.read(triangle, sizeof(triangle));
    if (!file) throw std::runtime_error("Truncated STL: " + label);
    for (int v = 0; v < 3; ++v) {
      float xyz[3];
      std::memcpy(xyz, triangle + 12 + v * 12, 12);
      Vec3 point = Add(position, Rotate(rotation, {xyz[0]*scale, xyz[1]*scale, xyz[2]*scale}));
      Include(point, low, high);
    }
  }
}

void ReadStlBounds(const std::filesystem::path& path, double scale, Vec3 position, Quat rotation,
                   Vec3& low, Vec3& high) {
  std::ifstream file(path, std::ios::binary);
  if (!file) throw std::runtime_error("Cannot read STL: " + path.string());
  ReadStlBoundsStream(file, path.string(), scale, position, rotation, low, high);
}

void ReadStlBounds(const std::vector<unsigned char>& bytes, const std::string& label,
                   double scale, Vec3 position, Quat rotation, Vec3& low, Vec3& high) {
  const std::string contents(reinterpret_cast<const char*>(bytes.data()), bytes.size());
  std::istringstream file(contents, std::ios::in | std::ios::binary);
  ReadStlBoundsStream(file, label, scale, position, rotation, low, high);
}

std::unique_ptr<std::istream> OpenMesh(const CadPart& part) {
  if (!part.mesh_data.empty()) {
    const std::string contents(reinterpret_cast<const char*>(part.mesh_data.data()),
                               part.mesh_data.size());
    return std::make_unique<std::istringstream>(contents,
                                                std::ios::in | std::ios::binary);
  }
  auto file = std::make_unique<std::ifstream>(part.absolute_file, std::ios::binary);
  if (!*file) return {};
  return file;
}

Mat3 OuterProduct(Vec3 a, Vec3 b) {
  return {a[0]*b[0], a[0]*b[1], a[0]*b[2],
          a[1]*b[0], a[1]*b[1], a[1]*b[2],
          a[2]*b[0], a[2]*b[1], a[2]*b[2]};
}

void AddScaled(Mat3& target, const Mat3& value, double scale) {
  for (size_t i = 0; i < target.size(); ++i) target[i] += value[i] * scale;
}

double Trace(const Mat3& value) { return value[0] + value[4] + value[8]; }

Mat3 InertiaFromSecondMoment(const Mat3& second) {
  const double trace = Trace(second);
  Mat3 inertia{};
  for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j)
    inertia[i*3+j] = (i == j ? trace : 0.0) - second[i*3+j];
  return inertia;
}

std::array<double, 6> PackTensor(const Mat3& value) {
  return {value[0], value[4], value[8], value[1], value[2], value[5]};
}

struct PrintTriangle {
  Vec3 a{}, b{}, c{}, normal{};
  double area = 0.0;
};

struct MeshVolumeProperties {
  double volume = 0.0;
  Vec3 center{};
  Mat3 second_origin{};
  Vec3 low{std::numeric_limits<double>::infinity(),
           std::numeric_limits<double>::infinity(),
           std::numeric_limits<double>::infinity()};
  Vec3 high{-std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity()};
  double signed_volume = 0.0;
};

bool ReadPrintMesh(const CadPart& part, std::vector<PrintTriangle>& triangles,
                   MeshVolumeProperties& properties, std::string& error) {
  auto file = OpenMesh(part);
  if (!file) { error = "Cannot read this part's STL mesh."; return false; }
  file->seekg(0, std::ios::end);
  const std::streamoff bytes = file->tellg();
  file->seekg(80);
  uint32_t count = 0;
  file->read(reinterpret_cast<char*>(&count), sizeof(count));
  if (!*file || count == 0 || bytes != std::streamoff(84) + std::streamoff(count) * 50) {
    error = "Print mass needs a nonempty binary STL for this part.";
    return false;
  }
  triangles.clear();
  triangles.reserve(count);
  Vec3 first_moment{};
  Mat3 second_origin{};
  for (uint32_t index = 0; index < count; ++index) {
    char bytes50[50];
    file->read(bytes50, sizeof(bytes50));
    if (!*file) { error = "The part's STL is truncated."; return false; }
    Vec3 vertex[3];
    for (int corner = 0; corner < 3; ++corner) {
      float xyz[3];
      std::memcpy(xyz, bytes50 + 12 + corner * 12, sizeof(xyz));
      vertex[corner] = {double(xyz[0]) * part.scale,
                        double(xyz[1]) * part.scale,
                        double(xyz[2]) * part.scale};
      Include(vertex[corner], properties.low, properties.high);
    }
    const Vec3 cross = Cross(Sub(vertex[1], vertex[0]), Sub(vertex[2], vertex[0]));
    const double cross_length = Length(cross);
    const double area = cross_length * 0.5;
    if (area <= 1e-16) continue;
    const Vec3 normal = Mul(cross, 1.0 / cross_length);
    triangles.push_back({vertex[0], vertex[1], vertex[2], normal, area});

    const double tetra_volume = Dot(vertex[0], Cross(vertex[1], vertex[2])) / 6.0;
    properties.signed_volume += tetra_volume;
    first_moment = Add(first_moment,
      Mul(Add(Add(vertex[0], vertex[1]), vertex[2]), tetra_volume / 4.0));
    Mat3 tetra_second = OuterProduct(vertex[0], vertex[0]);
    AddScaled(tetra_second, OuterProduct(vertex[1], vertex[1]), 1.0);
    AddScaled(tetra_second, OuterProduct(vertex[2], vertex[2]), 1.0);
    const Vec3 sum = Add(Add(vertex[0], vertex[1]), vertex[2]);
    AddScaled(tetra_second, OuterProduct(sum, sum), 1.0);
    AddScaled(second_origin, tetra_second, tetra_volume / 20.0);
  }
  if (triangles.empty()) { error = "The part's STL has no usable triangles."; return false; }
  const double orientation = properties.signed_volume < 0.0 ? -1.0 : 1.0;
  properties.volume = std::abs(properties.signed_volume);
  if (properties.volume > 1e-12) {
    properties.center = Mul(first_moment, orientation / properties.volume);
    for (double& value : second_origin) value *= orientation;
  }
  return true;
}

double InfillPatternFactor(PrintInfillPattern pattern) {
  // Small corrections approximate common pattern path overlap/turning. Actual
  // material usage depends on slicer version, line generator and geometry.
  switch (pattern) {
    case PrintInfillPattern::Grid: return 1.04;
    case PrintInfillPattern::Rectilinear: return 1.0;
    case PrintInfillPattern::Triangles: return 1.06;
    case PrintInfillPattern::Gyroid: return 1.03;
    case PrintInfillPattern::Honeycomb: return 1.10;
    case PrintInfillPattern::Cubic: return 1.04;
    case PrintInfillPattern::Lightning: return 0.62;
  }
  return 1.0;
}

bool EstimatePrintedMassProperties(const CadPart& part, const PrintProfile& profile,
                                   Vec3 build_axis, double& mass, Vec3& center,
                                   std::array<double, 6>& tensor, std::string& error) {
  const double axis_length = Length(build_axis);
  if (!std::isfinite(axis_length) || axis_length < 1e-8) {
    error = "Choose a valid bed face or build direction.";
    return false;
  }
  build_axis = Mul(build_axis, 1.0 / axis_length);
  std::vector<PrintTriangle> triangles;
  MeshVolumeProperties mesh;
  if (!ReadPrintMesh(part, triangles, mesh, error)) return false;

  double solid_volume = mesh.volume;
  Vec3 solid_center = mesh.center;
  Mat3 solid_second_origin = mesh.second_origin;
  if (!(solid_volume > 1e-12) && part.volume_m3 > 1e-12) {
    solid_volume = part.volume_m3;
    solid_center = part.has_fusion_center_of_mass ? part.center_of_mass_local :
      Mul(Add(mesh.low, mesh.high), 0.5);
    Mat3 inertia_center{};
    if (part.has_fusion_inertia_tensor && part.fusion_mass > 0.0) {
      inertia_center = UnpackTensor(part.fusion_inertia_tensor);
      for (double& value : inertia_center) value *= solid_volume / part.fusion_mass;
    } else {
      const Vec3 size = Sub(mesh.high, mesh.low);
      inertia_center[0] = solid_volume * (size[1]*size[1] + size[2]*size[2]) / 12.0;
      inertia_center[4] = solid_volume * (size[0]*size[0] + size[2]*size[2]) / 12.0;
      inertia_center[8] = solid_volume * (size[0]*size[0] + size[1]*size[1]) / 12.0;
    }
    const Mat3 second_center = [&] {
      Mat3 result{};
      const double trace = Trace(inertia_center);
      for (int row = 0; row < 3; ++row) for (int col = 0; col < 3; ++col)
        result[row*3+col] = (row == col ? 0.5 * trace : 0.0) - inertia_center[row*3+col];
      return result;
    }();
    solid_second_origin = second_center;
    AddScaled(solid_second_origin, OuterProduct(solid_center, solid_center), solid_volume);
  }
  if (!(solid_volume > 1e-12) || !std::isfinite(solid_volume)) {
    error = "Could not recover a closed solid volume from this STL; check the Fusion export.";
    return false;
  }
  double material_density_kg_m3 = part.density;
  if ((!std::isfinite(material_density_kg_m3) || material_density_kg_m3 <= 0.0) &&
      part.fusion_mass > 0.0) {
    // Older exporter bundles may contain Fusion mass and volume but omit the
    // explicit density field. Recover the assigned Fusion material density.
    const double fusion_volume = part.volume_m3 > 1e-12 ? part.volume_m3 : solid_volume;
    material_density_kg_m3 = part.fusion_mass / fusion_volume;
  }
  if (!std::isfinite(material_density_kg_m3) || material_density_kg_m3 <= 0.0) {
    error = "Fusion material density is missing. Assign a material with density in Fusion and re-export the body.";
    return false;
  }
  if (part.has_fusion_center_of_mass && part.volume_m3 > 1e-12 &&
      part.has_fusion_inertia_tensor && part.fusion_mass > 0.0) {
    solid_volume = part.volume_m3;
    solid_center = part.center_of_mass_local;
    Mat3 inertia_center = UnpackTensor(part.fusion_inertia_tensor);
    for (double& value : inertia_center) value *= solid_volume / part.fusion_mass;
    Mat3 second_center{};
    const double trace = Trace(inertia_center);
    for (int row = 0; row < 3; ++row) for (int col = 0; col < 3; ++col)
      second_center[row*3+col] = (row == col ? 0.5 * trace : 0.0) - inertia_center[row*3+col];
    solid_second_origin = second_center;
    AddScaled(solid_second_origin, OuterProduct(solid_center, solid_center), solid_volume);
  }

  const double wall_depth = profile.wall_loops * profile.line_width_mm * 0.001;
  const double top_depth = profile.top_layers * profile.layer_height_mm * 0.001;
  const double bottom_depth = profile.bottom_layers > 0 ?
    (profile.first_layer_height_mm + (profile.bottom_layers - 1) *
      profile.layer_height_mm) * 0.001 : 0.0;
  double shell_geometry_volume = 0.0;
  double shell_effective_volume = 0.0;
  Vec3 shell_geometry_first{}, shell_effective_first{};
  Mat3 shell_geometry_second{}, shell_effective_second{};
  const double orientation = mesh.signed_volume < 0.0 ? -1.0 : 1.0;
  for (const auto& triangle : triangles) {
    Vec3 normal = Mul(triangle.normal, orientation);
    const Vec3 face_center = Mul(Add(Add(triangle.a, triangle.b), triangle.c), 1.0 / 3.0);
    const double axis_dot = Dot(normal, build_axis);
    const double surface_fill = axis_dot > 0.70 ?
      profile.top_surface_density_percent / 100.0 :
      axis_dot < -0.70 ? profile.bottom_surface_density_percent / 100.0 : 1.0;
    const double thickness = axis_dot > 0.70 ? top_depth :
      axis_dot < -0.70 ? bottom_depth : wall_depth;
    if (thickness <= 0.0) continue;
    const double volume = triangle.area * thickness;
    const Vec3 inner_center = Sub(face_center, Mul(normal, thickness * 0.5));
    const Vec3 da = Sub(triangle.a, face_center);
    const Vec3 db = Sub(triangle.b, face_center);
    const Vec3 dc = Sub(triangle.c, face_center);
    Mat3 covariance{};
    AddScaled(covariance, OuterProduct(da, da), 1.0 / 12.0);
    AddScaled(covariance, OuterProduct(db, db), 1.0 / 12.0);
    AddScaled(covariance, OuterProduct(dc, dc), 1.0 / 12.0);
    AddScaled(covariance, OuterProduct(normal, normal), thickness*thickness / 12.0);
    shell_geometry_volume += volume;
    shell_effective_volume += volume * surface_fill;
    shell_geometry_first = Add(shell_geometry_first, Mul(inner_center, volume));
    shell_effective_first = Add(shell_effective_first,
      Mul(inner_center, volume * surface_fill));
    Mat3 triangle_second = OuterProduct(inner_center, inner_center);
    AddScaled(triangle_second, covariance, 1.0);
    AddScaled(shell_geometry_second, triangle_second, volume);
    AddScaled(shell_effective_second, triangle_second, volume * surface_fill);
  }

  // Area x shell thickness slightly overcounts corners and curved offsets.
  // Cap the approximation to leave a small physical core for infill.
  const double shell_cap = solid_volume * 0.95;
  const double shell_scale = shell_geometry_volume > shell_cap && shell_geometry_volume > 0.0 ?
    shell_cap / shell_geometry_volume : 1.0;
  shell_geometry_volume *= shell_scale;
  shell_effective_volume *= shell_scale;
  shell_geometry_first = Mul(shell_geometry_first, shell_scale);
  shell_effective_first = Mul(shell_effective_first, shell_scale);
  for (double& value : shell_geometry_second) value *= shell_scale;
  for (double& value : shell_effective_second) value *= shell_scale;

  const double core_volume = std::max(0.0, solid_volume - shell_geometry_volume);
  const double requested_density = profile.infill_density_percent / 100.0;
  const double infill_fraction = std::clamp(requested_density *
    profile.infill_usage_multiplier * InfillPatternFactor(profile.infill_pattern), 0.0, 1.0);
  const double weighted_volume = shell_effective_volume + core_volume * infill_fraction;
  if (!(weighted_volume > 1e-12)) {
    error = "This profile has no walls, top/bottom layers, or infill, so its estimated mass is zero.";
    return false;
  }
  const Vec3 full_first = Mul(solid_center, solid_volume);
  const Vec3 weighted_first = Add(shell_effective_first,
    Mul(Sub(full_first, shell_geometry_first), infill_fraction));
  center = Mul(weighted_first, 1.0 / weighted_volume);

  Mat3 weighted_second_origin = shell_effective_second;
  Mat3 core_second_origin = solid_second_origin;
  AddScaled(core_second_origin, shell_geometry_second, -1.0);
  AddScaled(weighted_second_origin, core_second_origin, infill_fraction);
  Mat3 second_center = weighted_second_origin;
  AddScaled(second_center, OuterProduct(center, center), -weighted_volume);
  Mat3 inertia_per_density = InertiaFromSecondMoment(second_center);
  const double material_density = material_density_kg_m3 *
    profile.flow_ratio_percent / 100.0;
  Mat3 inertia = inertia_per_density;
  for (double& value : inertia) value *= material_density;
  mass = weighted_volume * material_density;
  tensor = PackTensor(inertia);
  if (!ValidInertia(tensor)) {
    // Fallback to a scaled solid-body inertia if polygonal shell approximation
    // makes a thin or open tessellation's residual core tensor non-positive.
    Mat3 full_center_inertia = InertiaFromSecondMoment(solid_second_origin);
    AddTensor(full_center_inertia, ParallelAxis(-solid_volume, solid_center));
    for (double& value : full_center_inertia) value *= material_density * weighted_volume / solid_volume;
    AddTensor(full_center_inertia, ParallelAxis(mass, Sub(solid_center, center)));
    tensor = PackTensor(full_center_inertia);
  }
  if (!std::isfinite(mass) || mass <= 0.0 || !ValidInertia(tensor)) {
    error = "The estimated print inertia is invalid for this mesh. Try another bed face or manual mass.";
    return false;
  }
  return true;
}

bool RefreshPrintEstimate(CadPart& part) {
  double mass = 0.0;
  Vec3 center{};
  std::array<double, 6> tensor{};
  std::string error;
  const bool valid = EstimatePrintedMassProperties(part, part.print_profile,
    part.build_axis_local, mass, center, tensor, error);
  part.print_estimate_valid = valid;
  part.print_estimate_error = error;
  if (valid) {
    part.printed_mass = mass;
    part.printed_center_local = center;
    part.printed_inertia_tensor = tensor;
  }
  if (part.mass_mode == PartMassMode::Printed)
    part.mass = valid ? mass : part.fusion_mass;
  else if (part.mass_mode == PartMassMode::Manual)
    part.mass = part.manual_mass;
  else
    part.mass = part.fusion_mass;
  return valid;
}

Vec3 ClosestPointOnTriangle(Vec3 point, Vec3 a, Vec3 b, Vec3 c) {
  const Vec3 ab = Sub(b, a), ac = Sub(c, a), ap = Sub(point, a);
  const double d1 = Dot(ab, ap), d2 = Dot(ac, ap);
  if (d1 <= 0.0 && d2 <= 0.0) return a;
  const Vec3 bp = Sub(point, b);
  const double d3 = Dot(ab, bp), d4 = Dot(ac, bp);
  if (d3 >= 0.0 && d4 <= d3) return b;
  const double vc = d1*d4 - d3*d2;
  if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0)
    return Add(a, Mul(ab, d1 / (d1 - d3)));
  const Vec3 cp = Sub(point, c);
  const double d5 = Dot(ab, cp), d6 = Dot(ac, cp);
  if (d6 >= 0.0 && d5 <= d6) return c;
  const double vb = d5*d2 - d1*d6;
  if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0)
    return Add(a, Mul(ac, d2 / (d2 - d6)));
  const double va = d3*d6 - d5*d4;
  if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0)
    return Add(b, Mul(Sub(c, b), (d4 - d3) / ((d4 - d3) + (d5 - d6))));
  const double denominator = 1.0 / (va + vb + vc);
  return Add(a, Add(Mul(ab, vb * denominator), Mul(ac, vc * denominator)));
}

bool FindPrintBedAxis(const CadPart& part, Vec3 hit, Vec3& axis, std::string& error) {
  std::vector<PrintTriangle> triangles;
  MeshVolumeProperties mesh;
  if (!ReadPrintMesh(part, triangles, mesh, error)) return false;
  const Quat inverse{part.rotation[0], -part.rotation[1],
                     -part.rotation[2], -part.rotation[3]};
  const Vec3 local_hit = Rotate(inverse, Sub(hit, part.position));
  double nearest = std::numeric_limits<double>::infinity();
  const PrintTriangle* closest = nullptr;
  for (const auto& triangle : triangles) {
    const Vec3 point = ClosestPointOnTriangle(local_hit,
      triangle.a, triangle.b, triangle.c);
    const double distance = Dot(Sub(local_hit, point), Sub(local_hit, point));
    if (distance < nearest) { nearest = distance; closest = &triangle; }
  }
  if (!closest) { error = "Could not find a mesh face at that preview location."; return false; }
  axis = Mul(closest->normal, mesh.signed_volume < 0.0 ? 1.0 : -1.0);
  const double length = Length(axis);
  if (length < 1e-8) { error = "The selected STL face has no stable normal."; return false; }
  axis = Mul(axis, 1.0 / length);
  return true;
}

bool SnapToFeatureEdge(const CadPart& part, Vec3 hit, Vec3& midpoint) {
  struct Edge { Vec3 a, b, normal; int faces = 1; double min_dot = 1.0; };
  using Key = std::array<long long, 6>;
  std::map<Key, Edge> edges;
  auto file = OpenMesh(part);
  if (!file) return false;
  file->seekg(80);
  uint32_t count = 0; file->read(reinterpret_cast<char*>(&count), 4);
  for (uint32_t t = 0; t < count && *file; ++t) {
    char triangle[50]; file->read(triangle, sizeof(triangle));
    if (!*file) break;
    Vec3 v[3];
    std::array<long long, 3> key[3];
    for (int i = 0; i < 3; ++i) {
      float xyz[3]; std::memcpy(xyz, triangle + 12 + i*12, 12);
      for (int k = 0; k < 3; ++k) key[i][k] = std::llround(double(xyz[k])*10000.0);
      v[i] = Add(part.position, Rotate(part.rotation,
        Mul(Vec3{xyz[0], xyz[1], xyz[2]}, part.scale)));
    }
    Vec3 normal = Cross(Sub(v[1], v[0]), Sub(v[2], v[0]));
    const double normal_length = Length(normal);
    if (normal_length < 1e-12) continue;
    normal = Mul(normal, 1.0/normal_length);
    for (int i = 0; i < 3; ++i) {
      const int j = (i+1)%3;
      const bool forward = key[i] < key[j];
      const auto& first = forward ? key[i] : key[j];
      const auto& second = forward ? key[j] : key[i];
      if (first == second) continue;
      Key edge_key{first[0], first[1], first[2], second[0], second[1], second[2]};
      auto found = edges.find(edge_key);
      if (found == edges.end()) edges.emplace(edge_key, Edge{v[i], v[j], normal});
      else {
        found->second.min_dot = std::min(found->second.min_dot,
                                         Dot(found->second.normal, normal));
        ++found->second.faces;
      }
    }
  }
  double best_all = std::numeric_limits<double>::infinity();
  double best_feature = best_all;
  Vec3 all_mid{}, feature_mid{};
  for (const auto& entry : edges) {
    const auto& edge = entry.second;
    const Vec3 delta = Sub(edge.b, edge.a);
    const double length2 = Dot(delta, delta);
    if (length2 < 1e-16) continue;
    const double t = std::clamp(Dot(Sub(hit, edge.a), delta)/length2, 0.0, 1.0);
    const double distance = Length(Sub(hit, Add(edge.a, Mul(delta, t))));
    if (distance < best_all) { best_all = distance; all_mid = Mul(Add(edge.a, edge.b), 0.5); }
    // Boundary edges and creases over 25 degrees mark the polygonal rim.
    if ((edge.faces == 1 || edge.min_dot < 0.9063) && distance < best_feature) {
      best_feature = distance;
      feature_mid = Mul(Add(edge.a, edge.b), 0.5);
    }
  }
  if (!std::isfinite(best_all)) return false;
  const double limit = std::max(0.002, 0.04*Length(Sub(part.max_bound, part.min_bound)));
  midpoint = best_feature < limit || best_feature < 2.0*best_all ? feature_mid : all_mid;
  return true;
}

// Recover closed STL feature loops once per hovered part. Boundary loops are
// useful for open bores; sharp dihedral loops cover watertight cylindrical bores.
std::vector<std::vector<Vec3>> ReadFeatureLoops(const CadPart& part) {
  using Key = std::array<long long, 3>;
  using EdgeKey = std::pair<Key, Key>;
  struct Edge { int faces = 0; Vec3 normal{}; double min_dot = 1; };
  std::map<EdgeKey, Edge> edges;
  std::map<Key, Vec3> positions;
  auto file = OpenMesh(part);
  if (!file) return {};
  file->seekg(80);
  uint32_t count = 0; file->read(reinterpret_cast<char*>(&count), 4);
  for (uint32_t t = 0; t < count && *file; ++t) {
    char triangle[50]; file->read(triangle, sizeof(triangle));
    if (!*file) break;
    Vec3 v[3]; Key keys[3];
    for (int i = 0; i < 3; ++i) {
      float xyz[3]; std::memcpy(xyz, triangle + 12 + i*12, 12);
      for (int k = 0; k < 3; ++k) keys[i][k] = std::llround(double(xyz[k])*10000.0);
      v[i] = Add(part.position, Rotate(part.rotation,
        Mul(Vec3{xyz[0], xyz[1], xyz[2]}, part.scale)));
      positions[keys[i]] = v[i];
    }
    Vec3 normal = Cross(Sub(v[1], v[0]), Sub(v[2], v[0]));
    const double size = Length(normal);
    if (size < 1e-12) continue;
    normal = Mul(normal, 1.0/size);
    for (int i = 0; i < 3; ++i) {
      const Key& a = keys[i]; const Key& b = keys[(i+1)%3];
      if (a == b) continue;
      Edge& edge = edges[{std::min(a,b), std::max(a,b)}];
      if (edge.faces) edge.min_dot = std::min(edge.min_dot, Dot(edge.normal, normal));
      else edge.normal = normal;
      ++edge.faces;
    }
  }
  std::vector<std::vector<Vec3>> loops;
  const std::array<double, 5> crease_cosines = {0.9063078, 0.9659258, 0.9902681,
                                                0.9975641, 0.9993908};
  std::set<std::vector<Key>> emitted_loops;
  for (size_t mode = 0; mode <= crease_cosines.size(); ++mode) {
    std::map<Key, std::vector<Key>> graph;
    for (const auto& [key, edge] : edges)
      if ((mode == 0 && edge.faces == 1) ||
          (mode > 0 && edge.faces == 2 && edge.min_dot < crease_cosines[mode-1])) {
        graph[key.first].push_back(key.second);
        graph[key.second].push_back(key.first);
      }
    std::set<Key> seen;
    for (const auto& [start, adjacent] : graph) {
      if (seen.count(start) || adjacent.size() != 2) continue;
      std::vector<Key> chain;
      Key current = start, previous{};
      bool closed = false;
      for (size_t step = 0; step <= graph.size(); ++step) {
        if (seen.count(current)) { closed = current == start; break; }
        auto found = graph.find(current);
        if (found == graph.end() || found->second.size() != 2) break;
        seen.insert(current); chain.push_back(current);
        const auto& neighbors = found->second;
        Key next = chain.size() == 1 || neighbors[0] != previous ? neighbors[0] : neighbors[1];
        previous = current; current = next;
      }
      if (closed && chain.size() >= 6) {
        auto signature = chain;
        std::sort(signature.begin(), signature.end());
        if (!emitted_loops.insert(std::move(signature)).second) continue;
        std::vector<Vec3> loop;
        for (const Key& key : chain) loop.push_back(positions.at(key));
        loops.push_back(std::move(loop));
      }
    }
  }
  return loops;
}

double DistanceToLoop(const std::vector<Vec3>& loop, Vec3 hit) {
  double nearest = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < loop.size(); ++i) {
    const Vec3 a = loop[i], delta = Sub(loop[(i+1)%loop.size()], a);
    const double length2 = Dot(delta, delta);
    if (length2 < 1e-16) continue;
    nearest = std::min(nearest, Length(Sub(hit, Add(a, Mul(delta,
      std::clamp(Dot(Sub(hit, a), delta)/length2, 0.0, 1.0))))));
  }
  return nearest;
}

struct MassProperties { double mass = 0; Vec3 center{}; Mat3 inertia{}; };
MassProperties GroupMass(const std::vector<CadPart>& parts, int assembly, Vec3 origin) {
  MassProperties result;
  const auto included = [assembly](const CadPart& part) {
    return assembly == -2 ? part.assembly >= 0 : part.assembly == assembly;
  };
  for (const auto& part : parts) if (included(part) && ValidMass(part.mass)) {
    const Vec3 center = part.mass_mode == PartMassMode::Printed && part.print_estimate_valid ?
      Add(part.position, Rotate(part.rotation, part.printed_center_local)) :
      (part.has_fusion_center_of_mass ? part.center_of_mass :
       Mul(Add(part.min_bound, part.max_bound), 0.5));
    result.center = Add(result.center, Mul(center, part.mass));
    result.mass += part.mass;
  }
  if (result.mass <= 0) return result;
  result.center = Mul(result.center, 1.0 / result.mass);
  const Vec3 inertia_center = result.center;
  for (const auto& part : parts) if (included(part) && ValidMass(part.mass)) {
    const Vec3 center = part.mass_mode == PartMassMode::Printed && part.print_estimate_valid ?
      Add(part.position, Rotate(part.rotation, part.printed_center_local)) :
      (part.has_fusion_center_of_mass ? part.center_of_mass :
       Mul(Add(part.min_bound, part.max_bound), 0.5));
    const Vec3 offset = Sub(center, inertia_center);
    const double m = part.mass;
    Mat3 body_inertia{};
    if (part.mass_mode == PartMassMode::Printed && part.print_estimate_valid) {
      body_inertia = RotateTensor(UnpackTensor(part.printed_inertia_tensor), part.rotation);
    } else if (part.has_fusion_inertia_tensor && part.has_fusion_center_of_mass &&
        ValidMass(part.fusion_mass)) {
      const double mass_scale = m / part.fusion_mass;
      Mat3 local = UnpackTensor(part.fusion_inertia_tensor);
      for (double& value : local) value *= mass_scale;
      body_inertia = RotateTensor(local, part.rotation);
    } else {
      // Older bundles and manually-added unknown parts have no Fusion tensor.
      // Retain the existing bounding-box approximation for those bodies.
      const Vec3 size = Sub(part.max_bound, part.min_bound);
      body_inertia[0] = m * (size[1]*size[1] + size[2]*size[2]) / 12.0;
      body_inertia[4] = m * (size[0]*size[0] + size[2]*size[2]) / 12.0;
      body_inertia[8] = m * (size[0]*size[0] + size[1]*size[1]) / 12.0;
    }
    AddTensor(result.inertia, body_inertia);
    AddTensor(result.inertia, ParallelAxis(m, offset));
  }
  result.center = Sub(result.center, origin);
  return result;
}
Vec3 NativeDriveHeading(const std::vector<CadPart>& parts,
                        const std::vector<CadAssembly>& assemblies,
                        const FusionBundle& bundle) {
  Vec3 left_center{}, right_center{};
  int left_count = 0, right_count = 0;
  for (size_t group = 1; group < assemblies.size(); ++group) {
    if (!bundle.AssemblyEnabled(int(group))) continue;
    const auto& assembly = assemblies[group];
    if (GroupMass(parts, int(group), assembly.pivot).mass <= 0.0) continue;
    if (assembly.kind == AssemblyKind::LeftWheel) {
      left_center = Add(left_center, assembly.pivot);
      ++left_count;
    } else if (assembly.kind == AssemblyKind::RightWheel) {
      right_center = Add(right_center, assembly.pivot);
      ++right_count;
    }
  }
  if (left_count == 0 || right_count == 0) return {1, 0, 0};
  left_center = Mul(left_center, 1.0 / left_count);
  right_center = Mul(right_center, 1.0 / right_count);
  Vec3 lateral = Sub(right_center, left_center);
  lateral[2] = 0.0;
  Vec3 heading = Cross({0, 0, 1}, lateral);
  heading[2] = 0.0;
  const double length = Length(heading);
  if (length <= 1e-6) return {1, 0, 0};
  return Mul(heading, 1.0 / length);
}
Quat FacingQuaternion(Vec3 local_heading, double target_yaw) {
  const double local_yaw = std::atan2(local_heading[1], local_heading[0]);
  const double yaw = target_yaw - local_yaw;
  return {std::cos(yaw * 0.5), 0, 0, std::sin(yaw * 0.5)};
}
double MomentAboutAxis(const MassProperties& properties, Vec3 axis) {
  const double axis_length = Length(axis);
  if (!std::isfinite(axis_length) || axis_length <= 1e-10 || properties.mass <= 0.0)
    return 0.0;
  axis = Mul(axis, 1.0 / axis_length);
  double centered_moment = 0.0;
  for (int row = 0; row < 3; ++row)
    for (int column = 0; column < 3; ++column)
      centered_moment += axis[row] * properties.inertia[row * 3 + column] * axis[column];
  const double along_axis = Dot(properties.center, axis);
  const double parallel_axis = properties.mass *
    std::max(0.0, Dot(properties.center, properties.center) - along_axis * along_axis);
  const double total = centered_moment + parallel_axis;
  return std::isfinite(total) && total > 0.0 ? total : 0.0;
}
}

double MotorKvRpmPerVolt(const MotorSpec& motor) {
  if (motor.speed_input == MotorSpeedInput::RatedRpmAtVoltage) {
    if (!std::isfinite(motor.rated_rpm) || motor.rated_rpm <= 0.0 ||
        !std::isfinite(motor.rated_voltage) || motor.rated_voltage <= 0.0)
      return 0.0;
    return motor.rated_rpm / motor.rated_voltage;
  }
  return std::isfinite(motor.kv_rpm_per_volt) && motor.kv_rpm_per_volt > 0.0
    ? motor.kv_rpm_per_volt : 0.0;
}

double MotorNoLoadOutputRpm(const MotorSpec& motor, double battery_voltage) {
  const double kv = MotorKvRpmPerVolt(motor);
  if (!motor.enabled || !std::isfinite(battery_voltage) || battery_voltage <= 0.0 ||
      kv <= 0.0 ||
      !std::isfinite(motor.gear_ratio) || motor.gear_ratio <= 0.0)
    return 0.0;
  return kv * battery_voltage / motor.gear_ratio;
}

double MotorOutputTorque(const MotorSpec& motor, double battery_voltage,
                         double desired_output_speed, double actual_output_speed) {
  const double kv = MotorKvRpmPerVolt(motor);
  if (!motor.enabled || !std::isfinite(battery_voltage) || battery_voltage <= 0.0 ||
      kv <= 0.0 ||
      !std::isfinite(motor.peak_current_a) || motor.peak_current_a <= 0.0 ||
      !std::isfinite(motor.phase_resistance_ohm) || motor.phase_resistance_ohm < 0.0 ||
      !std::isfinite(motor.esc_current_limit_a) || motor.esc_current_limit_a < 0.0 ||
      !std::isfinite(motor.gear_ratio) || motor.gear_ratio <= 0.0 ||
      !std::isfinite(motor.gearbox_efficiency) || motor.gearbox_efficiency <= 0.0 ||
      motor.gearbox_efficiency > 1.0 || motor.motor_count <= 0 ||
      !std::isfinite(desired_output_speed) || !std::isfinite(actual_output_speed)) return 0.0;
  constexpr double pi = 3.14159265358979323846;
  const double motor_speed_constant = kv * 2.0 * pi / 60.0;
  const double motor_no_load_speed = motor_speed_constant * battery_voltage;
  if (motor_no_load_speed <= 0.0) return 0.0;
  const double output_no_load_speed = motor_no_load_speed / motor.gear_ratio;
  const double duty = std::clamp(desired_output_speed / output_no_load_speed, -1.0, 1.0);
  const double motor_speed = actual_output_speed * motor.gear_ratio;
  double current_limit = motor.peak_current_a;
  if (motor.esc_current_limit_a > 0.0)
    current_limit = std::min(current_limit, motor.esc_current_limit_a);
  double current = 0.0;
  if (motor.phase_resistance_ohm > 1e-9) {
    const double back_emf = motor_speed / motor_speed_constant;
    current = (duty * battery_voltage - back_emf) / motor.phase_resistance_ohm;
    current = std::clamp(current, -current_limit, current_limit);
  } else {
    const double normalized_speed = motor_speed / motor_no_load_speed;
    current = std::clamp(duty - normalized_speed, -1.0, 1.0) * current_limit;
  }
  const double torque_constant = 1.0 / motor_speed_constant;
  return torque_constant * current * motor.gear_ratio *
    motor.gearbox_efficiency * motor.motor_count;
}

double EstimateMotorSpinupSeconds(const MotorSpec& motor, double battery_voltage,
                                  double axis_inertia, double target_fraction) {
  if (!std::isfinite(axis_inertia) || axis_inertia <= 0.0 ||
      !std::isfinite(target_fraction) || target_fraction <= 0.0 || target_fraction >= 1.0)
    return 0.0;
  constexpr double pi = 3.14159265358979323846;
  constexpr int integration_steps = 1000;
  const double free_speed = MotorNoLoadOutputRpm(motor, battery_voltage) * 2.0 * pi / 60.0;
  if (!std::isfinite(free_speed) || free_speed <= 0.0) return 0.0;
  const double speed_step = target_fraction * free_speed / integration_steps;
  double seconds = 0.0;
  for (int step = 0; step < integration_steps; ++step) {
    const double midpoint_speed = (step + 0.5) * speed_step;
    const double torque = MotorOutputTorque(motor, battery_voltage, free_speed, midpoint_speed);
    if (!std::isfinite(torque) || torque <= 1e-12) return 0.0;
    seconds += axis_inertia * speed_step / torque;
  }
  return std::isfinite(seconds) && seconds > 0.0 ? seconds : 0.0;
}

bool FusionBundle::Load(const std::filesystem::path& manifest) {
  try {
    std::ifstream input(manifest, std::ios::binary);
    if (!input) throw std::runtime_error("Cannot open manifest: " + manifest.string());
    const std::string manifest_json{std::istreambuf_iterator<char>(input),
                                    std::istreambuf_iterator<char>()};
    return LoadContent(manifest_json, manifest.parent_path(), nullptr, {});
  } catch (const std::exception& e) { error_ = e.what(); return false; }
}

FusionBundle::~FusionBundle() {
  Clear();
}

bool FusionBundle::LoadContent(
    const std::string& manifest_json, const std::filesystem::path& root,
    const std::map<std::string, std::vector<unsigned char>>* meshes,
    const std::string& setup_json) {
  std::vector<CadPart> loaded;
  try {
    const bool from_memory = meshes != nullptr;
    const json source = json::parse(manifest_json);
    if (!source.contains("parts") || !source.at("parts").is_array() || source.at("parts").empty())
      throw std::runtime_error("Manifest has no parts array.");
    std::vector<std::string> missing;
    const auto folder = from_memory ? std::filesystem::path{} :
      std::filesystem::weakly_canonical(NativePath(root));
    const double default_scale = source.value("unitToMeters", 0.001);
    auto load_companion = [&](const std::string& relative_name,
                              std::vector<unsigned char>& data,
                              std::filesystem::path& absolute_path) {
      const std::filesystem::path relative(relative_name);
      const auto normalized = relative.lexically_normal();
      if (!IsBundleAssetPath(relative) || normalized.generic_u8string() != relative_name)
        throw std::runtime_error("Unsafe or unsupported bundle asset path: " + relative_name);
      if (from_memory) {
        const auto found = meshes->find(normalized.generic_u8string());
        if (found == meshes->end() || found->second.empty()) {
          missing.push_back(relative_name);
          return;
        }
        data = found->second;
        return;
      }
      const auto resolved = std::filesystem::weakly_canonical(NativePath(folder / normalized));
      const auto inside = resolved.lexically_relative(folder);
      if (inside.empty() || *inside.begin() == "..")
        throw std::runtime_error("Unsafe bundle asset path: " + relative_name);
      absolute_path = resolved;
      if (!std::filesystem::is_regular_file(NativePath(resolved)))
        missing.push_back(relative_name);
    };
    for (const auto& item : source.at("parts")) {
      CadPart part;
      part.file = item.at("file").get<std::string>();
      const std::filesystem::path relative_file(part.file);
      if (relative_file.empty() || relative_file.is_absolute() ||
          relative_file.has_root_name() || relative_file.has_root_directory())
        throw std::runtime_error("Unsafe or unsupported mesh path: " + part.file);
      const auto normalized_relative = relative_file.lexically_normal();
      if (normalized_relative.empty() || *normalized_relative.begin() == "..")
        throw std::runtime_error("Unsafe or unsupported mesh path: " + part.file);
      if (normalized_relative.extension() != ".stl")
        throw std::runtime_error("Unsafe or unsupported mesh path: " + part.file);
      if (from_memory) {
        const std::string key = normalized_relative.generic_u8string();
        const auto found = meshes->find(key);
        if (found == meshes->end() || found->second.empty()) {
          missing.push_back(part.file); continue;
        }
        part.mesh_data = found->second;
      } else {
        const auto resolved_path = std::filesystem::weakly_canonical(
          NativePath(folder / normalized_relative));
        const auto relative = resolved_path.lexically_relative(folder);
        if (relative.empty() || *relative.begin() == "..")
          throw std::runtime_error("Unsafe or unsupported mesh path: " + part.file);
        // weakly_canonical strips the Win32 extended-length prefix, so restore it
        // before filesystem checks and all later STL reads.
        const auto native_path = NativePath(resolved_path);
        if (!std::filesystem::is_regular_file(native_path)) {
          missing.push_back(part.file); continue;
        }
        part.absolute_file = resolved_path;
#if defined(_WIN32)
        if (resolved_path.native().size() > 240) {
          const auto cache_id = MeshCacheId(root / "manifest.json");
          auto cached_path = ShortMeshCachePath(
            folder / "Parts" / "_CombatSimCache" / cache_id, loaded.size());
          try {
            std::filesystem::create_directories(cached_path.parent_path());
            std::filesystem::copy_file(native_path, cached_path,
              std::filesystem::copy_options::overwrite_existing);
          } catch (const std::filesystem::filesystem_error&) {
            // Read-only bundles can still be imported; keep their copied meshes
            // in the user's short system temp directory for this session.
            cached_path = ShortMeshCachePath(
              std::filesystem::temp_directory_path() / "CombatSim" / cache_id,
              loaded.size());
            std::filesystem::create_directories(cached_path.parent_path());
            std::filesystem::copy_file(native_path, cached_path,
              std::filesystem::copy_options::overwrite_existing);
          }
          part.absolute_file = cached_path;
        }
#endif
      }
      part.render_file = item.value("renderFile", part.file);
      if (part.render_file == part.file) {
        part.absolute_render_file = part.absolute_file;
      } else {
        load_companion(part.render_file, part.render_mesh_data, part.absolute_render_file);
      }
      if (item.contains("textures") && item.at("textures").is_object()) {
        for (auto it = item.at("textures").begin(); it != item.at("textures").end(); ++it) {
          if (!it.value().is_string() || !IsTextureRole(it.key()))
            throw std::runtime_error("Manifest has an invalid Fusion texture entry.");
          CadPart::Texture texture;
          texture.role = it.key();
          texture.file = it.value().get<std::string>();
          load_companion(texture.file, texture.data, texture.absolute_file);
          part.textures.push_back(std::move(texture));
        }
      }
      part.name = item.value("partName", std::string("Part"));
      part.material = item.value("materialName", std::string("Unknown"));
      part.appearance = item.value("appearanceName", std::string());
      if (item.contains("roughness") && item.at("roughness").is_number())
        part.roughness = item.at("roughness").get<double>();
      if (item.contains("materialProperties") && item.at("materialProperties").is_array()) {
        for (const auto& prop : item.at("materialProperties")) {
          if (!prop.is_object() || !prop.contains("value")) continue;
          std::string value = prop.at("value").is_string() ? prop.at("value").get<std::string>() : prop.at("value").dump();
          part.material_properties.push_back(prop.value("name", std::string("Property")) + ": " + value +
            " " + prop.value("units", std::string()));
        }
      }
      if (item.contains("occurrencePath") && item.at("occurrencePath").is_array())
        part.occurrence_path = item.at("occurrencePath").get<std::vector<std::string>>();
      else part.occurrence_path = {item.value("bodyName", std::string("Root"))};
      // Flexible wire/cable STLs are poor rigid collision shapes. Their convex
      // hulls can extend below the chassis and act like unintended floor feet.
      part.collision_enabled = !FlexiblePartPath(part);
      part.density = item.value("densityKgPerM3", 0.0);
      part.mass = item.value("massKg", 0.0);
      if (part.mass <= 0 && item.contains("volumeM3") && part.density > 0)
        part.mass = item.at("volumeM3").get<double>() * part.density;
      part.fusion_mass = part.mass;
      part.manual_mass = part.mass;
      part.volume_m3 = item.value("volumeM3", 0.0);
      if ((!std::isfinite(part.density) || part.density <= 0.0) &&
          part.volume_m3 > 1e-12 && part.fusion_mass > 0.0)
        part.density = part.fusion_mass / part.volume_m3;
      const auto& print_profiles = PrintProfiles();
      if (!print_profiles.empty()) part.print_profile = print_profiles.front();
      part.friction = item.value("frictionEstimate", 0.0);
      if (part.friction <= 0) part.friction = 0.6;
      part.scale = item.value("unitToMeters", default_scale);
      if (!std::isfinite(part.scale) || part.scale <= 0) throw std::runtime_error("Invalid STL unit scale: " + part.file);
      if (!item.contains("positionFusionMeters") ||
          !item.at("positionFusionMeters").is_array() ||
          item.at("positionFusionMeters").size() != 3 ||
          !item.contains("rotationFusionWxyz") ||
          !item.at("rotationFusionWxyz").is_array() ||
          item.at("rotationFusionWxyz").size() != 4)
        throw std::runtime_error(
            "Manifest part is missing valid Fusion-native transforms; "
            "re-export with the current Fusion exporter: " + part.file);
      part.position = ReadArray<3>(item, "positionFusionMeters", {});
      part.rotation = ReadArray<4>(item, "rotationFusionWxyz", {1, 0, 0, 0});
      const double qlength = std::sqrt(part.rotation[0]*part.rotation[0]+part.rotation[1]*part.rotation[1]+
                                       part.rotation[2]*part.rotation[2]+part.rotation[3]*part.rotation[3]);
      if (qlength < 1e-8) throw std::runtime_error("Invalid rotation: " + part.file);
      for (double& q : part.rotation) q /= qlength;
      if (item.contains("centerOfMassLocalMeters") &&
          item.at("centerOfMassLocalMeters").is_array() &&
          item.at("centerOfMassLocalMeters").size() == 3) {
        const Vec3 local_center = ReadArray<3>(item, "centerOfMassLocalMeters", {});
        if (std::all_of(local_center.begin(), local_center.end(),
                        [](double value) { return std::isfinite(value); })) {
          part.center_of_mass_local = local_center;
          part.center_of_mass = Add(part.position, Rotate(part.rotation, local_center));
          part.has_fusion_center_of_mass = true;
        }
      }
      if (item.contains("inertiaTensorLocalKgM2") &&
          item.at("inertiaTensorLocalKgM2").is_array() &&
          item.at("inertiaTensorLocalKgM2").size() == 6) {
        part.fusion_inertia_tensor = ReadArray<6>(item, "inertiaTensorLocalKgM2", {});
        part.has_fusion_inertia_tensor = ValidInertia(part.fusion_inertia_tensor);
      }
      if (item.contains("colorRgba") && item.at("colorRgba").is_array())
        part.color = ReadArray<4>(item, "colorRgba", part.color);
      // Some Fusion opaque appearances export their base color with alpha 0,
      // even though the physical body has mass and should render opaque.
      RepairInvisibleOpaquePartColor(part);
      loaded.push_back(std::move(part));
      CadPart& loaded_part = loaded.back();
      if (from_memory)
        ReadStlBounds(loaded_part.mesh_data, loaded_part.file, loaded_part.scale,
                      loaded_part.position, loaded_part.rotation,
                      loaded_part.min_bound, loaded_part.max_bound);
      else
        ReadStlBounds(loaded_part.absolute_file, loaded_part.scale, loaded_part.position,
                      loaded_part.rotation, loaded_part.min_bound, loaded_part.max_bound);
    }
    if (!missing.empty()) {
      std::ostringstream message;
      message << "Fusion bundle is incomplete. Missing " << missing.size() << " asset files:\n";
      for (const auto& file : missing) message << file << '\n';
      throw std::runtime_error(message.str());
    }
    if (loaded.empty()) throw std::runtime_error("No valid STL bodies in manifest.");
    std::vector<CadAssembly> groups{{"Main frame", AssemblyKind::Frame, {}, {0, 1, 0}},
                                    {"Weapon", AssemblyKind::Weapon, {}, {0, 1, 0}},
                                    {"Front left wheel", AssemblyKind::LeftWheel, {}, {0, 1, 0}},
                                    {"Rear left wheel", AssemblyKind::LeftWheel, {}, {0, 1, 0}},
                                    {"Front right wheel", AssemblyKind::RightWheel, {}, {0, 1, 0}},
                                    {"Rear right wheel", AssemblyKind::RightWheel, {}, {0, 1, 0}}};
    groups[2].wheel_position = "front";
    groups[3].wheel_position = "rear";
    groups[4].wheel_position = "front";
    groups[5].wheel_position = "rear";
    int default_drive_group = 0;
    for (size_t i = 1; i < groups.size(); ++i) {
      if (groups[i].kind == AssemblyKind::LeftWheel ||
          groups[i].kind == AssemblyKind::RightWheel)
        groups[i].motor.drive_group = default_drive_group++;
      else
        groups[i].motor.drive_group = -1;
    }
    const auto saved = folder / "combatsim_setup.json";
    battery_voltage_ = 0.0;
    bot_rotation_degrees_ = 0.0;
    drive_layout_ = DriveLayout::FourWheel;
    two_wd_axle_ = "front";
    int saved_axis_assembly = 1;
    bool saved_show_axis_markers = true;
    bool saved_drive_layout_defined = false;
    std::ifstream disk_state_stream;
    std::istringstream memory_state_stream(setup_json);
    std::istream* state_stream = nullptr;
    if (from_memory && !setup_json.empty()) state_stream = &memory_state_stream;
    else if (!from_memory && std::filesystem::is_regular_file(saved)) {
      disk_state_stream.open(saved);
      state_stream = &disk_state_stream;
    }
    if (state_stream) {
      const json state = json::parse(*state_stream);
      saved_axis_assembly = state.value("axisAssembly", 1);
      saved_show_axis_markers = state.value("showAxisMarkers", true);
      battery_voltage_ = state.value("batteryVoltage", 0.0);
      bot_rotation_degrees_ = state.value("botRotationDegrees", 0.0);
      if (!std::isfinite(bot_rotation_degrees_)) bot_rotation_degrees_ = 0.0;
      bot_rotation_degrees_ = std::clamp(bot_rotation_degrees_, -3600.0, 3600.0);
      const std::string saved_drive_layout = state.value("driveLayout", std::string());
      saved_drive_layout_defined = !saved_drive_layout.empty();
      drive_layout_ = saved_drive_layout == "2WD" ? DriveLayout::TwoWheel :
        saved_drive_layout == "4WD" ? DriveLayout::FourWheel : DriveLayout::FourWheel;
      two_wd_axle_ = state.value("twoWdAxle", std::string("front")) == "rear" ? "rear" : "front";
      if (state.contains("assemblies") && state.at("assemblies").is_array()) {
        groups.resize(1);
        std::set<int> used_drive_groups;
        for (const auto& node : state.at("assemblies")) {
          CadAssembly group;
          group.name = node.value("name", std::string("Assembly"));
          group.kind = ParseKind(node.value("kind", std::string("Fixed")));
          group.pivot = ReadArray<3>(node, "pivotMeters", {});
          group.axis = ReadArray<3>(node, "axis", {0, 1, 0});
          group.fork_range_degrees = ReadArray<2>(node, "forkRangeDegrees", {-90, 90});
          group.fork_collides_with_frame = node.value("forkCollidesWithFrame", true);
          group.wheel_position = node.value("wheelPosition", std::string());
          bool has_saved_drive_group = false;
          if (node.contains("motor") && node.at("motor").is_object()) {
            const auto& motor = node.at("motor");
            group.motor.enabled = motor.value("enabled", false);
            group.motor.type = ParseMotorType(motor.value("motorType", std::string("brushless")));
            group.motor.speed_input = ParseMotorSpeedInput(
              motor.value("speedInput", std::string("kv")));
            if (motor.contains("driveGroup")) {
              group.motor.drive_group = motor.value("driveGroup", -1);
              has_saved_drive_group = true;
            }
            group.motor.motor_count = motor.value("motorCount", 1);
            group.motor.kv_rpm_per_volt = motor.value("kvRpmPerVolt", 0.0);
            group.motor.rated_rpm = motor.value("ratedRpm", 0.0);
            group.motor.rated_voltage = motor.value("ratedVoltage", 0.0);
            group.motor.peak_current_a = motor.value(
              "peakCurrentA", motor.value("ratedCurrentA", 0.0));
            group.motor.phase_resistance_ohm = motor.value(
              "armatureResistanceOhm", motor.value("phaseResistanceOhm", 0.0));
            group.motor.esc_current_limit_a = motor.value("escCurrentLimitA", 0.0);
            if (motor.contains("gearRatio") && motor.at("gearRatio").is_string()) {
              group.motor.gear_ratio_text = motor.at("gearRatio").get<std::string>();
              if (!ParseGearRatio(group.motor.gear_ratio_text, group.motor.gear_ratio))
                throw std::runtime_error("Invalid input:output gear ratio for: " + group.name);
            } else {
              group.motor.gear_ratio = motor.value("gearRatio", 1.0);
              group.motor.gear_ratio_text = FormatGearRatio(group.motor.gear_ratio);
            }
            group.motor.gearbox_efficiency = motor.value("gearboxEfficiency", 1.0);
          }
          if (group.kind == AssemblyKind::LeftWheel ||
              group.kind == AssemblyKind::RightWheel) {
            if (!has_saved_drive_group || group.motor.drive_group < 0 ||
                group.motor.drive_group > 7) {
              group.motor.drive_group = 0;
              while (used_drive_groups.count(group.motor.drive_group) &&
                     group.motor.drive_group < 7)
                ++group.motor.drive_group;
            }
            used_drive_groups.insert(group.motor.drive_group);
          } else {
            group.motor.drive_group = -1;
            if (group.kind != AssemblyKind::Weapon)
              group.motor.enabled = false;
          }
          groups.push_back(group);
        }
      }
      if (state.contains("partAssignments") && state.at("partAssignments").is_object()) {
        const auto& assignments = state.at("partAssignments");
        for (auto& part : loaded) if (assignments.contains(part.file))
          part.assembly = assignments.at(part.file).get<int>();
      }
      if (state.contains("partOverrides") && state.at("partOverrides").is_object()) {
        const auto& overrides = state.at("partOverrides");
        for (auto& part : loaded) if (overrides.contains(part.file)) {
          const auto& override = overrides.at(part.file);
          const double legacy_mass = override.value("massKg", part.mass);
          part.manual_mass = override.value("manualMassKg", legacy_mass);
          if (override.contains("massMode"))
            part.mass_mode = ParseMassMode(override.value("massMode", std::string("fusion")));
          else if (std::abs(legacy_mass - part.fusion_mass) > 1e-9)
            part.mass_mode = PartMassMode::Manual;
          if (override.contains("buildAxisLocal") && override.at("buildAxisLocal").is_array())
            part.build_axis_local = ReadArray<3>(override, "buildAxisLocal", part.build_axis_local);
          if (override.contains("printProfile") && override.at("printProfile").is_object())
            part.print_profile = ReadPrintProfile(override.at("printProfile"));
          part.friction = override.value("frictionEstimate", part.friction);
          part.color = ReadArray<4>(override, "colorRgba", part.color);
          part.preview_visible = override.value("previewVisible", true);
          part.collision_enabled = override.value("collisionEnabled", part.collision_enabled);
        }
      }
      // Older saved setups can contain the same zero-alpha color as the
      // manifest. Repair after applying overrides so reloading a configured
      // bot cannot make opaque UHMW or rubber parts invisible again.
      for (auto& part : loaded) {
        RepairInvisibleOpaquePartColor(part);
        if (part.mass_mode == PartMassMode::Printed) RefreshPrintEstimate(part);
        else if (part.mass_mode == PartMassMode::Manual) part.mass = part.manual_mass;
        else part.mass = part.fusion_mass;
      }
    }
    // Keep legacy wheel sets usable: infer axle position from old assembly names,
    // then assign stable front/rear roles to any remaining side wheel groups.
    for (const AssemblyKind side : {AssemblyKind::LeftWheel, AssemblyKind::RightWheel}) {
      std::vector<int> wheel_groups;
      for (size_t i = 1; i < groups.size(); ++i) {
        if (groups[i].kind != side) continue;
        wheel_groups.push_back(int(i));
        if (!groups[i].wheel_position.empty()) continue;
        const std::string lower_name = Lower(groups[i].name);
        if (lower_name.find("front") != std::string::npos) groups[i].wheel_position = "front";
        else if (lower_name.find("rear") != std::string::npos) groups[i].wheel_position = "rear";
      }
      for (size_t ordinal = 0; ordinal < wheel_groups.size(); ++ordinal) {
        auto& group = groups[wheel_groups[ordinal]];
        if (!group.wheel_position.empty()) continue;
        if (wheel_groups.size() == 1) group.wheel_position = "single";
        else if (ordinal == 0) group.wheel_position = "front";
        else if (ordinal == 1) group.wheel_position = "rear";
        else group.wheel_position = "other";
      }
    }
    if (!saved_drive_layout_defined) {
      size_t wheel_groups = 0;
      for (size_t i = 1; i < groups.size(); ++i) if (IsWheelKind(groups[i].kind)) ++wheel_groups;
      drive_layout_ = wheel_groups <= 2 ? DriveLayout::TwoWheel : DriveLayout::FourWheel;
    }
    for (auto& part : loaded) if (part.assembly < -1 || part.assembly >= int(groups.size())) part.assembly = 0;
    WipeParts(parts_);
    Wipe(manifest_json_);
    manifest_ = from_memory ? std::filesystem::path{} :
      std::filesystem::weakly_canonical(root / "manifest.json");
    manifest_json_ = manifest_json;
    design_name_ = source.value("designName", std::string("Fusion bot"));
    parts_ = std::move(loaded);
    assemblies_ = std::move(groups);
    memory_loaded_ = from_memory;
    show_axis_markers_ = saved_show_axis_markers;
    const auto path = manifest_.string();
    std::snprintf(path_buffer_, sizeof(path_buffer_), "%s", path.c_str());
    error_.clear();
    selected_part_ = focus_part_request_ = -1;
    focus_frames_remaining_ = 0;
    hovered_part_ = -1; hovered_loop_.clear(); hole_cache_.clear();
    pick_message_.clear(); axis_points_.clear(); axis_picking_ = false;
    print_bed_face_picking_ = false; print_bed_face_part_ = -1;
    axis_assembly_ = (saved_axis_assembly > 0 &&
                      saved_axis_assembly < int(assemblies_.size()) &&
                      assemblies_[saved_axis_assembly].kind != AssemblyKind::Fixed &&
                      AssemblyEnabled(saved_axis_assembly)) ?
      saved_axis_assembly : 0;
    if (axis_assembly_ == 0) {
      for (size_t i = 1; i < assemblies_.size(); ++i)
        if (assemblies_[i].kind != AssemblyKind::Fixed && AssemblyEnabled(int(i))) {
          axis_assembly_ = int(i);
          break;
        }
    }
    return true;
  } catch (const std::exception& e) {
    WipeParts(loaded);
    error_ = e.what();
    return false;
  }
}

bool FusionBundle::AssemblyEnabled(int assembly) const {
  if (assembly <= 0 || assembly >= int(assemblies_.size())) return false;
  const auto& group = assemblies_[assembly];
  if (!IsWheelKind(group.kind) || drive_layout_ == DriveLayout::FourWheel) return true;
  int selected_axle_group = -1;
  int single_group = -1;
  for (size_t i = 1; i < assemblies_.size(); ++i) {
    if (assemblies_[i].kind != group.kind) continue;
    if (assemblies_[i].wheel_position == two_wd_axle_ && selected_axle_group < 0)
      selected_axle_group = int(i);
    else if (assemblies_[i].wheel_position == "single" && single_group < 0)
      single_group = int(i);
  }
  return assembly == (selected_axle_group >= 0 ? selected_axle_group : single_group);
}

bool FusionBundle::PartIncludedInSimulation(int part_index) const {
  if (part_index < 0 || part_index >= int(parts_.size())) return false;
  const int assembly = parts_[part_index].assembly;
  return assembly == 0 || AssemblyEnabled(assembly);
}

void FusionBundle::SetTwoWheelDriveAxle(const std::string& axle) {
  two_wd_axle_ = axle == "rear" ? "rear" : "front";
  if (axis_assembly_ > 0 && axis_assembly_ < int(assemblies_.size()) &&
      AssemblyEnabled(axis_assembly_)) return;
  axis_picking_ = false;
  axis_points_.clear();
  hovered_loop_.clear();
  axis_assembly_ = 0;
  for (size_t i = 1; i < assemblies_.size(); ++i) {
    if (assemblies_[i].kind != AssemblyKind::Fixed && AssemblyEnabled(int(i))) {
      axis_assembly_ = int(i);
      break;
    }
  }
}

void FusionBundle::SetWheelDriveLayout(DriveLayout layout) {
  if (layout == DriveLayout::TwoWheel) {
    drive_layout_ = layout;
    if (axis_assembly_ <= 0 || axis_assembly_ >= int(assemblies_.size()) ||
        !AssemblyEnabled(axis_assembly_)) SetTwoWheelDriveAxle(two_wd_axle_);
    return;
  }
  for (const auto side : {AssemblyKind::LeftWheel, AssemblyKind::RightWheel}) {
    std::vector<int> side_groups;
    bool have_front = false, have_rear = false;
    for (size_t i = 1; i < assemblies_.size(); ++i) {
      if (assemblies_[i].kind != side) continue;
      side_groups.push_back(int(i));
      have_front = have_front || assemblies_[i].wheel_position == "front";
      have_rear = have_rear || assemblies_[i].wheel_position == "rear";
    }
    for (int index : side_groups) {
      auto& group = assemblies_[index];
      if (group.wheel_position == "single") {
        group.wheel_position = two_wd_axle_;
        have_front = have_front || group.wheel_position == "front";
        have_rear = have_rear || group.wheel_position == "rear";
      }
    }
    if (side_groups.empty()) {
      CadAssembly wheel;
      wheel.kind = side;
      wheel.name = side == AssemblyKind::LeftWheel ? "Left wheel" : "Right wheel";
      wheel.wheel_position = two_wd_axle_;
      wheel.motor.drive_group = 0;
      assemblies_.push_back(wheel);
      side_groups.push_back(int(assemblies_.size()) - 1);
      have_front = two_wd_axle_ == "front";
      have_rear = two_wd_axle_ == "rear";
    }
    if (!have_front || !have_rear) {
      const std::string missing_position = !have_front ? "front" : "rear";
      CadAssembly wheel;
      wheel.kind = side;
      wheel.wheel_position = missing_position;
      wheel.name = (missing_position == "front" ? "Front " : "Rear ") +
        std::string(side == AssemblyKind::LeftWheel ? "left wheel" : "right wheel");
      int drive_group = 0;
      for (size_t i = 1; i < assemblies_.size(); ++i)
        if (IsWheelKind(assemblies_[i].kind))
          drive_group = std::max(drive_group, assemblies_[i].motor.drive_group + 1);
      wheel.motor.drive_group = std::min(drive_group, 7);
      assemblies_.push_back(wheel);
    }
  }
  drive_layout_ = DriveLayout::FourWheel;
  if (axis_assembly_ <= 0 || axis_assembly_ >= int(assemblies_.size()) ||
      assemblies_[axis_assembly_].kind == AssemblyKind::Fixed) {
    axis_assembly_ = 0;
    for (size_t i = 1; i < assemblies_.size(); ++i)
      if (assemblies_[i].kind != AssemblyKind::Fixed) {
        axis_assembly_ = int(i);
        break;
      }
  }
}

bool FusionBundle::ExportSessionPackage(std::vector<unsigned char>& package) {
  Wipe(package);
  std::map<std::string, std::vector<unsigned char>> files;
  std::string manifest_text;
  std::string setup_text;
  try {
    if (!HasBundle()) throw std::runtime_error("Load a bot bundle before starting a session.");
    constexpr uint64_t max_package_bytes = 512ull * 1024ull * 1024ull;
    manifest_text = manifest_json_;
    setup_text = SetupJson();
    if (!memory_loaded_) {
      std::ifstream manifest_file(manifest_, std::ios::binary);
      if (!manifest_file) throw std::runtime_error("Cannot read the loaded manifest for LAN play.");
      manifest_text.assign(std::istreambuf_iterator<char>(manifest_file),
                           std::istreambuf_iterator<char>());
    }
    if (manifest_text.empty() || manifest_text.size() > 16u * 1024u * 1024u ||
        setup_text.size() > 16u * 1024u * 1024u)
      throw std::runtime_error("Manifest or setup exceeds the LAN package limit.");

    const json manifest_source = json::parse(manifest_text);
    if (!manifest_source.contains("parts") || !manifest_source.at("parts").is_array() ||
        manifest_source.at("parts").empty() || manifest_source.at("parts").size() > 4096)
      throw std::runtime_error("Manifest has an invalid part list for LAN play.");
    auto safe_name = [](const std::string& input) {
      const std::filesystem::path path(input);
      if (input.empty() || path.is_absolute() || path.has_root_name() ||
          path.has_root_directory())
        throw std::runtime_error("Unsafe asset path in LAN package.");
      const auto normalized = path.lexically_normal();
      if (!IsBundleAssetPath(path) || normalized.generic_u8string() != input)
        throw std::runtime_error("Unsupported asset path in LAN package.");
      return normalized.generic_u8string();
    };
    for (const auto& raw_name : ManifestAssetPaths(manifest_source.at("parts"))) {
      const std::string name = safe_name(raw_name);
      if (files.count(name)) continue;
      if (memory_loaded_) {
        bool found_asset = false;
        for (const auto& part : parts_) {
          if (safe_name(part.file) == name && !part.mesh_data.empty()) {
            files.emplace(name, part.mesh_data); found_asset = true; break;
          }
          if (safe_name(part.render_file) == name && !part.render_mesh_data.empty()) {
            files.emplace(name, part.render_mesh_data); found_asset = true; break;
          }
          for (const auto& texture : part.textures) {
            if (safe_name(texture.file) == name && !texture.data.empty()) {
              files.emplace(name, texture.data); found_asset = true; break;
            }
          }
          if (found_asset) break;
        }
        if (!found_asset)
          throw std::runtime_error("Session asset is missing from memory: " + name);
      } else {
        std::filesystem::path asset_path;
        for (const auto& part : parts_) {
          if (safe_name(part.file) == name) { asset_path = part.absolute_file; break; }
          if (safe_name(part.render_file) == name) { asset_path = part.absolute_render_file; break; }
          for (const auto& texture : part.textures) {
            if (safe_name(texture.file) == name) { asset_path = texture.absolute_file; break; }
          }
          if (!asset_path.empty()) break;
        }
        if (asset_path.empty()) throw std::runtime_error("Manifest asset was not loaded: " + name);
        std::ifstream mesh(asset_path, std::ios::binary);
        if (!mesh) throw std::runtime_error("Cannot read asset for LAN play: " + name);
        std::vector<unsigned char> bytes{std::istreambuf_iterator<char>(mesh),
                                         std::istreambuf_iterator<char>()};
        if (bytes.empty()) throw std::runtime_error("Empty asset cannot be sent: " + name);
        files.emplace(name, std::move(bytes));
      }
    }

    uint64_t total = 20u + manifest_text.size() + setup_text.size();
    for (const auto& [name, bytes] : files) {
      if (name.size() > 65535 || bytes.empty())
        throw std::runtime_error("LAN package contains an invalid mesh entry.");
      total += 10u + name.size() + bytes.size();
    }
    if (total > max_package_bytes)
      throw std::runtime_error("Bot bundle exceeds the 512 MiB LAN session limit.");
    package.clear();
    package.reserve(size_t(total));
    auto put_u32 = [&](uint32_t value) {
      for (int i = 0; i < 4; ++i)
        package.push_back(static_cast<unsigned char>(value >> (8 * i)));
    };
    auto put_u64 = [&](uint64_t value) {
      for (int i = 0; i < 8; ++i)
        package.push_back(static_cast<unsigned char>(value >> (8 * i)));
    };
    auto put_u16 = [&](uint16_t value) {
      package.push_back(static_cast<unsigned char>(value));
      package.push_back(static_cast<unsigned char>(value >> 8));
    };
    const unsigned char magic[8] = {'C','S','B','P','K','G','0','1'};
    package.insert(package.end(), std::begin(magic), std::end(magic));
    put_u32(uint32_t(manifest_text.size()));
    put_u32(uint32_t(setup_text.size()));
    put_u32(uint32_t(files.size()));
    package.insert(package.end(), manifest_text.begin(), manifest_text.end());
    package.insert(package.end(), setup_text.begin(), setup_text.end());
    for (const auto& [name, bytes] : files) {
      put_u16(uint16_t(name.size()));
      put_u64(uint64_t(bytes.size()));
      package.insert(package.end(), name.begin(), name.end());
      package.insert(package.end(), bytes.begin(), bytes.end());
    }
    WipeMeshes(files);
    Wipe(manifest_text);
    Wipe(setup_text);
    return true;
  } catch (const std::exception& e) {
    Wipe(package);
    WipeMeshes(files);
    Wipe(manifest_text);
    Wipe(setup_text);
    error_ = e.what();
    return false;
  }
}

bool FusionBundle::LoadSessionPackage(const std::vector<unsigned char>& package) {
  std::map<std::string, std::vector<unsigned char>> meshes;
  std::string manifest_text;
  std::string setup_text;
  try {
    constexpr uint64_t max_package_bytes = 512ull * 1024ull * 1024ull;
    constexpr unsigned char magic[8] = {'C','S','B','P','K','G','0','1'};
    if (package.size() < 20 || package.size() > max_package_bytes ||
        !std::equal(std::begin(magic), std::end(magic), package.begin()))
      throw std::runtime_error("Invalid or oversized in-memory bot package.");
    size_t offset = 8;
    auto read_u32 = [&]() {
      if (package.size() - offset < 4) throw std::runtime_error("Truncated LAN package header.");
      uint32_t value = 0;
      for (int i = 0; i < 4; ++i) value |= uint32_t(package[offset++]) << (8 * i);
      return value;
    };
    auto read_u64 = [&]() {
      if (package.size() - offset < 8) throw std::runtime_error("Truncated LAN package header.");
      uint64_t value = 0;
      for (int i = 0; i < 8; ++i) value |= uint64_t(package[offset++]) << (8 * i);
      return value;
    };
    auto read_u16 = [&]() {
      if (package.size() - offset < 2) throw std::runtime_error("Truncated LAN package entry.");
      const uint16_t value = uint16_t(package[offset]) |
                             uint16_t(uint16_t(package[offset + 1]) << 8);
      offset += 2;
      return value;
    };
    const uint32_t manifest_size = read_u32();
    const uint32_t setup_size = read_u32();
    const uint32_t file_count = read_u32();
    if (!manifest_size || manifest_size > 16u * 1024u * 1024u ||
        setup_size > 16u * 1024u * 1024u || file_count == 0 || file_count > 4096)
      throw std::runtime_error("LAN package header exceeds its allowed limits.");
    if (uint64_t(manifest_size) + setup_size > package.size() - offset)
      throw std::runtime_error("Truncated manifest in LAN package.");
    manifest_text.assign(package.begin() + offset,
                         package.begin() + offset + manifest_size);
    offset += manifest_size;
    setup_text.assign(package.begin() + offset,
                      package.begin() + offset + setup_size);
    offset += setup_size;
    auto safe_name = [](const std::string& input) {
      const std::filesystem::path path(input);
      if (input.empty() || path.is_absolute() || path.has_root_name() ||
          path.has_root_directory())
        throw std::runtime_error("Unsafe asset path in LAN package.");
      const auto normalized = path.lexically_normal();
      if (!IsBundleAssetPath(path) || normalized.generic_u8string() != input)
        throw std::runtime_error("Unsupported asset path in LAN package.");
      return normalized.generic_u8string();
    };
    uint64_t total_mesh_bytes = 0;
    for (uint32_t i = 0; i < file_count; ++i) {
      const uint16_t name_size = read_u16();
      const uint64_t data_size = read_u64();
      if (!name_size || name_size > 4096 || !data_size ||
          data_size > max_package_bytes || name_size > package.size() - offset ||
          data_size > package.size() - offset - name_size)
        throw std::runtime_error("Invalid or truncated mesh entry in LAN package.");
      const std::string raw_name(package.begin() + offset,
                                 package.begin() + offset + name_size);
      offset += name_size;
      const std::string name = safe_name(raw_name);
      if (meshes.count(name))
        throw std::runtime_error("Duplicate mesh in LAN package.");
      std::vector<unsigned char> bytes(package.begin() + offset,
          package.begin() + offset + size_t(data_size));
      offset += size_t(data_size);
      total_mesh_bytes += data_size;
      if (total_mesh_bytes > max_package_bytes)
        throw std::runtime_error("Duplicate or oversized mesh in LAN package.");
      meshes.emplace(name, std::move(bytes));
    }
    if (offset != package.size()) throw std::runtime_error("Unexpected trailing bytes in LAN package.");
    const json manifest_source = json::parse(manifest_text);
    if (!manifest_source.contains("parts") || !manifest_source.at("parts").is_array() ||
        manifest_source.at("parts").empty() || manifest_source.at("parts").size() > 4096)
      throw std::runtime_error("Manifest has an invalid part list for LAN play.");
    std::set<std::string> required;
    for (const auto& raw_name : ManifestAssetPaths(manifest_source.at("parts")))
      required.insert(safe_name(raw_name));
    if (required.size() != meshes.size())
      throw std::runtime_error("LAN package mesh list does not match its manifest.");
    for (const auto& name : required)
      if (!meshes.count(name)) throw std::runtime_error("LAN package is missing mesh: " + name);
    const bool loaded = LoadContent(manifest_text, {}, &meshes, setup_text);
    WipeMeshes(meshes);
    Wipe(manifest_text);
    Wipe(setup_text);
    return loaded;
  } catch (const std::exception& e) {
    WipeMeshes(meshes);
    Wipe(manifest_text);
    Wipe(setup_text);
    error_ = e.what();
    return false;
  }
}

std::string FusionBundle::SetupJson() const {
  if (!HasBundle()) throw std::runtime_error("Load a Fusion manifest first.");
  json state;
  state["schemaVersion"] = 1;
  state["designName"] = design_name_;
  state["driveLayout"] = drive_layout_ == DriveLayout::TwoWheel ? "2WD" : "4WD";
  state["twoWdAxle"] = two_wd_axle_;
  state["axisAssembly"] = axis_assembly_;
  state["showAxisMarkers"] = show_axis_markers_;
  state["batteryVoltage"] = battery_voltage_;
  state["botRotationDegrees"] = bot_rotation_degrees_;
  state["assemblies"] = json::array();
  for (size_t i = 1; i < assemblies_.size(); ++i) {
    const auto& g = assemblies_[i];
    state["assemblies"].push_back({{"name", g.name}, {"kind", KindLabel(g.kind)},
                                    {"pivotMeters", g.pivot}, {"axis", g.axis},
                                    {"forkRangeDegrees", g.fork_range_degrees},
                                    {"forkCollidesWithFrame", g.fork_collides_with_frame},
                                    {"wheelPosition", g.wheel_position},
                                    {"motor", {
                                      {"enabled", g.motor.enabled},
                                      {"motorType", MotorTypeKey(g.motor.type)},
                                      {"speedInput", MotorSpeedInputKey(g.motor.speed_input)},
                                      {"driveGroup", g.motor.drive_group},
                                      {"motorCount", g.motor.motor_count},
                                      {"kvRpmPerVolt", g.motor.kv_rpm_per_volt},
                                      {"ratedRpm", g.motor.rated_rpm},
                                      {"ratedVoltage", g.motor.rated_voltage},
                                      {"peakCurrentA", g.motor.peak_current_a},
                                      {"phaseResistanceOhm", g.motor.phase_resistance_ohm},
                                      {"armatureResistanceOhm", g.motor.phase_resistance_ohm},
                                      {"escCurrentLimitA", g.motor.esc_current_limit_a},
                                      {"gearRatio", g.motor.gear_ratio_text},
                                      {"gearboxEfficiency", g.motor.gearbox_efficiency}}}});
  }
  state["partAssignments"] = json::object();
  state["partOverrides"] = json::object();
  for (const auto& part : parts_) {
    state["partAssignments"][part.file] = part.assembly;
    state["partOverrides"][part.file] = {{"massKg", part.mass},
      {"massMode", MassModeKey(part.mass_mode)},
      {"manualMassKg", part.manual_mass},
      {"printProfile", PrintProfileJson(part.print_profile)},
      {"buildAxisLocal", part.build_axis_local},
      {"frictionEstimate", part.friction}, {"colorRgba", part.color},
      {"previewVisible", part.preview_visible},
      {"collisionEnabled", part.collision_enabled}};
  }
  return state.dump(2);
}

bool FusionBundle::Save() {
  try {
    if (memory_loaded_)
      throw std::runtime_error("Session-loaded CAD stays in memory and cannot be saved from the app.");
    const std::string state = SetupJson();
    std::ofstream output(manifest_.parent_path() / "combatsim_setup.json", std::ios::trunc);
    if (!output) throw std::runtime_error("Cannot write combatsim_setup.json beside the manifest.");
    output << state << '\n';
    if (!output) throw std::runtime_error("Failed writing combatsim_setup.json.");
    error_.clear();
    return true;
  } catch (const std::exception& e) { error_ = e.what(); return false; }
}

int FusionBundle::ForkCount() const {
  return int(std::count_if(assemblies_.begin(), assemblies_.end(), [](const CadAssembly& assembly) {
    return assembly.kind == AssemblyKind::GravityFork;
  }));
}
bool FusionBundle::IsForkPart(int part_index) const {
  if (part_index < 0 || part_index >= int(parts_.size())) return false;
  const auto& part = parts_[part_index];
  if (part.assembly >= 0 && part.assembly < int(assemblies_.size()) &&
      assemblies_[part.assembly].kind == AssemblyKind::GravityFork) return true;
  return ForkPartPath(part);
}

bool FusionBundle::SetForkCount(int requested) {
  requested = std::clamp(requested, 0, 16);
  int count = ForkCount();
  if (requested == count) {
    fork_message_.clear();
    return true;
  }
  if (count > requested) {
    const int needed = count - requested;
    int removable = 0;
    for (int i = 1; i < int(assemblies_.size()); ++i) {
      if (assemblies_[i].kind == AssemblyKind::GravityFork &&
          std::none_of(parts_.begin(), parts_.end(), [i](const CadPart& part) {
            return part.assembly == i;
          })) ++removable;
    }
    if (removable < needed) {
      fork_message_ = "Reassign CAD bodies from enough fork modules before reducing the fork count.";
      return false;
    }
  }
  while (count < requested) {
    int suffix = 1;
    std::string name;
    do {
      name = "Fork module " + std::to_string(suffix++);
    } while (std::any_of(assemblies_.begin(), assemblies_.end(), [&](const CadAssembly& a) {
      return a.name == name;
    }));
    assemblies_.push_back({name, AssemblyKind::GravityFork, {}, {0, 1, 0}});
    axis_assembly_ = int(assemblies_.size()) - 1;
    ++count;
  }
  while (count > requested) {
    int remove = -1;
    for (int i = int(assemblies_.size()) - 1; i > 0; --i) {
      if (assemblies_[i].kind != AssemblyKind::GravityFork) continue;
      const bool assigned = std::any_of(parts_.begin(), parts_.end(), [i](const CadPart& part) {
        return part.assembly == i;
      });
      if (!assigned) { remove = i; break; }
    }
    if (remove < 0) {
      fork_message_ = "Reassign CAD bodies from a fork module before reducing the fork count.";
      return false;
    }
    if (axis_picking_ && axis_assembly_ == remove) CancelAxis();
    assemblies_.erase(assemblies_.begin() + remove);
    for (auto& part : parts_) if (part.assembly > remove) --part.assembly;
    if (axis_assembly_ > remove) --axis_assembly_;
    else if (axis_assembly_ == remove) axis_assembly_ = 0;
    if (axis_assembly_ <= 0 || axis_assembly_ >= int(assemblies_.size()) ||
        assemblies_[axis_assembly_].kind == AssemblyKind::Fixed) {
      axis_assembly_ = 0;
      for (size_t i = 1; i < assemblies_.size(); ++i)
        if (assemblies_[i].kind != AssemblyKind::Fixed) {
          axis_assembly_ = int(i);
          break;
        }
    }
    --count;
  }
  fork_message_.clear();
  return true;
}

std::array<double, 3> FusionBundle::GroupCenter(int assembly) const {
  Vec3 low, high;
  low.fill(std::numeric_limits<double>::infinity()); high.fill(-std::numeric_limits<double>::infinity());
  for (const auto& part : parts_) if (part.assembly == assembly) {
    Include(part.min_bound, low, high); Include(part.max_bound, low, high);
  }
  if (!std::isfinite(low[0])) return {};
  return Mul(Add(low, high), 0.5);
}

std::array<double, 3> FusionBundle::PreviewCenter() const {
  Vec3 low, high;
  low.fill(std::numeric_limits<double>::infinity());
  high.fill(-std::numeric_limits<double>::infinity());
  for (const auto& part : parts_) {
    Include(part.min_bound, low, high); Include(part.max_bound, low, high);
  }
  return std::isfinite(low[0]) ? Mul(Add(low, high), 0.5) : Vec3{};
}

double FusionBundle::PreviewDistance() const {
  Vec3 low, high;
  low.fill(std::numeric_limits<double>::infinity());
  high.fill(-std::numeric_limits<double>::infinity());
  for (const auto& part : parts_) {
    Include(part.min_bound, low, high); Include(part.max_bound, low, high);
  }
  return std::isfinite(low[0]) ? std::max(0.25, 2.5 * Length(Sub(high, low))) : 1.0;
}

bool FusionBundle::GeneratePreview(std::filesystem::path& output) {
  try {
    if (!HasBundle()) throw std::runtime_error("Load a Fusion manifest first.");
    if (memory_loaded_)
      throw std::runtime_error("Session-loaded CAD is kept in memory and has no disk preview.");
    std::ostringstream xml; xml.imbue(std::locale::classic());
    xml << "<mujoco model=\"CAD setup\"><asset>\n";
    for (size_t i = 0; i < parts_.size(); ++i) {
      const auto& part = parts_[i];
      xml << "<mesh name=\"cad_mesh_" << i << "\" file=\""
          << Escape(AssetReference(part.absolute_render_file, manifest_.parent_path()))
          << "\" scale=\"" << Text({part.scale, part.scale, part.scale})
          << "\" smoothnormal=\"true\"/>\n";
      for (const auto& texture : part.textures) {
        xml << "<texture name=\"preview_tex_" << i << '_' << Escape(texture.role)
            << "\" type=\"2d\" file=\""
            << Escape(AssetReference(texture.absolute_file, manifest_.parent_path()))
            << "\"/>\n";
      }
      const auto rgb = FindTexture(part, "rgb");
      xml << "<material name=\"preview_surface_" << i << "\" rgba=\""
          << Text4(part.color) << "\" specular=\"" << Number(SurfaceForPart(part).specular)
          << "\" shininess=\"" << Number(SurfaceForPart(part).shininess) << '"';
#if defined(COMBATSIM_USE_FILAMENT)
      const auto pbr = PbrSurfaceForPart(part);
      xml << " metallic=\"" << Number(pbr.metallic) << "\" roughness=\""
          << Number(pbr.roughness) << '"';
      if (!part.textures.empty()) {
        xml << ">\n";
        for (const auto& texture : part.textures)
          xml << "<layer role=\"" << Escape(texture.role) << "\" texture=\"preview_tex_"
              << i << '_' << Escape(texture.role) << "\"/>\n";
        xml << "</material>\n";
      } else {
        xml << "/>\n";
      }
#else
      if (rgb) xml << " texture=\"preview_tex_" << i << "_rgb\"";
      xml << "/>\n";
#endif
    }
    xml << "</asset><worldbody><body name=\"CAD_Preview\">\n";
    for (size_t i = 0; i < parts_.size(); ++i) {
      const auto& part = parts_[i];
      xml << "<geom name=\"cad_part_" << i << "\" type=\"mesh\" mesh=\"cad_mesh_" << i
          << "\" pos=\"" << Text(part.position) << "\" quat=\"" << Text4(part.rotation)
          << "\" material=\"preview_surface_" << i << "\" rgba=\""
          << Text4(part.color) << "\"/>\n";
    }
    xml << "</body></worldbody></mujoco>\n";
    output = manifest_.parent_path() / "combatsim_preview.xml";
    std::ofstream file(output, std::ios::trunc);
    if (!file) throw std::runtime_error("Cannot write CAD preview beside manifest.");
    file << xml.str();
    if (!file) throw std::runtime_error("Failed to write CAD preview.");
    error_.clear(); return true;
  } catch (const std::exception& e) { error_ = e.what(); return false; }
}

bool FusionBundle::Generate(std::filesystem::path& output, FusionBundle* opponent) {
  return GenerateImpl(&output, nullptr, nullptr, opponent);
}

bool FusionBundle::GenerateInMemory(std::string& xml,
                                    std::vector<MemoryBundleAsset>& assets,
                                    FusionBundle* opponent) {
  return GenerateImpl(nullptr, &xml, &assets, opponent);
}

bool FusionBundle::GenerateImpl(std::filesystem::path* output, std::string* xml_output,
                                std::vector<MemoryBundleAsset>* assets,
                                FusionBundle* opponent) {
  try {
    const bool memory_output = xml_output && assets;
    if (memory_output == (output != nullptr))
      throw std::runtime_error("Select either disk or in-memory model generation.");
    if (memory_output) {
      xml_output->clear();
      assets->clear();
    }
    if (!HasBundle()) throw std::runtime_error("Load a Fusion manifest first.");
    std::ostringstream xml;
    auto mesh_asset_name = [memory_output](bool is_opponent, size_t index,
                                          const std::string& extension) {
      return std::string(is_opponent ? "opponent_asset_" : "cad_asset_") +
        std::to_string(index) + extension;
    };
    auto texture_asset_name = [](bool is_opponent, size_t index,
                                const CadPart::Texture& texture) {
      return std::string(is_opponent ? "opponent_texture_" : "cad_texture_") +
        std::to_string(index) + "_" + texture.role +
        std::filesystem::path(texture.file).extension().string();
    };
    auto add_asset = [&](const std::string& name,
                         const std::vector<unsigned char>& resident_bytes,
                         const std::filesystem::path& path,
                         const std::string& label) {
      if (!memory_output) return;
      std::vector<unsigned char> bytes = resident_bytes;
      if (bytes.empty()) {
        std::ifstream input(path, std::ios::binary);
        if (!input) throw std::runtime_error("Cannot read asset for in-memory model: " + label);
        bytes.assign(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
      }
      if (bytes.empty()) throw std::runtime_error("Empty asset for in-memory model: " + label);
      assets->push_back({name, std::move(bytes)});
    };
    auto emit_texture_assets = [&](const CadPart& part, size_t index, bool is_opponent) {
      const std::string prefix = is_opponent ? "opponent_" : "cad_";
      for (const auto& texture : part.textures) {
        const std::string texture_name = prefix + "texture_" + std::to_string(index) +
                                         "_" + texture.role;
        std::string filename;
        if (memory_output) {
          filename = texture_asset_name(is_opponent, index, texture);
        } else if (is_opponent) {
          filename = AssetPath(texture.absolute_file);
        } else {
          filename = AssetReference(texture.absolute_file, manifest_.parent_path());
        }
        xml << "<texture name=\"" << texture_name << "\" type=\"2d\" file=\""
            << Escape(filename) << "\"/>\n";
        add_asset(filename, texture.data, texture.absolute_file, texture.file);
      }
    };
    auto emit_material = [&](const CadPart& part, size_t index, bool is_opponent) {
      const std::string prefix = is_opponent ? "opponent_" : "cad_";
      const std::string material_name = prefix + "surface_" + std::to_string(index);
      const std::string rgb_texture = prefix + "texture_" + std::to_string(index) + "_rgb";
      const auto surface = SurfaceForPart(part);
      xml << "<material name=\"" << material_name << "\" specular=\""
          << Number(surface.specular) << "\" shininess=\""
          << Number(surface.shininess) << '"';
#if defined(COMBATSIM_USE_FILAMENT)
      const auto pbr = PbrSurfaceForPart(part);
      xml << " metallic=\"" << Number(pbr.metallic) << "\" roughness=\""
          << Number(pbr.roughness) << '"';
      if (part.textures.empty()) {
        xml << "/>\n";
      } else {
        xml << ">\n";
        for (const auto& texture : part.textures)
          xml << "<layer role=\"" << texture.role << "\" texture=\"" << prefix
              << "texture_" << index << '_' << texture.role << "\"/>\n";
        xml << "</material>\n";
      }
#else
      if (FindTexture(part, "rgb")) xml << " texture=\"" << rgb_texture << '"';
      xml << "/>\n";
#endif
    };
    for (size_t i = 0; i < parts_.size(); ++i)
      if (PartIncludedInSimulation(int(i)) && !ValidMass(parts_[i].mass))
        throw std::runtime_error("Set a positive mass for included body: " + parts_[i].name +
                                 " (" + parts_[i].file + ")");
    const bool has_opponent = opponent && opponent->HasBundle();
    if (has_opponent) {
      for (size_t i = 0; i < opponent->parts_.size(); ++i)
        if (opponent->PartIncludedInSimulation(int(i)) && !ValidMass(opponent->parts_[i].mass))
          throw std::runtime_error("Set a positive mass for included opponent body: " +
            opponent->parts_[i].name + " (" + opponent->parts_[i].file + ")");
      if (!ValidMass(GroupMass(opponent->parts_, 0, {}).mass))
        throw std::runtime_error("Assign at least one opponent body to Main frame with a positive mass.");
    }
    const auto frame = GroupMass(parts_, 0, {});
    if (!ValidMass(frame.mass)) throw std::runtime_error("Assign at least one body with Fusion mass to Main frame.");
    Vec3 low, high;
    low.fill(std::numeric_limits<double>::infinity()); high.fill(-std::numeric_limits<double>::infinity());
    for (size_t i = 0; i < parts_.size(); ++i) if (PartIncludedInSimulation(int(i))) {
      const auto& part = parts_[i];
      Include(part.min_bound, low, high); Include(part.max_bound, low, high);
    }
    const Vec3 center = Mul(Add(low, high), 0.5);
    const Quat player_heading = FacingQuaternion(
      NativeDriveHeading(parts_, assemblies_, *this),
      bot_rotation_degrees_ * 3.14159265358979323846 / 180.0);
    const Vec3 rotated_center = Rotate(player_heading, center);
    const Vec3 spawn{-0.62 - rotated_center[0], -rotated_center[1], 0.012 - low[2]};
    xml.imbue(std::locale::classic());
    xml << "<mujoco model=\"" << Escape(design_name_) << "\">\n"
        << "<compiler angle=\"radian\" autolimits=\"true\"/>\n"
        << "<option timestep=\"0.0004\" gravity=\"0 0 -9.80665\" integrator=\"implicitfast\"/>\n"
        << "<visual><headlight ambient=\"0.12 0.12 0.12\" diffuse=\"0.07 0.07 0.07\" specular=\"0.008 0.008 0.008\"/></visual>\n"
        << "<asset>\n";
    xml << "<material name=\"arena_floor_laminate\" rgba=\"0.84 0.83 0.79 1\" specular=\"0.06\" shininess=\"0.12\" reflectance=\"0\""
#if defined(COMBATSIM_USE_FILAMENT)
        << " metallic=\"0\" roughness=\"0.78\""
#endif
        << "/>\n";
    for (size_t i = 0; i < parts_.size(); ++i) if (PartIncludedInSimulation(int(i))) {
      const auto& part = parts_[i];
      emit_texture_assets(part, i, false);
      emit_material(part, i, false);
      const std::string mesh_file = memory_output
        ? mesh_asset_name(false, i, std::filesystem::path(part.render_file).extension().string())
        : AssetReference(part.absolute_render_file, manifest_.parent_path());
      xml << "<mesh name=\"cad_mesh_" << i << "\" file=\""
          << Escape(mesh_file)
          << "\" scale=\"" << Text({part.scale, part.scale, part.scale})
          << "\" smoothnormal=\"true\"/>\n";
      const auto& render_data = part.render_mesh_data.empty() && part.render_file == part.file
        ? part.mesh_data : part.render_mesh_data;
      add_asset(mesh_file, render_data, part.absolute_render_file, part.render_file);
    }
    if (has_opponent) for (size_t i = 0; i < opponent->parts_.size(); ++i)
      if (opponent->PartIncludedInSimulation(int(i))) {
        const auto& part = opponent->parts_[i];
        emit_texture_assets(part, i, true);
        emit_material(part, i, true);
        const std::string mesh_file = memory_output
          ? mesh_asset_name(true, i, std::filesystem::path(part.render_file).extension().string())
          : AssetPath(part.absolute_render_file);
        xml << "<mesh name=\"opponent_mesh_" << i << "\" file=\""
            << Escape(mesh_file)
            << "\" scale=\""
            << Text({part.scale, part.scale, part.scale})
            << "\" smoothnormal=\"true\"/>\n";
        const auto& render_data = part.render_mesh_data.empty() && part.render_file == part.file
          ? part.mesh_data : part.render_mesh_data;
        add_asset(mesh_file, render_data, part.absolute_render_file, part.render_file);
      }
    xml << "</asset>\n<worldbody>\n"
        << "<light name=\"arena_key_light\" directional=\"true\" castshadow=\"false\" pos=\"0 0 2\" dir=\"0.3 -0.4 -1\" diffuse=\"0.88 0.85 0.78\" specular=\"0.18 0.18 0.18\" ambient=\"0.01 0.01 0.01\"/>\n"
        << "<light name=\"arena_fill_light\" directional=\"true\" castshadow=\"false\" pos=\"0 0 2\" dir=\"-0.5 0.3 -1\" diffuse=\"0.42 0.48 0.54\" specular=\"0.035 0.035 0.035\" ambient=\"0.005 0.005 0.005\"/>\n"
        << "<geom name=\"floor\" type=\"plane\" size=\"0 0 0.02\" rgba=\"0 0 0 0\" friction=\"1.4 0.02 0.001\"/>\n"
        << "<geom name=\"floor_visual\" type=\"box\" pos=\"0 0 -0.001\" size=\"1 1 0.001\" material=\"arena_floor_laminate\" contype=\"0\" conaffinity=\"0\"/>\n"
        << "<!-- Hidden deep collision volumes catch fast impacts; the visible acrylic skins are 8 mm thick. -->\n"
        << "<geom name=\"north_wall\" type=\"box\" pos=\"0 1.30 0.5\" size=\"1.62 0.30 0.55\" margin=\"0.01\" solref=\"0.001 1\" solimp=\"0.99 0.999 0.001\" rgba=\"0.5 0.58 0.58 0\"/>\n"
        << "<geom name=\"south_wall\" type=\"box\" pos=\"0 -1.30 0.5\" size=\"1.62 0.30 0.55\" margin=\"0.01\" solref=\"0.001 1\" solimp=\"0.99 0.999 0.001\" rgba=\"0.5 0.58 0.58 0\"/>\n"
        << "<geom name=\"east_wall\" type=\"box\" pos=\"1.30 0 0.5\" size=\"0.30 1.62 0.55\" margin=\"0.01\" solref=\"0.001 1\" solimp=\"0.99 0.999 0.001\" rgba=\"0.5 0.58 0.58 0\"/>\n"
        << "<geom name=\"west_wall\" type=\"box\" pos=\"-1.30 0 0.5\" size=\"0.30 1.62 0.55\" margin=\"0.01\" solref=\"0.001 1\" solimp=\"0.99 0.999 0.001\" rgba=\"0.5 0.58 0.58 0\"/>\n"
        << "<geom name=\"north_wall_acrylic\" type=\"box\" pos=\"0 1.004 0.5\" size=\"1 0.004 0.5\" contype=\"0\" conaffinity=\"0\" rgba=\"0.57 0.66 0.66 0.11\"/>\n"
        << "<geom name=\"south_wall_acrylic\" type=\"box\" pos=\"0 -1.004 0.5\" size=\"1 0.004 0.5\" contype=\"0\" conaffinity=\"0\" rgba=\"0.57 0.66 0.66 0.11\"/>\n"
        << "<geom name=\"east_wall_acrylic\" type=\"box\" pos=\"1.004 0 0.5\" size=\"0.004 1 0.5\" contype=\"0\" conaffinity=\"0\" rgba=\"0.57 0.66 0.66 0.11\"/>\n"
        << "<geom name=\"west_wall_acrylic\" type=\"box\" pos=\"-1.004 0 0.5\" size=\"0.004 1 0.5\" contype=\"0\" conaffinity=\"0\" rgba=\"0.57 0.66 0.66 0.11\"/>\n"
        << "<geom name=\"roof\" type=\"box\" pos=\"0 0 1.125\" size=\"1.62 1.62 0.175\" rgba=\"0.5 0.58 0.58 0\"/>\n"
        << "<geom name=\"roof_acrylic\" type=\"box\" pos=\"0 0 1.004\" size=\"1 1 0.004\" contype=\"0\" conaffinity=\"0\" rgba=\"0.57 0.66 0.66 0.07\"/>\n"
        << "<body name=\"Player_Robot\" pos=\"" << Text(spawn)
        << "\" quat=\"" << Text4(player_heading) << "\">\n"
        << "<freejoint name=\"robot_free\"/>\n"
        << "<inertial pos=\"" << Text(frame.center) << "\" mass=\"" << Number(frame.mass)
        << "\" fullinertia=\"" << TextInertia(frame.inertia) << "\"/>\n";
    auto geom = [&](size_t i, Vec3 origin) {
      const auto& p = parts_[i];
      xml << "<geom name=\"cad_part_" << i << "\" type=\"mesh\" mesh=\"cad_mesh_" << i
          << "\" pos=\"" << Text(Sub(p.position, origin)) << "\" quat=\"" << Text4(p.rotation)
          << "\" material=\"cad_surface_" << i << "\" mass=\"0\" friction=\"" << Number(p.friction)
          << " 0.02 0.001\" rgba=\"" << Text4(p.color) << "\"";
      if (!p.collision_enabled) xml << " contype=\"0\" conaffinity=\"0\"";
      xml << "/>\n";
    };
    for (size_t i = 0; i < parts_.size(); ++i) if (parts_[i].assembly == 0) geom(i, {});
    for (size_t group = 1; group < assemblies_.size(); ++group) {
      if (!AssemblyEnabled(int(group))) continue;
      const auto& a = assemblies_[group];
      const auto props = GroupMass(parts_, int(group), a.pivot);
      if (props.mass <= 0) continue;
      xml << "<body name=\"cad_assembly_" << group << "\" pos=\"" << Text(a.pivot) << "\">\n"
          << "<inertial pos=\"" << Text(props.center) << "\" mass=\"" << Number(props.mass)
          << "\" fullinertia=\"" << TextInertia(props.inertia) << "\"/>\n";
      if (a.kind != AssemblyKind::Fixed) {
        Vec3 axis = a.axis;
        const double length = Length(axis);
        if (length < 1e-8) throw std::runtime_error("Rotation axis cannot be zero: " + a.name);
        axis = Mul(axis, 1.0/length);
        xml << "<joint name=\"cad_joint_" << group << "\" type=\"hinge\" axis=\""
            << Text(axis) << "\" damping=\""
            << Number(a.kind == AssemblyKind::GravityFork ? 0.02 : 0.00001) << "\"";
        if (a.kind == AssemblyKind::GravityFork) {
          if (!std::isfinite(a.fork_range_degrees[0]) || !std::isfinite(a.fork_range_degrees[1]) ||
              a.fork_range_degrees[0] >= a.fork_range_degrees[1])
            throw std::runtime_error("Fork travel limits must be finite and lower < upper: " + a.name);
          constexpr double degrees_to_radians = 3.14159265358979323846 / 180.0;
          xml << " limited=\"true\" range=\""
              << Number(a.fork_range_degrees[0] * degrees_to_radians) << ' '
              << Number(a.fork_range_degrees[1] * degrees_to_radians) << "\"";
        }
        xml << "/>\n";
      }
      for (size_t i = 0; i < parts_.size(); ++i) if (parts_[i].assembly == int(group)) geom(i, a.pivot);
      xml << "</body>\n";
    }
    xml << "</body>\n";
    if (has_opponent) {
      Vec3 opponent_low, opponent_high;
      opponent_low.fill(std::numeric_limits<double>::infinity());
      opponent_high.fill(-std::numeric_limits<double>::infinity());
      for (const auto& part : opponent->parts_) if (part.assembly >= 0) {
        Include(part.min_bound, opponent_low, opponent_high);
        Include(part.max_bound, opponent_low, opponent_high);
      }
      const Vec3 opponent_origin{
        (opponent_low[0] + opponent_high[0]) * 0.5,
        (opponent_low[1] + opponent_high[1]) * 0.5,
        opponent_low[2]};
      const auto opponent_frame = GroupMass(opponent->parts_, 0, opponent_origin);
      const Quat opponent_heading = FacingQuaternion(
        NativeDriveHeading(opponent->parts_, opponent->assemblies_, *opponent),
        3.14159265358979323846 +
          opponent->bot_rotation_degrees_ * 3.14159265358979323846 / 180.0);
      xml << "<body name=\"Target_Robot\" pos=\"0.55 0 0.012\" quat=\""
          << Text4(opponent_heading) << "\">\n"
          << "<freejoint name=\"target_free\"/>\n"
          << "<inertial pos=\"" << Text(opponent_frame.center) << "\" mass=\""
          << Number(opponent_frame.mass) << "\" fullinertia=\""
          << TextInertia(opponent_frame.inertia) << "\"/>\n";
      auto opponent_geom = [&](size_t i, Vec3 local_origin) {
        const auto& part = opponent->parts_[i];
        xml << "<geom name=\"opponent_part_" << i << "\" type=\"mesh\" mesh=\"opponent_mesh_"
            << i << "\" pos=\"" << Text(Sub(part.position, local_origin)) << "\" quat=\""
            << Text4(part.rotation) << "\" material=\"opponent_surface_" << i
            << "\" mass=\"0\" friction=\"" << Number(part.friction)
            << " 0.02 0.001\" rgba=\"" << Text4(part.color) << "\"";
        if (!part.collision_enabled) xml << " contype=\"0\" conaffinity=\"0\"";
        xml << "/>\n";
      };
      for (size_t i = 0; i < opponent->parts_.size(); ++i)
        if (opponent->parts_[i].assembly == 0) opponent_geom(i, opponent_origin);
      for (size_t group = 1; group < opponent->assemblies_.size(); ++group) {
        if (!opponent->AssemblyEnabled(int(group))) continue;
        const auto& assembly = opponent->assemblies_[group];
        const auto properties = GroupMass(opponent->parts_, int(group), assembly.pivot);
        if (properties.mass <= 0) continue;
        xml << "<body name=\"opponent_assembly_" << group << "\" pos=\""
            << Text(Sub(assembly.pivot, opponent_origin)) << "\">\n"
            << "<inertial pos=\"" << Text(properties.center) << "\" mass=\""
            << Number(properties.mass) << "\" fullinertia=\""
            << TextInertia(properties.inertia) << "\"/>\n";
        if (assembly.kind != AssemblyKind::Fixed) {
          Vec3 axis = assembly.axis;
          const double length = Length(axis);
          if (length < 1e-8)
            throw std::runtime_error("Opponent rotation axis cannot be zero: " + assembly.name);
          axis = Mul(axis, 1.0 / length);
          xml << "<joint name=\"opponent_joint_" << group
              << "\" type=\"hinge\" axis=\"" << Text(axis) << "\" damping=\""
              << Number(assembly.kind == AssemblyKind::GravityFork ? 0.02 : 0.00001) << "\"";
          if (assembly.kind == AssemblyKind::GravityFork) {
            if (!std::isfinite(assembly.fork_range_degrees[0]) ||
                !std::isfinite(assembly.fork_range_degrees[1]) ||
                assembly.fork_range_degrees[0] >= assembly.fork_range_degrees[1])
              throw std::runtime_error("Opponent fork travel limits must be finite and lower < upper: " + assembly.name);
            constexpr double degrees_to_radians = 3.14159265358979323846 / 180.0;
            xml << " limited=\"true\" range=\""
                << Number(assembly.fork_range_degrees[0] * degrees_to_radians) << ' '
                << Number(assembly.fork_range_degrees[1] * degrees_to_radians) << "\"";
          }
          xml << "/>\n";
        }
        for (size_t i = 0; i < opponent->parts_.size(); ++i)
          if (opponent->parts_[i].assembly == int(group))
            opponent_geom(i, assembly.pivot);
        xml << "</body>\n";
      }
      xml << "</body>\n";
    } else {
      xml << "<body name=\"Target_Robot\" pos=\"0.55 0 0.08\" quat=\"0 0 0 1\">"
          << "<freejoint/><geom name=\"target_chassis\" type=\"box\" size=\"0.17 0.13 0.045\" mass=\"0.5\" rgba=\"0.82 0.16 0.14 1\"/>"
          << "</body>\n";
    }
    xml << "</worldbody>\n<contact>\n";
    // Chassis, fixed structural parts, and forks can all scrape the floor.
    // Override only those floor contacts so drive wheels and weapon impacts
    // keep their normal contact response against other robot parts.
    auto add_floor_scrape_pairs = [&](const FusionBundle& robot, const char* prefix) {
      for (size_t i = 0; i < robot.parts_.size(); ++i) {
        const auto& part = robot.parts_[i];
        if (!robot.PartIncludedInSimulation(int(i)) || !part.collision_enabled) continue;
        const bool is_fork = robot.IsForkPart(int(i));
        const bool is_frame = part.assembly == 0;
        const bool is_fixed_structure = part.assembly > 0 &&
          size_t(part.assembly) < robot.assemblies_.size() &&
          robot.assemblies_[part.assembly].kind == AssemblyKind::Fixed;
        if (!is_frame && !is_fixed_structure && !is_fork) continue;
        // Match MuJoCo's usual equal-priority floor/geom friction mix for the
        // frame. Forks retain their explicit CAD friction estimate.
        const double friction = is_fork ? part.friction : std::max(1.4, part.friction);
        xml << "<pair geom1=\"floor\" geom2=\"" << prefix << i
            << "\" condim=\"3\" friction=\"" << Number(friction)
            << " 0.02 0.001\" solref=\"0.02 2\"/>\n";
      }
    };
    add_floor_scrape_pairs(*this, "cad_part_");
    if (has_opponent) add_floor_scrape_pairs(*opponent, "opponent_part_");
    // A fork is a child hinge of Player_Robot. MuJoCo filters parent-child
    // contacts by default, so explicitly pair each fork geom with each main-
    // frame geom to provide physical contact without changing other filters.
    for (size_t group = 1; group < assemblies_.size(); ++group) {
      if (!AssemblyEnabled(int(group))) continue;
      if (assemblies_[group].kind != AssemblyKind::GravityFork ||
          !assemblies_[group].fork_collides_with_frame ||
          GroupMass(parts_, int(group), assemblies_[group].pivot).mass <= 0) continue;
      for (size_t fork_part = 0; fork_part < parts_.size(); ++fork_part) {
        if (parts_[fork_part].assembly != int(group) ||
            !parts_[fork_part].collision_enabled) continue;
        for (size_t frame_part = 0; frame_part < parts_.size(); ++frame_part) {
          if (parts_[frame_part].assembly != 0 ||
              !parts_[frame_part].collision_enabled) continue;
          const double friction = std::sqrt(std::max(0.0, parts_[fork_part].friction) *
                                            std::max(0.0, parts_[frame_part].friction));
          xml << "<pair geom1=\"cad_part_" << fork_part << "\" geom2=\"cad_part_"
              << frame_part << "\" condim=\"3\" friction=\"" << Number(friction)
              << " 0.02 0.001\" solref=\"0.003 1\"/>\n";
        }
      }
    }
    if (has_opponent) {
      // MuJoCo excludes contacts between a fork and its parent frame by default.
      // Pair configured opponent fork geoms with the opponent frame explicitly.
      for (size_t group = 1; group < opponent->assemblies_.size(); ++group) {
        if (!opponent->AssemblyEnabled(int(group))) continue;
        const auto& fork = opponent->assemblies_[group];
        if (fork.kind != AssemblyKind::GravityFork || !fork.fork_collides_with_frame ||
            GroupMass(opponent->parts_, int(group), fork.pivot).mass <= 0) continue;
        for (size_t fork_part = 0; fork_part < opponent->parts_.size(); ++fork_part) {
          if (opponent->parts_[fork_part].assembly != int(group) ||
              !opponent->parts_[fork_part].collision_enabled) continue;
          for (size_t frame_part = 0; frame_part < opponent->parts_.size(); ++frame_part) {
            if (opponent->parts_[frame_part].assembly != 0 ||
                !opponent->parts_[frame_part].collision_enabled) continue;
            const double friction = std::sqrt(
              std::max(0.0, opponent->parts_[fork_part].friction) *
              std::max(0.0, opponent->parts_[frame_part].friction));
            xml << "<pair geom1=\"opponent_part_" << fork_part
                << "\" geom2=\"opponent_part_" << frame_part
                << "\" condim=\"3\" friction=\"" << Number(friction)
                << " 0.02 0.001\" solref=\"0.003 1\"/>\n";
          }
        }
      }
    }
    xml << "</contact>\n<actuator>\n";
    for (size_t group = 1; group < assemblies_.size(); ++group) {
      if (!AssemblyEnabled(int(group))) continue;
      const auto& a = assemblies_[group];
      if (a.kind == AssemblyKind::Fixed || a.kind == AssemblyKind::GravityFork ||
          GroupMass(parts_, int(group), a.pivot).mass <= 0) continue;
      const bool drive_assembly = a.kind == AssemblyKind::LeftWheel ||
                                  a.kind == AssemblyKind::RightWheel;
      const bool weapon_assembly = a.kind == AssemblyKind::Weapon;
      const bool use_motor_model = (drive_assembly || weapon_assembly) && a.motor.enabled;
      if (use_motor_model &&
          (!std::isfinite(battery_voltage_) || battery_voltage_ <= 0.0 ||
           (drive_assembly && (a.motor.drive_group < 0 || a.motor.drive_group > 7)) ||
           a.motor.motor_count < 1 || a.motor.motor_count > 32 ||
           MotorKvRpmPerVolt(a.motor) <= 0.0 ||
           !std::isfinite(a.motor.peak_current_a) || a.motor.peak_current_a <= 0.0 ||
           !std::isfinite(a.motor.phase_resistance_ohm) || a.motor.phase_resistance_ohm < 0.0 ||
           !std::isfinite(a.motor.esc_current_limit_a) || a.motor.esc_current_limit_a < 0.0 ||
           !std::isfinite(a.motor.gear_ratio) || a.motor.gear_ratio <= 0.0 ||
           !std::isfinite(a.motor.gearbox_efficiency) ||
           a.motor.gearbox_efficiency <= 0.0 || a.motor.gearbox_efficiency > 1.0))
        throw std::runtime_error("Invalid motor settings or missing battery voltage for: " + a.name);
      const double force = a.kind == AssemblyKind::Weapon ? 0.25 : 0.08;
      const double gain = a.kind == AssemblyKind::Weapon ? 0.001 : 0.004;
      if (use_motor_model) {
        xml << "<motor name=\"cad_assembly_" << group << "_drive\" joint=\"cad_joint_"
            << group << "\"/><!-- torque-controlled motor model -->\n";
        if (drive_assembly)
          xml << "<velocity name=\"cad_assembly_" << group
              << "_manual_drive\" joint=\"cad_joint_" << group
              << "\" kv=\"0.004\" forcerange=\"-2 2\"/>\n";
      } else {
        xml << "<velocity name=\"cad_assembly_" << group << "_drive\" joint=\"cad_joint_" << group
            << "\" kv=\"" << Number(gain) << "\" forcerange=\"-" << Number(force)
            << ' ' << Number(force) << "\"/>\n";
      }
    }
    if (has_opponent) {
      for (size_t group = 1; group < opponent->assemblies_.size(); ++group) {
        if (!opponent->AssemblyEnabled(int(group))) continue;
        const auto& assembly = opponent->assemblies_[group];
        if (assembly.kind == AssemblyKind::Fixed || assembly.kind == AssemblyKind::GravityFork ||
            GroupMass(opponent->parts_, int(group), assembly.pivot).mass <= 0) continue;
        const bool drive_assembly = assembly.kind == AssemblyKind::LeftWheel ||
                                    assembly.kind == AssemblyKind::RightWheel;
        const bool weapon_assembly = assembly.kind == AssemblyKind::Weapon;
        const bool use_motor_model = (drive_assembly || weapon_assembly) && assembly.motor.enabled;
        if (use_motor_model &&
            (!std::isfinite(opponent->battery_voltage_) || opponent->battery_voltage_ <= 0.0 ||
             (drive_assembly && (assembly.motor.drive_group < 0 || assembly.motor.drive_group > 7)) ||
             assembly.motor.motor_count < 1 || assembly.motor.motor_count > 32 ||
             MotorKvRpmPerVolt(assembly.motor) <= 0.0 ||
             !std::isfinite(assembly.motor.peak_current_a) || assembly.motor.peak_current_a <= 0.0 ||
             !std::isfinite(assembly.motor.phase_resistance_ohm) || assembly.motor.phase_resistance_ohm < 0.0 ||
             !std::isfinite(assembly.motor.esc_current_limit_a) || assembly.motor.esc_current_limit_a < 0.0 ||
             !std::isfinite(assembly.motor.gear_ratio) || assembly.motor.gear_ratio <= 0.0 ||
             !std::isfinite(assembly.motor.gearbox_efficiency) ||
             assembly.motor.gearbox_efficiency <= 0.0 || assembly.motor.gearbox_efficiency > 1.0))
          throw std::runtime_error("Invalid opponent motor settings or missing battery voltage for: " + assembly.name);
        const double force = assembly.kind == AssemblyKind::Weapon ? 0.25 : 0.08;
        const double gain = assembly.kind == AssemblyKind::Weapon ? 0.001 : 0.004;
        if (use_motor_model) {
          xml << "<motor name=\"opponent_assembly_" << group
              << "_drive\" joint=\"opponent_joint_" << group
              << "\"/><!-- opponent torque-controlled motor model -->\n";
          if (drive_assembly)
            xml << "<velocity name=\"opponent_assembly_" << group
                << "_manual_drive\" joint=\"opponent_joint_" << group
                << "\" kv=\"0.004\" forcerange=\"-2 2\"/>\n";
        } else {
          xml << "<velocity name=\"opponent_assembly_" << group
              << "_drive\" joint=\"opponent_joint_" << group << "\" kv=\""
              << Number(gain) << "\" forcerange=\"-" << Number(force)
              << ' ' << Number(force) << "\"/>\n";
        }
      }
    }
    xml << "</actuator>\n</mujoco>\n";
    if (memory_output) {
      *xml_output = xml.str();
      error_.clear();
      return true;
    }
    *output = manifest_.parent_path() / "combatsim_generated.xml";
    std::ofstream file(*output, std::ios::trunc);
    if (!file) throw std::runtime_error("Cannot write generated MJCF beside manifest.");
    file << xml.str();
    if (!file) throw std::runtime_error("Failed to write generated MJCF.");
    error_.clear();
    return true;
  } catch (const std::exception& e) { error_ = e.what(); return false; }
}

void FusionBundle::BindControls(const mjModel* model,
                                std::vector<std::pair<int, double>>& left,
                                std::vector<std::pair<int, double>>& right,
                                std::vector<int>& weapon,
                                std::map<int, MotorSpec>& motor_models,
                                std::map<int, MotorSpec>& weapon_motor_models,
                                std::map<int, int>& manual_velocity_actuators,
                                double& battery_voltage, bool opponent) const {
  motor_models.clear();
  weapon_motor_models.clear();
  manual_velocity_actuators.clear();
  battery_voltage = battery_voltage_;
  const std::string actuator_prefix = opponent ? "opponent_assembly_" : "cad_assembly_";
  const Vec3 drive_heading = NativeDriveHeading(parts_, assemblies_, *this);
  for (size_t group = 1; group < assemblies_.size(); ++group) {
    if (!AssemblyEnabled(int(group))) continue;
    const auto kind = assemblies_[group].kind;
    const auto name = actuator_prefix + std::to_string(group) + "_drive";
    const int id = mj_name2id(model, mjOBJ_ACTUATOR, name.c_str());
    if (id < 0) continue;
    if (kind == AssemblyKind::LeftWheel || kind == AssemblyKind::RightWheel) {
      if (assemblies_[group].motor.enabled) {
        motor_models[id] = assemblies_[group].motor;
        const auto manual_name = actuator_prefix + std::to_string(group) + "_manual_drive";
        const int manual_id = mj_name2id(model, mjOBJ_ACTUATOR, manual_name.c_str());
        if (manual_id >= 0) manual_velocity_actuators[id] = manual_id;
      }
      Vec3 axis = assemblies_[group].axis;
      const double axis_length = Length(axis);
      double forward_sign = 1.0;
      if (axis_length > 1e-8) {
        axis = Mul(axis, 1.0/axis_length);
        // Forward body motion opposes the wheel's bottom-contact velocity.
        // Derive forward from the assigned left/right wheel layout so a CAD
        // model whose native forward axis is Y is handled like an X-forward bot.
        const double forward_speed = Dot(Cross(axis, {0, 0, -1}), drive_heading);
        if (std::abs(forward_speed) > 1e-6) forward_sign = forward_speed > 0 ? -1.0 : 1.0;
      }
      if (kind == AssemblyKind::LeftWheel) left.emplace_back(id, forward_sign);
      if (kind == AssemblyKind::RightWheel) right.emplace_back(id, forward_sign);
    }
    if (kind == AssemblyKind::Weapon) {
      weapon.push_back(id);
      if (assemblies_[group].motor.enabled)
        weapon_motor_models[id] = assemblies_[group].motor;
    }
  }
}

double FusionBundle::EstimatedWeaponSpinupSeconds() const {
  bool found_weapon = false;
  double slowest_seconds = 0.0;
  for (size_t i = 1; i < assemblies_.size(); ++i) {
    const auto& assembly = assemblies_[i];
    if (assembly.kind != AssemblyKind::Weapon) continue;
    found_weapon = true;
    if (!assembly.motor.enabled) return 0.0;
    const MassProperties properties = GroupMass(parts_, int(i), assembly.pivot);
    const double inertia = MomentAboutAxis(properties, assembly.axis);
    const double seconds = EstimateMotorSpinupSeconds(
      assembly.motor, battery_voltage_, inertia);
    if (!std::isfinite(seconds) || seconds <= 0.0) return 0.0;
    slowest_seconds = std::max(slowest_seconds, seconds);
  }
  return found_weapon ? slowest_seconds : 0.0;
}

void FusionBundle::PickFace(int part_index, const std::array<double, 3>& cad_point) {
  if (part_index < 0 || part_index >= int(parts_.size())) return;
  selected_part_ = part_index;
  const auto& part = parts_[part_index];
  if (part.assembly <= 0) {
    pick_message_ = "Selected " + part.name + ". Assign it to a weapon or wheel assembly first.";
    return;
  }
  auto file = OpenMesh(part);
  if (!file) return;
  file->seekg(80);
  uint32_t count = 0; file->read(reinterpret_cast<char*>(&count), 4);
  double nearest = std::numeric_limits<double>::infinity();
  Vec3 center{};
  for (uint32_t t = 0; t < count && *file; ++t) {
    char triangle[50]; file->read(triangle, sizeof(triangle));
    if (!*file) break;
    Vec3 sum{};
    for (int v = 0; v < 3; ++v) {
      float xyz[3]; std::memcpy(xyz, triangle + 12 + v*12, 12);
      sum = Add(sum, Vec3{xyz[0], xyz[1], xyz[2]});
    }
    Vec3 point = Add(part.position, Rotate(part.rotation, Mul(sum, part.scale/3.0)));
    const double distance = Length(Sub(point, cad_point));
    if (distance < nearest) { nearest = distance; center = point; }
  }
  if (std::isfinite(nearest)) {
    assemblies_[part.assembly].pivot = center;
    pick_message_ = "Set " + assemblies_[part.assembly].name + " pivot to the selected triangle center.";
  }
}

void FusionBundle::SelectPart(int part_index) {
  if (part_index < 0 || part_index >= int(parts_.size())) return;
  selected_part_ = part_index;
  focus_part_request_ = part_index;
  focus_frames_remaining_ = 2;
  pick_message_ = "Selected " + parts_[part_index].name + ". Choose its rigid assembly below.";
}

void FusionBundle::UpdateHover(int part_index, const std::array<double, 3>& hit) {
  hovered_hit_ = hit;
  const bool valid_part = PreviewVisible(part_index);
  // The axis target is independent of the CAD part supplying the reference hole.
  // This also prevents the body hover highlight from competing with the rim cue.
  hovered_part_ = axis_picking_ ? (valid_part ? part_index : -1) :
                                  (valid_part ? part_index : -1);
  hovered_loop_.clear();
  if (!axis_picking_ || hovered_part_ < 0) return;
  auto [found, inserted] = hole_cache_.try_emplace(hovered_part_);
  if (inserted) found->second = ReadFeatureLoops(parts_[hovered_part_]);
  if (found->second.empty()) {
    pick_message_ = "No closed rim loops found on " + parts_[hovered_part_].name +
                    ". Check that its STL has a clear hole edge.";
    return;
  }
  const double limit = std::clamp(0.06*Length(Sub(
    parts_[hovered_part_].max_bound, parts_[hovered_part_].min_bound)), 0.003, 0.012);
  double nearest = limit;
  for (const auto& loop : found->second) {
    const double distance = DistanceToLoop(loop, hit);
    if (distance < nearest) { nearest = distance; hovered_loop_ = loop; }
  }
  if (hovered_loop_.empty())
    pick_message_ = "Move closer to a closed rim on " + parts_[hovered_part_].name + ".";
  else
    pick_message_ = "Rim highlighted on " + parts_[hovered_part_].name +
                    ". Click to fit " + assemblies_[axis_assembly_].name + " axis.";
}

void FusionBundle::StartPrintBedFacePick(int part_index) {
  if (part_index < 0 || part_index >= int(parts_.size())) return;
  axis_picking_ = false;
  axis_points_.clear();
  hovered_loop_.clear();
  print_bed_face_picking_ = true;
  print_bed_face_part_ = part_index;
  selected_part_ = part_index;
  hovered_part_ = -1;
  pick_message_ = "Click a face on this part that will sit on the build plate. The build axis points into the part.";
}

bool FusionBundle::SelectHoveredPrintBedFace() {
  if (!print_bed_face_picking_) return false;
  if (hovered_part_ != print_bed_face_part_ || !PreviewVisible(hovered_part_)) {
    pick_message_ = "Click a visible face on the selected part to set its bed contact.";
    return false;
  }
  auto& part = parts_[print_bed_face_part_];
  Vec3 axis{};
  std::string error;
  if (!FindPrintBedAxis(part, hovered_hit_, axis, error)) {
    pick_message_ = error;
    return false;
  }
  part.build_axis_local = axis;
  print_bed_face_picking_ = false;
  print_bed_face_part_ = -1;
  if (part.mass_mode == PartMassMode::Printed) RefreshPrintEstimate(part);
  pick_message_ = "Build direction set from the selected bed face on " + part.name + ".";
  return true;
}

void FusionBundle::CancelPrintBedFacePick() {
  print_bed_face_picking_ = false;
  print_bed_face_part_ = -1;
  pick_message_ = "Bed-face selection cancelled.";
}

bool FusionBundle::SelectHoveredHole() {
  if (!axis_picking_ || hovered_loop_.size() < 6) {
    pick_message_ = "Move over a highlighted closed hole rim, then click it.";
    return false;
  }
  selected_part_ = hovered_part_;
  axis_points_ = hovered_loop_;
  return FinishAxis();
}

bool FusionBundle::PreviewVisible(int part_index) const {
  return part_index >= 0 && part_index < int(parts_.size()) && parts_[part_index].preview_visible;
}

void FusionBundle::StartAxis(int assembly) {
  if (assembly <= 0 || assembly >= int(assemblies_.size()) || !AssemblyEnabled(assembly) ||
      assemblies_[assembly].kind == AssemblyKind::Fixed) {
    pick_message_ = "Choose a weapon, wheel, or gravity-fork assembly for the rotation axis.";
    return;
  }
  axis_assembly_ = assembly;
  print_bed_face_picking_ = false;
  print_bed_face_part_ = -1;
  axis_points_.clear(); axis_picking_ = true;
  hovered_part_ = -1;
  hovered_loop_.clear();
  pick_message_ = "Hover a hole rim until its complete loop lights up, then click once.";
}

bool FusionBundle::AddAxisPoint(int part_index, const std::array<double, 3>& cad_point,
                                bool snap_to_mesh_edge) {
  if (!axis_picking_ || part_index < 0 || part_index >= int(parts_.size())) return false;
  selected_part_ = part_index;
  if (axis_assembly_ <= 0 || axis_assembly_ >= int(assemblies_.size()) ||
      !AssemblyEnabled(axis_assembly_)) {
    pick_message_ = "Choose a rotating assembly to receive the fitted axis.";
    return false;
  }
  Vec3 point = cad_point;
  if (snap_to_mesh_edge && !SnapToFeatureEdge(parts_[part_index], cad_point, point)) {
    pick_message_ = "Could not find an STL edge near this part.";
    return false;
  }
  for (const auto& existing : axis_points_)
    if (Length(Sub(point, existing)) < 1e-5) {
      pick_message_ = "That edge is already in the chain. Click the next edge around the hole.";
      return false;
    }
  axis_points_.push_back(point);
  pick_message_ = "Hole outline: " + std::to_string(axis_points_.size()) +
                  " points. Reference part stays assigned as-is; click around the rim, "
                  "then press Fit axis through hole.";
  return true;
}

void FusionBundle::CancelAxis() {
  axis_points_.clear(); axis_picking_ = false;
  pick_message_ = "Axis selection cancelled.";
}

bool FusionBundle::FinishAxis() {
  if (axis_points_.size() < 3 || axis_assembly_ <= 0 || axis_assembly_ >= int(assemblies_.size())) {
    pick_message_ = "Select at least three points around one hole before fitting its axis.";
    return false;
  }
  Vec3 mean{};
  for (const auto& point : axis_points_) mean = Add(mean, point);
  mean = Mul(mean, 1.0/axis_points_.size());
  Vec3 normal{};
  for (size_t i = 0; i < axis_points_.size(); ++i)
    normal = Add(normal, Cross(Sub(axis_points_[i], mean),
                               Sub(axis_points_[(i+1)%axis_points_.size()], mean)));
  const double normal_length = Length(normal);
  if (normal_length < 1e-10) {
    pick_message_ = "Selected points are nearly in a line. Click around the full hole.";
    return false;
  }
  normal = Mul(normal, 1.0/normal_length);
  Vec3 u{};
  for (const auto& point : axis_points_) {
    u = Sub(point, mean);
    u = Sub(u, Mul(normal, Dot(u, normal)));
    if (Length(u) > 1e-8) break;
  }
  if (Length(u) < 1e-8) return false;
  u = Mul(u, 1.0/Length(u));
  const Vec3 v = Cross(normal, u);
  double radius_scale = 0;
  for (const auto& point : axis_points_)
    radius_scale = std::max(radius_scale, Length(Sub(point, mean)));
  if (radius_scale < 1e-8) return false;
  double equations[3][4]{};
  for (const auto& point : axis_points_) {
    const Vec3 offset = Sub(point, mean);
    const double x = Dot(offset, u)/radius_scale;
    const double y = Dot(offset, v)/radius_scale;
    const double row[3] = {2*x, 2*y, 1};
    const double rhs = x*x + y*y;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) equations[r][c] += row[r]*row[c];
      equations[r][3] += row[r]*rhs;
    }
  }
  double circle[3]{};
  if (!Solve3(equations, circle)) {
    pick_message_ = "Could not fit a circle. Click points around more of the hole.";
    return false;
  }
  const Vec3 center = Add(mean, Add(Mul(u, circle[0]*radius_scale),
                                    Mul(v, circle[1]*radius_scale)));
  auto& assembly = assemblies_[axis_assembly_];
  assembly.pivot = center;
  assembly.axis = normal;
  pick_message_ = "Fitted " + assembly.name + " axis at (" +
    std::to_string(center[0]) + ", " + std::to_string(center[1]) + ", " +
    std::to_string(center[2]) + ") m; direction (" + std::to_string(normal[0]) + ", " +
    std::to_string(normal[1]) + ", " + std::to_string(normal[2]) + "). Click Apply and preview to rebuild the simulation.";
  axis_points_.clear(); axis_picking_ = false;
  return true;
}

void FusionBundle::PickEdge(int part_index, const std::array<double, 3>& cad_point) {
  if (part_index < 0 || part_index >= int(parts_.size())) return;
  selected_part_ = part_index;
  const auto& part = parts_[part_index];
  if (part.assembly <= 0) {
    pick_message_ = "Assign " + part.name + " to a rotating assembly first.";
    return;
  }
  auto file = OpenMesh(part);
  if (!file) return;
  file->seekg(80);
  uint32_t count = 0; file->read(reinterpret_cast<char*>(&count), 4);
  double nearest = std::numeric_limits<double>::infinity();
  Vec3 midpoint{}, direction{};
  for (uint32_t t = 0; t < count && *file; ++t) {
    char triangle[50]; file->read(triangle, sizeof(triangle));
    if (!*file) break;
    Vec3 vertices[3];
    for (int v = 0; v < 3; ++v) {
      float xyz[3]; std::memcpy(xyz, triangle + 12 + v*12, 12);
      vertices[v] = Add(part.position, Rotate(part.rotation,
        Mul(Vec3{xyz[0], xyz[1], xyz[2]}, part.scale)));
    }
    for (int e = 0; e < 3; ++e) {
      const Vec3 a = vertices[e], b = vertices[(e+1)%3];
      const Vec3 edge = Sub(b, a);
      const double length2 = std::pow(Length(edge), 2);
      if (length2 < 1e-16) continue;
      const Vec3 delta = Sub(cad_point, a);
      const double dot = delta[0]*edge[0] + delta[1]*edge[1] + delta[2]*edge[2];
      const Vec3 closest = Add(a, Mul(edge, std::clamp(dot/length2, 0.0, 1.0)));
      const double distance = Length(Sub(closest, cad_point));
      if (distance < nearest) {
        nearest = distance; midpoint = Mul(Add(a, b), 0.5);
        direction = Mul(edge, 1.0/std::sqrt(length2));
      }
    }
  }
  if (std::isfinite(nearest)) {
    assemblies_[part.assembly].pivot = midpoint;
    assemblies_[part.assembly].axis = direction;
    pick_message_ = "Set " + assemblies_[part.assembly].name + " axis through the selected mesh edge midpoint.";
  }
}

void FusionBundle::AddAxisMarkers(const mjModel* model, const mjData* data,
                                  mjvScene& scene, bool opponent) const {
  auto append_geom = [&]() -> mjvGeom* {
    if (scene.ngeom >= scene.maxgeom || !scene.geoms || !scene.geomorder) return nullptr;
    const int index = scene.ngeom++;
    scene.geomorder[index] = index;
    mjvGeom& geom = scene.geoms[index];
    geom.objtype = mjOBJ_UNKNOWN;
    geom.objid = -1;
    geom.category = mjCAT_DECOR;
    return &geom;
  };
  for (size_t group = 1; group < assemblies_.size(); ++group) {
    if (!AssemblyEnabled(int(group))) continue;
    const auto& assembly = assemblies_[group];
    const bool assigned = std::any_of(parts_.begin(), parts_.end(), [group](const CadPart& part) {
      return part.assembly == int(group);
    });
    if (!assigned) continue;
    if (assembly.kind == AssemblyKind::Fixed || scene.ngeom + 2 > scene.maxgeom) continue;
    const std::string body_name = (opponent ? "opponent_assembly_" : "cad_assembly_") +
      std::to_string(group);
    const int body = mj_name2id(model, mjOBJ_BODY, body_name.c_str());
    if (body < 0) continue;
    const double axis_length = Length(assembly.axis);
    if (axis_length < 1e-8) continue;
    const mjtNum* origin = data->xpos + body*3;
    const mjtNum* rotation = data->xmat + body*9;
    const Vec3 axis = Mul(assembly.axis, 1.0/axis_length);
    mjtNum start[3], end[3];
    for (int i = 0; i < 3; ++i) {
      const double component = rotation[i*3]*axis[0] + rotation[i*3+1]*axis[1] + rotation[i*3+2]*axis[2];
      start[i] = origin[i] - 0.08*component;
      end[i] = origin[i] + 0.08*component;
    }
    const float weapon_color[4] = {1.0f, 0.68f, 0.12f, 1.0f};
    const float wheel_color[4] = {0.15f, 0.9f, 1.0f, 1.0f};
    const float fork_color[4] = {0.8f, 0.35f, 1.0f, 1.0f};
    const float* color = assembly.kind == AssemblyKind::Weapon ? weapon_color :
      (assembly.kind == AssemblyKind::GravityFork ? fork_color : wheel_color);
    if (mjvGeom* line = append_geom()) {
      mjv_initGeom(line, mjGEOM_LINE, nullptr, nullptr, nullptr, color);
      mjv_connector(line, mjGEOM_LINE, 3, start, end);
    }
    const mjtNum size[3] = {0.009, 0.009, 0.009};
    if (mjvGeom* marker = append_geom())
      mjv_initGeom(marker, mjGEOM_SPHERE, size, origin, nullptr, color);
  }
}

void FusionBundle::AddSetupAxisMarkers(mjvScene& scene,
                                       std::array<double, 3> camera_position,
                                       bool show_assembly_axes) const {
  auto append_geom = [&]() -> mjvGeom* {
    if (scene.ngeom >= scene.maxgeom || !scene.geoms || !scene.geomorder) return nullptr;
    const int index = scene.ngeom++;
    scene.geomorder[index] = index;
    mjvGeom& geom = scene.geoms[index];
    geom.objtype = mjOBJ_UNKNOWN;
    geom.objid = -1;
    geom.category = mjCAT_DECOR;
    return &geom;
  };
  if (axis_picking_ && hovered_loop_.size() >= 3) {
    const float hover_color[4] = {0.16f, 1.0f, 0.28f, 1.0f};
    Vec3 loop_center{};
    for (const auto& point : hovered_loop_) loop_center = Add(loop_center, point);
    loop_center = Mul(loop_center, 1.0 / hovered_loop_.size());
    Vec3 toward_camera = Sub(camera_position, loop_center);
    const double camera_distance = Length(toward_camera);
    if (camera_distance > 1e-9) {
      const auto& part = parts_[hovered_part_];
      const double size = Length(Sub(part.max_bound, part.min_bound));
      const double offset = std::clamp(size * 0.0025, 0.0001, 0.0005);
      toward_camera = Mul(toward_camera, offset / camera_distance);
    } else {
      toward_camera = {};
    }
    for (size_t i = 0; i < hovered_loop_.size() && scene.ngeom < scene.maxgeom; ++i) {
      const auto first = Add(hovered_loop_[i], toward_camera);
      const auto second = Add(hovered_loop_[(i+1)%hovered_loop_.size()], toward_camera);
      mjtNum a[3] = {first[0], first[1], first[2]};
      mjtNum b[3] = {second[0], second[1], second[2]};
      if (mjvGeom* line = append_geom()) {
        mjv_initGeom(line, mjGEOM_LINE, nullptr, nullptr, nullptr, hover_color);
        mjv_connector(line, mjGEOM_LINE, 7, a, b);
      }
    }
  }
  const float picked_color[4] = {0.25f, 1.0f, 0.3f, 1.0f};
  const mjtNum point_size[3] = {0.005, 0.005, 0.005};
  for (const auto& point : axis_points_) {
    if (scene.ngeom >= scene.maxgeom) break;
    mjtNum pos[3] = {point[0], point[1], point[2]};
    if (mjvGeom* marker = append_geom())
      mjv_initGeom(marker, mjGEOM_SPHERE, point_size, pos, nullptr, picked_color);
  }
  for (size_t i = 1; i < axis_points_.size() && scene.ngeom < scene.maxgeom; ++i) {
    mjtNum a[3] = {axis_points_[i-1][0], axis_points_[i-1][1], axis_points_[i-1][2]};
    mjtNum b[3] = {axis_points_[i][0], axis_points_[i][1], axis_points_[i][2]};
    if (mjvGeom* line = append_geom()) {
      mjv_initGeom(line, mjGEOM_LINE, nullptr, nullptr, nullptr, picked_color);
      mjv_connector(line, mjGEOM_LINE, 2, a, b);
    }
  }
  if (axis_points_.size() >= 3 && scene.ngeom < scene.maxgeom) {
    const auto& last = axis_points_.back();
    const auto& first = axis_points_.front();
    mjtNum a[3] = {last[0], last[1], last[2]};
    mjtNum b[3] = {first[0], first[1], first[2]};
    if (mjvGeom* line = append_geom()) {
      mjv_initGeom(line, mjGEOM_LINE, nullptr, nullptr, nullptr, picked_color);
      mjv_connector(line, mjGEOM_LINE, 2, a, b);
    }
  }
  if (!show_assembly_axes) return;
  for (size_t group = 1; group < assemblies_.size(); ++group) {
    if (!AssemblyEnabled(int(group))) continue;
    const auto& assembly = assemblies_[group];
    const bool assigned = std::any_of(parts_.begin(), parts_.end(), [group](const CadPart& part) {
      return part.assembly == int(group);
    });
    if (!assigned) continue;
    if (assembly.kind == AssemblyKind::Fixed || scene.ngeom + 2 > scene.maxgeom) continue;
    const double length = Length(assembly.axis);
    if (length < 1e-8) continue;
    const Vec3 axis = Mul(assembly.axis, 1.0/length);
    mjtNum start[3], end[3], center[3];
    for (int i = 0; i < 3; ++i) {
      center[i] = assembly.pivot[i];
      start[i] = center[i] - 0.08*axis[i];
      end[i] = center[i] + 0.08*axis[i];
    }
    const float weapon_color[4] = {1.0f, 0.68f, 0.12f, 1.0f};
    const float wheel_color[4] = {0.15f, 0.9f, 1.0f, 1.0f};
    const float fork_color[4] = {0.8f, 0.35f, 1.0f, 1.0f};
    const float* color = assembly.kind == AssemblyKind::Weapon ? weapon_color :
      (assembly.kind == AssemblyKind::GravityFork ? fork_color : wheel_color);
    if (mjvGeom* line = append_geom()) {
      mjv_initGeom(line, mjGEOM_LINE, nullptr, nullptr, nullptr, color);
      mjv_connector(line, mjGEOM_LINE, 3, start, end);
    }
    const mjtNum size[3] = {0.009, 0.009, 0.009};
    if (mjvGeom* marker = append_geom())
      mjv_initGeom(marker, mjGEOM_SPHERE, size, center, nullptr, color);
  }
}

void FusionBundle::DrawPart(size_t index) {
  auto& part = parts_[index];
  ImGui::PushID(int(index));
  const std::string label = part.name + "##part";
  const bool focus_selected = focus_part_request_ == int(index);
  const float part_row_y = ImGui::GetCursorPosY();
  if (focus_part_request_ >= 0)
    ImGui::SetNextItemOpen(focus_selected, ImGuiCond_Always);
  const ImGuiTreeNodeFlags node_flags = selected_part_ == int(index)
    ? ImGuiTreeNodeFlags_Selected : ImGuiTreeNodeFlags_None;
  const bool node_open = ImGui::TreeNodeEx(label.c_str(), node_flags);
  if (node_open) {
    ImGui::Checkbox("Show in preview", &part.preview_visible);
    ImGui::Checkbox("Use for collisions", &part.collision_enabled);
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("Disable collision for flexible wires/cables or decorative meshes that should not push on the arena floor. The part stays visible and keeps its mass.");
    ImGui::TextWrapped("STL: %s", part.file.c_str());
    ImGui::TextWrapped("Fusion material: %s", part.material.c_str());
    if (!part.appearance.empty()) ImGui::TextWrapped("Appearance: %s", part.appearance.c_str());
    if (part.roughness >= 0) ImGui::Text("Fusion roughness: %.2f", part.roughness);
    ImGui::Text("Fusion material density: %.0f kg/m3", part.density);
    if (!part.material_properties.empty() && ImGui::TreeNode("Fusion material properties")) {
      for (const auto& property : part.material_properties) ImGui::TextWrapped("%s", property.c_str());
      ImGui::TreePop();
    }
    const char* chosen = part.assembly < 0 ? "Exclude" : assemblies_[part.assembly].name.c_str();
    if (ImGui::BeginCombo("Rigid assembly", chosen)) {
      if (ImGui::Selectable("Exclude", part.assembly < 0)) part.assembly = -1;
      for (size_t i = 0; i < assemblies_.size(); ++i) {
        if (ImGui::Selectable(assemblies_[i].name.c_str(), part.assembly == int(i))) {
          bool first = true;
          for (size_t j = 0; j < parts_.size(); ++j)
            if (j != index && parts_[j].assembly == int(i)) { first = false; break; }
          part.assembly = int(i);
          if (first && i > 0) assemblies_[i].pivot = Mul(Add(part.min_bound, part.max_bound), 0.5);
          if (i > 0 && assemblies_[i].kind != AssemblyKind::Fixed) {
            axis_assembly_ = int(i);
            axis_points_.clear(); hovered_loop_.clear(); axis_picking_ = false;
          }
        }
      }
      ImGui::EndCombo();
    }
    ImGui::Text("Fusion solid mass: %.2f g", part.fusion_mass * 1000.0);
    if (ImGui::BeginCombo("Mass model", MassModeLabel(part.mass_mode))) {
      for (const auto mode : {PartMassMode::Fusion, PartMassMode::Manual,
                              PartMassMode::Printed})
        if (ImGui::Selectable(MassModeLabel(mode), part.mass_mode == mode)) {
          part.mass_mode = mode;
          if (mode == PartMassMode::Printed) RefreshPrintEstimate(part);
          else part.mass = mode == PartMassMode::Manual ? part.manual_mass : part.fusion_mass;
        }
      ImGui::EndCombo();
    }
    if (part.mass_mode == PartMassMode::Manual) {
      double mass_grams = part.manual_mass * 1000.0;
      if (ImGui::InputDouble("Manual weight (g)", &mass_grams, 0.1, 1.0, "%.2f")) {
        part.manual_mass = std::max(0.0, mass_grams / 1000.0);
        part.mass = part.manual_mass;
      }
    } else if (part.mass_mode == PartMassMode::Printed) {
      auto& profiles = PrintProfiles();
      if (profiles.empty()) profiles.push_back(PrintProfile{});
      if (part.print_profile_selection.empty())
        part.print_profile_selection = part.print_profile.name;
      if (!FindPrintProfile(part.print_profile_selection))
        part.print_profile_selection = FindPrintProfile(part.print_profile.name) ?
          part.print_profile.name : profiles.front().name;
      const std::string applied_picker_label =
        FindPrintProfile(part.print_profile_selection) ?
          part.print_profile_selection : part.print_profile_selection + " (saved copy)";
      if (ImGui::BeginCombo("Profile to apply", applied_picker_label.c_str())) {
        for (const auto& profile : profiles)
          if (ImGui::Selectable(profile.name.c_str(),
                                profile.name == part.print_profile_selection)) {
            part.print_profile_selection = profile.name;
          }
        ImGui::EndCombo();
      }
      if (ImGui::Button("Apply profile to this part")) {
        if (const PrintProfile* selected = FindPrintProfile(part.print_profile_selection)) {
          part.print_profile = *selected;
          RefreshPrintEstimate(part);
        }
      }
      ImGui::Text("Applied profile snapshot: %s", part.print_profile.name.c_str());
      ImGui::TextWrapped("Saved to this part: %.2f mm line width, %d walls, %d top / %d bottom layers, %.1f%% %s infill.",
        part.print_profile.line_width_mm, part.print_profile.wall_loops,
        part.print_profile.top_layers, part.print_profile.bottom_layers,
        part.print_profile.infill_density_percent,
        InfillPatternLabel(part.print_profile.infill_pattern));
      ImGui::TextDisabled("Save setup stores this part's applied settings and bed direction beside the manifest.");
      ImGui::TextWrapped("Build axis in part coordinates: %.3f, %.3f, %.3f",
        part.build_axis_local[0], part.build_axis_local[1], part.build_axis_local[2]);
      if (ImGui::Button("Pick bed contact face in preview"))
        StartPrintBedFacePick(int(index));
      if (print_bed_face_picking_ && print_bed_face_part_ == int(index)) {
        ImGui::SameLine();
        if (ImGui::Button("Cancel face pick")) CancelPrintBedFacePick();
      }
      if (part.print_estimate_valid) {
        ImGui::Text("Estimated printed weight: %.2f g", part.printed_mass * 1000.0);
        if (part.has_fusion_center_of_mass) {
          const Vec3 offset_mm = Mul(
            Sub(part.printed_center_local, part.center_of_mass_local), 1000.0);
          ImGui::TextWrapped("Estimated center-of-mass shift: %.2f, %.2f, %.2f mm",
            offset_mm[0], offset_mm[1], offset_mm[2]);
        }
      }
      if (!part.print_estimate_error.empty())
        ImGui::TextWrapped("Print estimate: %s", part.print_estimate_error.c_str());
      if (ImGui::SmallButton("Recalculate print mass")) RefreshPrintEstimate(part);
    } else {
      ImGui::Text("Physics weight: %.2f g", part.fusion_mass * 1000.0);
    }
    ImGui::DragScalar("Friction estimate", ImGuiDataType_Double, &part.friction, 0.01, nullptr, nullptr, "%.2f");
    part.mass = std::max(0.0, part.mass);
    part.friction = std::max(0.01, part.friction);
    float color[4] = {float(part.color[0]), float(part.color[1]), float(part.color[2]), float(part.color[3])};
    if (ImGui::ColorEdit4("Color", color)) for (int i = 0; i < 4; ++i) part.color[i] = color[i];
    ImGui::TreePop();
  }
  if (focus_selected) {
    // Apply the scroll request after the full expanded body has been laid out,
    // so ImGui knows the final scroll range for this frame. Repeat briefly to
    // let the parent configuration child settle after opening nested nodes.
    ImGui::SetScrollFromPosY(part_row_y, 0.15f);
  }
  ImGui::PopID();
}

void FusionBundle::DrawTree(const std::vector<std::string>& parent) {
  std::set<std::string> children;
  for (size_t i = 0; i < parts_.size(); ++i) {
    const auto& path = parts_[i].occurrence_path;
    if (path.size() < parent.size() || !std::equal(parent.begin(), parent.end(), path.begin())) continue;
    if (path.size() == parent.size()) DrawPart(i);
    else children.insert(path[parent.size()]);
  }
  for (const auto& child : children) {
    auto path = parent; path.push_back(child);
    ImGui::PushID(int(parent.size()));
    bool contains_focus_target = false;
    if (focus_part_request_ >= 0 && focus_part_request_ < int(parts_.size())) {
      const auto& target_path = parts_[focus_part_request_].occurrence_path;
      contains_focus_target = target_path.size() >= path.size() &&
        std::equal(path.begin(), path.end(), target_path.begin());
    }
    if (focus_part_request_ >= 0)
      ImGui::SetNextItemOpen(contains_focus_target, ImGuiCond_Always);
    if (ImGui::TreeNode(child.c_str())) {
      int folder_assignment = -2;
      bool folder_has_parts = false;
      bool mixed_assignments = false;
      for (const auto& part : parts_) {
        const auto& part_path = part.occurrence_path;
        if (part_path.size() < path.size() || !std::equal(path.begin(), path.end(), part_path.begin())) continue;
        if (!folder_has_parts) {
          folder_assignment = part.assembly;
          folder_has_parts = true;
        } else if (folder_assignment != part.assembly) {
          mixed_assignments = true;
          break;
        }
      }
      const std::string assignment_label = mixed_assignments ? "Mixed assemblies" :
        !folder_has_parts ? "No bodies" :
        folder_assignment < 0 ? "Exclude" : assemblies_[folder_assignment].name;
      if (ImGui::SmallButton("Show folder")) {
        for (auto& part : parts_)
          if (part.occurrence_path.size() >= path.size() &&
              std::equal(path.begin(), path.end(), part.occurrence_path.begin()))
            part.preview_visible = true;
      }
      ImGui::SameLine();
      if (ImGui::SmallButton("Hide folder")) {
        for (auto& part : parts_)
          if (part.occurrence_path.size() >= path.size() &&
              std::equal(path.begin(), path.end(), part.occurrence_path.begin()))
            part.preview_visible = false;
      }
      if (ImGui::BeginCombo("Assign this folder", assignment_label.c_str())) {
        for (int group = -1; group < int(assemblies_.size()); ++group) {
          const char* label = group < 0 ? "Exclude" : assemblies_[group].name.c_str();
          if (ImGui::Selectable(label, folder_has_parts && !mixed_assignments && folder_assignment == group)) {
            for (auto& part : parts_) {
              const auto& part_path = part.occurrence_path;
              if (part_path.size() >= path.size() && std::equal(path.begin(), path.end(), part_path.begin()))
                part.assembly = group;
            }
            if (group > 0) {
              assemblies_[group].pivot = GroupCenter(group);
              if (assemblies_[group].kind != AssemblyKind::Fixed) axis_assembly_ = group;
            }
            axis_points_.clear(); hovered_loop_.clear(); axis_picking_ = false;
            pick_message_ = "Assigned folder " + child + " to " + label + ".";
          }
        }
        ImGui::EndCombo();
      }
      DrawTree(path); ImGui::TreePop();
    }
    ImGui::PopID();
  }
}

void FusionBundle::Draw(SDL_Window* window, bool& load_requested, bool& apply_requested) {
  {
    std::lock_guard<std::mutex> lock(dialog_mutex_);
    if (!pending_path_.empty()) {
      std::snprintf(path_buffer_, sizeof(path_buffer_), "%s", pending_path_.c_str());
      pending_path_.clear();
      if (Load(path_buffer_)) load_requested = true;
    }
  }
  ImGui::InputText("manifest.json path", path_buffer_, sizeof(path_buffer_));
  if (ImGui::Button("Browse...")) {
    static const SDL_DialogFileFilter filters[] = {{"Fusion manifest", "json"}};
    SDL_ShowOpenFileDialog([](void* user, const char* const* paths, int) {
      if (!paths || !paths[0]) return;
      auto* editor = static_cast<FusionBundle*>(user);
      std::lock_guard<std::mutex> lock(editor->dialog_mutex_);
      editor->pending_path_ = paths[0];
    }, this, window, filters, 1, nullptr, false);
  }
  ImGui::SameLine();
  if (ImGui::Button("Load Fusion bundle")) {
    if (Load(path_buffer_)) load_requested = true;
  }
  if (!error_.empty()) ImGui::TextWrapped("Error: %s", error_.c_str());
  if (!HasBundle()) {
    ImGui::TextWrapped("Export the design from Fusion, then enter its manifest.json path above.");
    return;
  }
  ImGui::TextWrapped("%s: %d STL bodies", design_name_.c_str(), int(parts_.size()));
  if (ImGui::Button("Save setup")) Save();
  ImGui::SameLine();
  if (ImGui::Button("Apply and preview")) {
    if (Save()) apply_requested = true;
  }
  ImGui::Checkbox("Show wheel, weapon and fork axis markers", &show_axis_markers_);
  ImGui::SeparatorText("Bot orientation");
  DrawMotorDoubleInput("Bot rotation about Z (deg)", &bot_rotation_degrees_, 1.0, 15.0, "%.1f");
  if (!std::isfinite(bot_rotation_degrees_)) bot_rotation_degrees_ = 0.0;
  bot_rotation_degrees_ = std::clamp(bot_rotation_degrees_, -3600.0, 3600.0);
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("Rotates the entire assembled bot around vertical Z. CAD body transforms, component assignments, and fitted wheel/weapon axes remain in their original CAD coordinates.");
  ImGui::TextWrapped("This rotates the imported assembly as a whole; it does not rewrite the CAD-defined part or axis coordinates.");
  DrawPrintProfileManager();
  if (print_bed_face_picking_) {
    ImGui::TextColored(ImVec4(0.35f, 0.90f, 0.82f, 1.0f),
      "Bed-face pick active: click the selected part's face in the CAD preview.");
    if (ImGui::SmallButton("Cancel bed-face pick")) CancelPrintBedFacePick();
  }
  ImGui::SeparatorText("Bot powertrain");
  DrawMotorDoubleInput("Battery voltage (V)", &battery_voltage_, 0.1, 1.0, "%.2f");
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("Use the battery pack's nominal voltage. For a LiPo pack, cell count x 3.7 V is a reasonable nominal value.");
  if (!std::isfinite(battery_voltage_) || battery_voltage_ < 0.0) battery_voltage_ = 0.0;
  ImGui::TextWrapped("Per motor, enter either Kv or a published RPM and voltage, plus peak current. Detailed electrical specs can be left blank; the pack voltage is required.");
  const char* layout_label = drive_layout_ == DriveLayout::TwoWheel ? "2WD" : "4WD";
  if (ImGui::BeginCombo("Drive layout", layout_label)) {
    if (ImGui::Selectable("2WD", drive_layout_ == DriveLayout::TwoWheel))
      SetWheelDriveLayout(DriveLayout::TwoWheel);
    if (ImGui::Selectable("4WD", drive_layout_ == DriveLayout::FourWheel))
      SetWheelDriveLayout(DriveLayout::FourWheel);
    ImGui::EndCombo();
  }
  if (drive_layout_ == DriveLayout::TwoWheel) {
    const char* axle_label = two_wd_axle_ == "rear" ? "Rear axle" : "Front axle";
    if (ImGui::BeginCombo("Driven axle", axle_label)) {
      if (ImGui::Selectable("Front axle", two_wd_axle_ == "front"))
        SetTwoWheelDriveAxle("front");
      if (ImGui::Selectable("Rear axle", two_wd_axle_ == "rear"))
        SetTwoWheelDriveAxle("rear");
      ImGui::EndCombo();
    }
    ImGui::TextWrapped("Only the selected axle's left and right wheel assemblies are simulated. The other axle stays saved and returns when you switch to 4WD.");
  } else {
    ImGui::TextWrapped("4WD simulates the assigned front and rear left/right wheel assemblies.");
  }
  if (ImGui::TreeNodeEx("Drive motor settings", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::TextWrapped("Assign wheels driven by the same physical motor to the same group. The group's available torque is shared evenly across its wheel assemblies. Wheel-speed linkages are not constrained yet, so chain/belt coupling is approximate.");
    static const char* motor_group_labels[] = {
      "Motor A", "Motor B", "Motor C", "Motor D",
      "Motor E", "Motor F", "Motor G", "Motor H"};
    bool has_drive_wheels = false;
    for (size_t i = 1; i < assemblies_.size(); ++i) {
      auto& group = assemblies_[i];
      if (group.kind != AssemblyKind::LeftWheel && group.kind != AssemblyKind::RightWheel)
        continue;
      if (!AssemblyEnabled(int(i))) continue;
      has_drive_wheels = true;
      ImGui::PushID(int(i));
      const int old_drive_group = group.motor.drive_group;
      const char* current_group = old_drive_group >= 0 && old_drive_group < 8
        ? motor_group_labels[old_drive_group] : "Select motor group";
      ImGui::TextWrapped("%s (%s)", group.name.c_str(), KindLabel(group.kind));
      ImGui::TextDisabled("Motor group");
      ImGui::SetNextItemWidth(240.0f);
      if (ImGui::BeginCombo("##driven_by", current_group)) {
        for (int candidate = 0; candidate < 8; ++candidate) {
          if (!ImGui::Selectable(motor_group_labels[candidate],
                                 group.motor.drive_group == candidate))
            continue;
          group.motor.drive_group = candidate;
          for (size_t other = 1; other < assemblies_.size(); ++other) {
            if (other == i || assemblies_[other].kind == AssemblyKind::Fixed ||
                (assemblies_[other].kind != AssemblyKind::LeftWheel &&
                 assemblies_[other].kind != AssemblyKind::RightWheel) ||
                !AssemblyEnabled(int(other)) ||
                assemblies_[other].motor.drive_group != candidate)
              continue;
            group.motor = assemblies_[other].motor;
            group.motor.drive_group = candidate;
            break;
          }
        }
        ImGui::EndCombo();
      }
      ImGui::PopID();
    }
    if (has_drive_wheels) {
      ImGui::SeparatorText("Motor group specifications");
      std::set<int> shown_groups;
      for (size_t i = 1; i < assemblies_.size(); ++i) {
        auto& owner = assemblies_[i];
        if ((owner.kind != AssemblyKind::LeftWheel && owner.kind != AssemblyKind::RightWheel) ||
            !AssemblyEnabled(int(i)) || owner.motor.drive_group < 0 || owner.motor.drive_group >= 8 ||
            shown_groups.count(owner.motor.drive_group))
          continue;
        shown_groups.insert(owner.motor.drive_group);
        int member_count = 0;
        for (size_t member = 1; member < assemblies_.size(); ++member)
          if ((assemblies_[member].kind == AssemblyKind::LeftWheel ||
               assemblies_[member].kind == AssemblyKind::RightWheel) &&
              AssemblyEnabled(int(member)) &&
              assemblies_[member].motor.drive_group == owner.motor.drive_group)
            ++member_count;
        ImGui::PushID(1000 + owner.motor.drive_group);
        const bool default_open = shown_groups.size() == 1;
        const bool open = ImGui::TreeNodeEx(motor_group_labels[owner.motor.drive_group],
          default_open ? ImGuiTreeNodeFlags_DefaultOpen : ImGuiTreeNodeFlags_None);
        if (open) {
          ImGui::TextWrapped("Torque is shared across %d wheel assemblies.", member_count);
          ImGui::Checkbox("Enable wheel torque model", &owner.motor.enabled);
          DrawMotorIntInput("Physical motors in group", &owner.motor.motor_count);
          owner.motor.motor_count = std::clamp(owner.motor.motor_count, 1, 32);
          DrawMotorSpeedInputs(11000 + owner.motor.drive_group, owner.motor);
          DrawMotorDoubleInput("Peak current per motor (A)", &owner.motor.peak_current_a,
                               0.1, 1.0, "%.2f");
          if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Used as the peak torque-producing current cap. The model does not simulate motor heating or thermal derating.");
          DrawGearRatioInput(1000 + owner.motor.drive_group, owner.motor);
          if (ImGui::TreeNode("Optional motor and gearbox settings")) {
            double phase_resistance_milliohm = owner.motor.phase_resistance_ohm * 1000.0;
            ImGui::TextWrapped("%s", MotorResistanceLabel(owner.motor.type));
            ImGui::SetNextItemWidth(std::min(180.0f, ImGui::GetContentRegionAvail().x));
            if (ImGui::InputDouble("##motor_resistance", &phase_resistance_milliohm,
                                   1.0, 10.0, "%.2f"))
              owner.motor.phase_resistance_ohm = phase_resistance_milliohm / 1000.0;
            if (ImGui::IsItemHovered())
              ImGui::SetTooltip("Optional. Leave at 0 if the motor datasheet does not list it; the current-limited approximate torque model still works.");
            DrawMotorDoubleInput(MotorCurrentLimitLabel(owner.motor.type),
                                 &owner.motor.esc_current_limit_a, 0.1, 1.0, "%.2f");
            if (ImGui::IsItemHovered())
              ImGui::SetTooltip("Optional. Enter the controller's motor-current limit if known; otherwise leave 0.");
            double efficiency_percent = owner.motor.gearbox_efficiency * 100.0;
            ImGui::TextWrapped("Gearbox efficiency (%)");
            ImGui::SetNextItemWidth(std::min(180.0f, ImGui::GetContentRegionAvail().x));
            if (ImGui::InputDouble("##gearbox_efficiency", &efficiency_percent,
                                   1.0, 5.0, "%.1f"))
              owner.motor.gearbox_efficiency = efficiency_percent / 100.0;
            owner.motor.gearbox_efficiency = std::clamp(owner.motor.gearbox_efficiency, 1e-3, 1.0);
            ImGui::TextWrapped("Defaults: resistance and controller limit 0 (unknown), 1:1 ratio, 100%% efficiency.");
            ImGui::TreePop();
          }
          const double wheel_no_load_rpm = MotorNoLoadOutputRpm(owner.motor, battery_voltage_);
          if (wheel_no_load_rpm > 0.0)
            ImGui::TextWrapped("Estimated wheel no-load speed: %.0f RPM", wheel_no_load_rpm);
          const MotorSpec shared_spec = owner.motor;
          for (size_t member = 1; member < assemblies_.size(); ++member)
            if ((assemblies_[member].kind == AssemblyKind::LeftWheel ||
                 assemblies_[member].kind == AssemblyKind::RightWheel) &&
                AssemblyEnabled(int(member)) &&
                assemblies_[member].motor.drive_group == owner.motor.drive_group)
              assemblies_[member].motor = shared_spec;
          ImGui::TreePop();
        }
        ImGui::PopID();
      }
    }
    if (!has_drive_wheels)
      ImGui::TextWrapped("Set an assembly type to Left wheel or Right wheel below to configure it here.");
    ImGui::TreePop();
  }
  if (ImGui::TreeNodeEx("Weapon motor settings", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::TextWrapped("Weapon spin-up and speed are driven by motor torque and the weapon assembly's inertia. Configure the motor, battery, and input:output gearing here.");
    bool has_weapon = false;
    for (size_t i = 1; i < assemblies_.size(); ++i) {
      auto& weapon = assemblies_[i];
      if (weapon.kind != AssemblyKind::Weapon) continue;
      has_weapon = true;
      ImGui::PushID(2000 + int(i));
      ImGui::TextWrapped("%s", weapon.name.c_str());
      ImGui::Separator();
      ImGui::Checkbox("Enable weapon torque model", &weapon.motor.enabled);
      DrawMotorIntInput("Physical motors in weapon assembly", &weapon.motor.motor_count);
      weapon.motor.motor_count = std::clamp(weapon.motor.motor_count, 1, 32);
      DrawMotorSpeedInputs(21000 + int(i), weapon.motor);
      DrawMotorDoubleInput("Peak current per motor (A)", &weapon.motor.peak_current_a,
                           0.1, 1.0, "%.2f");
      if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Used as the peak torque-producing current cap. Motor heating and thermal derating are not simulated.");
      DrawGearRatioInput(20000 + int(i), weapon.motor);
      if (ImGui::TreeNode("Optional motor and gearbox settings")) {
        double phase_resistance_milliohm = weapon.motor.phase_resistance_ohm * 1000.0;
        ImGui::TextWrapped("%s", MotorResistanceLabel(weapon.motor.type));
        ImGui::SetNextItemWidth(std::min(180.0f, ImGui::GetContentRegionAvail().x));
        if (ImGui::InputDouble("##motor_resistance", &phase_resistance_milliohm,
                               1.0, 10.0, "%.2f"))
          weapon.motor.phase_resistance_ohm = phase_resistance_milliohm / 1000.0;
        if (ImGui::IsItemHovered())
          ImGui::SetTooltip("Optional. Leave at 0 if the motor datasheet does not list it; the current-limited approximate torque model still works.");
        DrawMotorDoubleInput(MotorCurrentLimitLabel(weapon.motor.type),
                             &weapon.motor.esc_current_limit_a, 0.1, 1.0, "%.2f");
        if (ImGui::IsItemHovered())
          ImGui::SetTooltip("Optional. Enter the controller's motor-current limit if known; otherwise leave 0.");
        double efficiency_percent = weapon.motor.gearbox_efficiency * 100.0;
        ImGui::TextWrapped("Gearbox efficiency (%)");
        ImGui::SetNextItemWidth(std::min(180.0f, ImGui::GetContentRegionAvail().x));
        if (ImGui::InputDouble("##gearbox_efficiency", &efficiency_percent,
                               1.0, 5.0, "%.1f"))
          weapon.motor.gearbox_efficiency = efficiency_percent / 100.0;
        weapon.motor.gearbox_efficiency = std::clamp(weapon.motor.gearbox_efficiency, 1e-3, 1.0);
        ImGui::TextWrapped("Defaults: resistance and controller limit 0 (unknown), 1:1 ratio, 100%% efficiency.");
        ImGui::TreePop();
      }
      const double weapon_no_load_rpm = MotorNoLoadOutputRpm(weapon.motor, battery_voltage_);
      if (weapon_no_load_rpm > 0.0)
        ImGui::TextWrapped("Estimated weapon no-load speed: %.0f RPM", weapon_no_load_rpm);
      if (weapon.motor.enabled) {
        const MassProperties properties = GroupMass(parts_, int(i), weapon.pivot);
        const double inertia = MomentAboutAxis(properties, weapon.axis);
        const double spinup_seconds = EstimateMotorSpinupSeconds(
          weapon.motor, battery_voltage_, inertia);
        if (spinup_seconds > 0.0)
          ImGui::TextWrapped("Estimated 0-to-90%% no-load spin-up: %.2f s", spinup_seconds);
        else
          ImGui::TextWrapped("Enter valid battery and motor data and assign weapon bodies to estimate spin-up.");
        ImGui::TextWrapped("Estimate uses the selected axis and modeled assembly inertia; it excludes battery sag, impacts, and thermal derating.");
      }
      ImGui::PopID();
    }
    if (!has_weapon)
      ImGui::TextWrapped("Assign at least one body group to a Weapon assembly to configure its motor.");
    ImGui::TreePop();
  }
  if (selected_part_ >= 0 && selected_part_ < int(parts_.size())) {
    auto& selected = parts_[selected_part_];
    ImGui::SeparatorText("Selected in 3D preview");
    ImGui::TextWrapped("%s", selected.name.c_str());
    if (ImGui::Button(selected.preview_visible ? "Hide selected" : "Show selected"))
      selected.preview_visible = !selected.preview_visible;
    ImGui::SameLine();
    if (ImGui::Button("Deselect part")) {
      selected_part_ = focus_part_request_ = -1;
      focus_frames_remaining_ = 0;
    }
    const char* chosen = selected.assembly < 0 ? "Exclude" : assemblies_[selected.assembly].name.c_str();
    if (ImGui::BeginCombo("Assign selected part", chosen)) {
      if (ImGui::Selectable("Exclude", selected.assembly < 0)) selected.assembly = -1;
      for (size_t i = 0; i < assemblies_.size(); ++i)
        if (ImGui::Selectable(assemblies_[i].name.c_str(), selected.assembly == int(i))) {
          selected.assembly = int(i);
          if (i > 0) assemblies_[i].pivot = GroupCenter(int(i));
            if (i > 0 && assemblies_[i].kind != AssemblyKind::Fixed &&
                AssemblyEnabled(int(i))) {
              axis_assembly_ = int(i);
              axis_points_.clear(); hovered_loop_.clear(); axis_picking_ = false;
            }
        }
      ImGui::EndCombo();
    }
  }
  if (ImGui::Button("Show all parts")) for (auto& part : parts_) part.preview_visible = true;
  ImGui::SameLine();
  if (ImGui::Button("Hide all parts")) for (auto& part : parts_) part.preview_visible = false;
  ImGui::SameLine();
  if (ImGui::Button("Show only selected") && selected_part_ >= 0)
    for (size_t i = 0; i < parts_.size(); ++i) parts_[i].preview_visible = int(i) == selected_part_;
  double total = 0.0, included_mass = 0.0;
  for (size_t i = 0; i < parts_.size(); ++i) {
    if (parts_[i].assembly >= 0) total += parts_[i].mass;
    if (PartIncludedInSimulation(int(i))) included_mass += parts_[i].mass;
  }
  ImGui::Text("Assigned CAD mass: %.1f g / 500 g", total * 1000.0);
  if (std::abs(total - included_mass) > 1e-8)
    ImGui::Text("Mass in current simulation: %.1f g", included_mass * 1000.0);
  if (total > 0.5) ImGui::TextWrapped("Assigned CAD mass exceeds the 500 g class. Check for duplicate CAD bodies or adjust overrides.");
  ImGui::SeparatorText("Rigid assemblies");
  ImGui::TextWrapped("Put each physical wheel in its own assembly with its tire, hub, motor, and fasteners. Keep front and rear wheels separate; every assembly gets one joint and one axis. Wheel drive direction is normalized from each selected axis.");
  int fork_count = ForkCount();
  if (ImGui::InputInt("Gravity-driven fork modules", &fork_count)) SetForkCount(fork_count);
  if (!fork_message_.empty())
    ImGui::TextColored(ImVec4(1.00f, 0.75f, 0.36f, 1.0f), "%s", fork_message_.c_str());
  ImGui::TextWrapped("Each fork module is a separate passive hinge with no motor. Assign its CAD bodies, set the hinge center and axis at the attachment, and check that it does not begin interpenetrating the main frame. Travel defaults to +/-90 degrees from the CAD pose and is editable per fork. Explicit contact pairs are generated between every fork body and every main-frame body. Chassis, fixed structural parts, and forks use a damped response only when they touch the floor, including forks left fixed to the frame.");
  for (size_t i = 1; i < assemblies_.size(); ++i) {
    auto& group = assemblies_[i];
    ImGui::PushID(int(i));
    const bool assembly_open = ImGui::TreeNodeEx("##assembly", ImGuiTreeNodeFlags_DefaultOpen);
    ImGui::SameLine();
    ImGui::TextUnformatted(group.name.c_str());
    if (assembly_open) {
      char name[128]; std::snprintf(name, sizeof(name), "%s", group.name.c_str());
      if (ImGui::InputText("Name", name, sizeof(name))) group.name = name;
      if (ImGui::BeginCombo("Type", KindLabel(group.kind))) {
        for (const auto kind : {AssemblyKind::Fixed, AssemblyKind::LeftWheel,
                                AssemblyKind::RightWheel, AssemblyKind::Weapon,
                                AssemblyKind::GravityFork})
          if (ImGui::Selectable(KindLabel(kind), group.kind == kind)) {
            const bool was_drive_wheel = group.kind == AssemblyKind::LeftWheel ||
                                         group.kind == AssemblyKind::RightWheel;
            group.kind = kind;
            const bool is_drive_wheel = kind == AssemblyKind::LeftWheel ||
                                        kind == AssemblyKind::RightWheel;
            if (is_drive_wheel && !was_drive_wheel && group.wheel_position.empty())
              group.wheel_position = "single";
            if (!is_drive_wheel) {
              group.motor.enabled = false;
              group.motor.drive_group = -1;
            } else if (!was_drive_wheel || group.motor.drive_group < 0 ||
                       group.motor.drive_group > 7) {
              std::set<int> used_groups;
              for (size_t candidate = 1; candidate < assemblies_.size(); ++candidate)
                if (candidate != i &&
                    (assemblies_[candidate].kind == AssemblyKind::LeftWheel ||
                     assemblies_[candidate].kind == AssemblyKind::RightWheel))
                  used_groups.insert(assemblies_[candidate].motor.drive_group);
              int available_group = 0;
              while (available_group < 7 && used_groups.count(available_group))
                ++available_group;
              group.motor.drive_group = available_group;
            }
            if (kind != AssemblyKind::Fixed && AssemblyEnabled(int(i))) axis_assembly_ = int(i);
            else if (axis_assembly_ == int(i)) {
              axis_picking_ = false; axis_points_.clear(); hovered_loop_.clear();
              axis_assembly_ = 0;
              for (size_t candidate = 1; candidate < assemblies_.size(); ++candidate)
                if (assemblies_[candidate].kind != AssemblyKind::Fixed &&
                    AssemblyEnabled(int(candidate))) {
                  axis_assembly_ = int(candidate);
                  break;
                }
            }
          }
        ImGui::EndCombo();
      }
      if (group.kind == AssemblyKind::LeftWheel || group.kind == AssemblyKind::RightWheel) {
        const char* wheel_position_label = group.wheel_position == "rear" ? "Rear axle" :
          group.wheel_position == "front" ? "Front axle" : "Single axle";
        if (ImGui::BeginCombo("Wheel position", wheel_position_label)) {
          const std::array<std::pair<const char*, const char*>, 3> positions{{
            {"Single axle", "single"}, {"Front axle", "front"}, {"Rear axle", "rear"}}};
          for (const auto& position : positions) {
            if (!ImGui::Selectable(position.first,
                                   group.wheel_position == position.second)) continue;
            group.wheel_position = position.second;
            if (drive_layout_ == DriveLayout::TwoWheel)
              SetTwoWheelDriveAxle(two_wd_axle_);
            if (AssemblyEnabled(int(i))) axis_assembly_ = int(i);
          }
          ImGui::EndCombo();
        }
        if (drive_layout_ == DriveLayout::TwoWheel && !AssemblyEnabled(int(i)))
          ImGui::TextDisabled("Stored for 4WD; excluded from this 2WD simulation.");
      }
      if (group.kind != AssemblyKind::Fixed) {
        const auto assembly_mass_properties = GroupMass(parts_, int(i), group.pivot);
        Vec3 shaft_axis = group.axis;
        const double axis_length = Length(shaft_axis);
        if (axis_length > 1e-8) {
          shaft_axis = Mul(shaft_axis, 1.0 / axis_length);
          const Vec3 center_offset = assembly_mass_properties.center;
          const double radial_offset = Length(Sub(center_offset,
            Mul(shaft_axis, Dot(center_offset, shaft_axis))));
          ImGui::Text("Combined COM offset from axis: %.2f mm", radial_offset * 1000.0);
          if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Mass-weighted across every body assigned to this assembly. For a spinner this sets imbalance; for a passive fork it creates gravity torque.");
        }
      }
      if (group.kind == AssemblyKind::GravityFork) {
        ImGui::Checkbox("Collide with main frame", &group.fork_collides_with_frame);
        if (ImGui::IsItemHovered())
          ImGui::SetTooltip("Adds explicit MuJoCo contacts between this fork's CAD geoms and the main-frame geoms.");
        ImGui::TextWrapped("Fork hinge travel relative to the imported CAD pose (degrees; zero is the current preview pose).");
        ImGui::InputScalarN("Travel min/max", ImGuiDataType_Double,
                            group.fork_range_degrees.data(), 2);
      }
      ImGui::Text("Rotation center (mm)");
      double pivot_mm[3] = {group.pivot[0] * 1000.0,
                            group.pivot[1] * 1000.0,
                            group.pivot[2] * 1000.0};
      if (ImGui::InputScalarN("Center X/Y/Z", ImGuiDataType_Double, pivot_mm, 3))
        for (int axis = 0; axis < 3; ++axis) group.pivot[axis] = pivot_mm[axis] * 0.001;
      ImGui::Text("Rotation axis (direction)");
      ImGui::InputScalarN("Axis X/Y/Z", ImGuiDataType_Double, group.axis.data(), 3);
      if (ImGui::SmallButton("Axis X")) group.axis = {1, 0, 0};
      ImGui::SameLine();
      if (ImGui::SmallButton("Axis Y")) group.axis = {0, 1, 0};
      ImGui::SameLine();
      if (ImGui::SmallButton("Axis Z")) group.axis = {0, 0, 1};
      ImGui::SameLine();
      if (ImGui::SmallButton("Flip axis")) group.axis = Mul(group.axis, -1.0);
      if (ImGui::Button("Center on assigned bodies")) group.pivot = GroupCenter(int(i));
      ImGui::TreePop();
    }
    ImGui::PopID();
  }
  if (ImGui::Button("Add rigid assembly"))
    assemblies_.push_back({"Assembly " + std::to_string(assemblies_.size()), AssemblyKind::Fixed, {}, {0, 1, 0}});
  ImGui::SeparatorText("Fusion assembly tree");
  DrawTree({});
  if (focus_frames_remaining_ > 0 && --focus_frames_remaining_ == 0)
    focus_part_request_ = -1;
}

void FusionBundle::Clear() {
  {
    std::lock_guard<std::mutex> lock(dialog_mutex_);
    pending_path_.clear();
  }
  WipeParts(parts_);
  assemblies_.clear();
  manifest_.clear();
  Wipe(manifest_json_);
  memory_loaded_ = false;
  design_name_.clear();
  error_.clear();
  selected_part_ = hovered_part_ = -1;
  focus_part_request_ = -1;
  focus_frames_remaining_ = 0;
  axis_picking_ = false;
  print_bed_face_picking_ = false;
  print_bed_face_part_ = -1;
  axis_points_.clear();
  hovered_loop_.clear();
}

void FusionBundle::DiscardSessionMeshBuffers() {
  if (!memory_loaded_) return;
  for (auto& part : parts_) {
    Wipe(part.mesh_data);
    Wipe(part.render_mesh_data);
    for (auto& texture : part.textures) Wipe(texture.data);
  }
}

void FusionBundle::DrawImporter(SDL_Window* window, bool& changed) {
  {
    std::lock_guard<std::mutex> lock(dialog_mutex_);
    if (!pending_path_.empty()) {
      std::snprintf(path_buffer_, sizeof(path_buffer_), "%s", pending_path_.c_str());
      pending_path_.clear();
      if (Load(path_buffer_)) changed = true;
    }
  }
  ImGui::PushID(this);
  ImGui::InputText("Opponent manifest.json", path_buffer_, sizeof(path_buffer_));
  if (ImGui::Button("Browse opponent...")) {
    static const SDL_DialogFileFilter filters[] = {{"Fusion manifest", "json"}};
    SDL_ShowOpenFileDialog([](void* user, const char* const* paths, int) {
      if (!paths || !paths[0]) return;
      auto* editor = static_cast<FusionBundle*>(user);
      std::lock_guard<std::mutex> lock(editor->dialog_mutex_);
      editor->pending_path_ = paths[0];
    }, this, window, filters, 1, nullptr, false);
  }
  ImGui::SameLine();
  if (ImGui::Button("Load opponent")) {
    if (Load(path_buffer_)) changed = true;
  }
  if (!error_.empty()) ImGui::TextWrapped("Opponent import: %s", error_.c_str());
  if (HasBundle()) {
    ImGui::TextWrapped("%s: %d imported bodies; configure assemblies in the selected bot editor.",
                       design_name_.c_str(), int(parts_.size()));
    ImGui::SameLine();
    if (ImGui::SmallButton("Remove")) { Clear(); changed = true; }
  } else {
    ImGui::TextDisabled("Import a separate Fusion bundle for the opposing bot.");
  }
  ImGui::PopID();
}

void FusionBundle::DrawAxisSelectionControls() {
  if (!HasBundle()) return;
  ImGui::SeparatorText("Select rotation axis");
  const char* axis_name = axis_assembly_ > 0 && axis_assembly_ < int(assemblies_.size()) ?
    assemblies_[axis_assembly_].name.c_str() : "Choose assembly";
  ImGui::SetNextItemWidth(260.0f);
  if (ImGui::BeginCombo("Axis target (weapon / wheel / fork)", axis_name)) {
    for (size_t i = 1; i < assemblies_.size(); ++i) {
      if (!AssemblyEnabled(int(i)) || assemblies_[i].kind == AssemblyKind::Fixed) continue;
      const std::string label = assemblies_[i].name + " (" + KindLabel(assemblies_[i].kind) + ")";
      if (ImGui::Selectable(label.c_str(), int(i) == axis_assembly_)) {
        axis_assembly_ = int(i); axis_points_.clear(); hovered_loop_.clear(); hovered_part_ = -1;
      }
    }
    ImGui::EndCombo();
  }
  if (!axis_picking_) {
    if (ImGui::Button("Select hole for chosen axis")) StartAxis(axis_assembly_);
  } else {
    ImGui::Text("Hole points: %d", int(axis_points_.size()));
    if (ImGui::Button("Fit axis through hole")) FinishAxis();
    ImGui::SameLine();
    if (ImGui::Button("Undo point") && !axis_points_.empty()) axis_points_.pop_back();
    ImGui::SameLine();
    if (ImGui::Button("Cancel axis")) CancelAxis();
  }
  ImGui::TextWrapped("Hover a closed hole rim to highlight its full edge loop, then click once to fit the axis.");
  if (!pick_message_.empty()) ImGui::TextWrapped("%s", pick_message_.c_str());
}
