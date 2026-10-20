#include "FusionBundle.h"

#include "imgui.h"
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace {
using json = nlohmann::json;
using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>;

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
const char* KindLabel(AssemblyKind kind) {
  switch (kind) {
    case AssemblyKind::Frame: return "Frame";
    case AssemblyKind::Fixed: return "Fixed";
    case AssemblyKind::LeftWheel: return "Left wheel";
    case AssemblyKind::RightWheel: return "Right wheel";
    case AssemblyKind::Weapon: return "Weapon";
  }
  return "Fixed";
}
AssemblyKind ParseKind(const std::string& kind) {
  if (kind == "Left wheel") return AssemblyKind::LeftWheel;
  if (kind == "Right wheel") return AssemblyKind::RightWheel;
  if (kind == "Weapon") return AssemblyKind::Weapon;
  return AssemblyKind::Fixed;
}
void Include(Vec3 point, Vec3& low, Vec3& high) {
  for (int i = 0; i < 3; ++i) { low[i] = std::min(low[i], point[i]); high[i] = std::max(high[i], point[i]); }
}
bool ValidMass(double value) { return std::isfinite(value) && value > 0.0; }

void ReadStlBounds(const std::filesystem::path& path, double scale, Vec3 position, Quat rotation,
                   Vec3& low, Vec3& high) {
  std::ifstream file(path, std::ios::binary);
  if (!file) throw std::runtime_error("Cannot read STL: " + path.string());
  file.seekg(0, std::ios::end);
  const auto bytes = file.tellg();
  file.seekg(80);
  uint32_t count = 0;
  file.read(reinterpret_cast<char*>(&count), 4);
  if (!file || count == 0 || bytes != std::streamoff(84) + std::streamoff(count) * 50)
    throw std::runtime_error("Expected a nonempty binary STL: " + path.string());
  low.fill(std::numeric_limits<double>::infinity());
  high.fill(-std::numeric_limits<double>::infinity());
  for (uint32_t t = 0; t < count; ++t) {
    char triangle[50];
    file.read(triangle, sizeof(triangle));
    if (!file) throw std::runtime_error("Truncated STL: " + path.string());
    for (int v = 0; v < 3; ++v) {
      float xyz[3];
      std::memcpy(xyz, triangle + 12 + v * 12, 12);
      Vec3 point = Add(position, Rotate(rotation, {xyz[0]*scale, xyz[1]*scale, xyz[2]*scale}));
      Include(point, low, high);
    }
  }
}

bool SnapToFeatureEdge(const CadPart& part, Vec3 hit, Vec3& midpoint) {
  struct Edge { Vec3 a, b, normal; int faces = 1; double min_dot = 1.0; };
  using Key = std::array<long long, 6>;
  std::map<Key, Edge> edges;
  std::ifstream file(part.absolute_file, std::ios::binary);
  if (!file) return false;
  file.seekg(80);
  uint32_t count = 0; file.read(reinterpret_cast<char*>(&count), 4);
  for (uint32_t t = 0; t < count && file; ++t) {
    char triangle[50]; file.read(triangle, sizeof(triangle));
    if (!file) break;
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
  std::ifstream file(part.absolute_file, std::ios::binary);
  if (!file) return {};
  file.seekg(80);
  uint32_t count = 0; file.read(reinterpret_cast<char*>(&count), 4);
  for (uint32_t t = 0; t < count && file; ++t) {
    char triangle[50]; file.read(triangle, sizeof(triangle));
    if (!file) break;
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
  for (int mode = 0; mode < 2; ++mode) {
    std::map<Key, std::vector<Key>> graph;
    for (const auto& [key, edge] : edges)
      if ((mode == 0 && edge.faces == 1) ||
          (mode == 1 && edge.faces == 2 && edge.min_dot < 0.9063)) {
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

struct MassProperties { double mass = 0; Vec3 center{}; Vec3 inertia{}; };
MassProperties GroupMass(const std::vector<CadPart>& parts, int assembly, Vec3 origin) {
  MassProperties result;
  for (const auto& part : parts) if (part.assembly == assembly && ValidMass(part.mass)) {
    const Vec3 center = Mul(Add(part.min_bound, part.max_bound), 0.5);
    result.center = Add(result.center, Mul(center, part.mass));
    result.mass += part.mass;
  }
  if (result.mass <= 0) return result;
  result.center = Mul(result.center, 1.0 / result.mass);
  for (const auto& part : parts) if (part.assembly == assembly && ValidMass(part.mass)) {
    const Vec3 center = Mul(Add(part.min_bound, part.max_bound), 0.5);
    const Vec3 size = Sub(part.max_bound, part.min_bound);
    const Vec3 offset = Sub(center, result.center);
    const double m = part.mass;
    result.inertia[0] += m * (size[1]*size[1] + size[2]*size[2]) / 12.0 + m * (offset[1]*offset[1] + offset[2]*offset[2]);
    result.inertia[1] += m * (size[0]*size[0] + size[2]*size[2]) / 12.0 + m * (offset[0]*offset[0] + offset[2]*offset[2]);
    result.inertia[2] += m * (size[0]*size[0] + size[1]*size[1]) / 12.0 + m * (offset[0]*offset[0] + offset[1]*offset[1]);
  }
  result.center = Sub(result.center, origin);
  for (double& i : result.inertia) i = std::max(i, 1e-9);
  return result;
}
}

bool FusionBundle::Load(const std::filesystem::path& manifest) {
  try {
    std::ifstream input(manifest);
    if (!input) throw std::runtime_error("Cannot open manifest: " + manifest.string());
    const json source = json::parse(input);
    if (!source.contains("parts") || !source.at("parts").is_array() || source.at("parts").empty())
      throw std::runtime_error("Manifest has no parts array.");
    std::vector<CadPart> loaded;
    std::vector<std::string> missing;
    const auto folder = std::filesystem::weakly_canonical(manifest.parent_path());
    const double default_scale = source.value("unitToMeters", 0.001);
    for (const auto& item : source.at("parts")) {
      CadPart part;
      part.file = item.at("file").get<std::string>();
      const auto path = std::filesystem::weakly_canonical(folder / std::filesystem::path(part.file));
      const auto relative = path.lexically_relative(folder);
      if (relative.empty() || *relative.begin() == ".." || path.extension() != ".stl")
        throw std::runtime_error("Unsafe or unsupported mesh path: " + part.file);
      if (!std::filesystem::is_regular_file(path)) { missing.push_back(part.file); continue; }
      part.absolute_file = path;
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
      part.density = item.value("densityKgPerM3", 0.0);
      part.mass = item.value("massKg", 0.0);
      if (part.mass <= 0 && item.contains("volumeM3") && part.density > 0)
        part.mass = item.at("volumeM3").get<double>() * part.density;
      part.fusion_mass = part.mass;
      part.friction = item.value("frictionEstimate", 0.0);
      if (part.friction <= 0) part.friction = 0.6;
      part.scale = item.value("unitToMeters", default_scale);
      if (!std::isfinite(part.scale) || part.scale <= 0) throw std::runtime_error("Invalid STL unit scale: " + part.file);
      if (item.contains("positionFusionMeters")) {
        part.position = ReadArray<3>(item, "positionFusionMeters", {});
        part.rotation = ReadArray<4>(item, "rotationFusionWxyz", {1, 0, 0, 0});
      } else {
        const auto unity_pos = ReadArray<3>(item, "positionMeters", {});
        const auto unity_rot = ReadArray<4>(item, "rotationUnity", {0, 0, 0, 1});
        part.position = {unity_pos[0], unity_pos[2], unity_pos[1]};
        part.rotation = {-unity_rot[3], unity_rot[0], unity_rot[2], unity_rot[1]};
      }
      const double qlength = std::sqrt(part.rotation[0]*part.rotation[0]+part.rotation[1]*part.rotation[1]+
                                       part.rotation[2]*part.rotation[2]+part.rotation[3]*part.rotation[3]);
      if (qlength < 1e-8) throw std::runtime_error("Invalid rotation: " + part.file);
      for (double& q : part.rotation) q /= qlength;
      if (item.contains("colorRgba") && item.at("colorRgba").is_array())
        part.color = ReadArray<4>(item, "colorRgba", part.color);
      ReadStlBounds(path, part.scale, part.position, part.rotation, part.min_bound, part.max_bound);
      loaded.push_back(std::move(part));
    }
    if (!missing.empty()) {
      std::ostringstream message;
      message << "Fusion bundle is incomplete. Missing " << missing.size() << " STL files:\n";
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
    const auto saved = folder / "combatsim_setup.json";
    if (std::filesystem::is_regular_file(saved)) {
      std::ifstream state_stream(saved);
      const json state = json::parse(state_stream);
      if (state.contains("assemblies") && state.at("assemblies").is_array()) {
        groups.resize(1);
        for (const auto& node : state.at("assemblies")) {
          CadAssembly group;
          group.name = node.value("name", std::string("Assembly"));
          group.kind = ParseKind(node.value("kind", std::string("Fixed")));
          group.pivot = ReadArray<3>(node, "pivotMeters", {});
          group.axis = ReadArray<3>(node, "axis", {0, 1, 0});
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
          part.mass = override.value("massKg", part.mass);
          part.friction = override.value("frictionEstimate", part.friction);
          part.color = ReadArray<4>(override, "colorRgba", part.color);
          part.preview_visible = override.value("previewVisible", true);
        }
      }
    }
    for (auto& part : loaded) if (part.assembly < -1 || part.assembly >= int(groups.size())) part.assembly = 0;
    manifest_ = std::filesystem::weakly_canonical(manifest);
    design_name_ = source.value("designName", std::string("Fusion bot"));
    parts_ = std::move(loaded);
    assemblies_ = std::move(groups);
    const auto path = manifest_.string();
    std::snprintf(path_buffer_, sizeof(path_buffer_), "%s", path.c_str());
    error_.clear();
    selected_part_ = -1; hovered_part_ = -1; hovered_loop_.clear(); hole_cache_.clear();
    pick_message_.clear(); axis_points_.clear(); axis_picking_ = false;
    axis_assembly_ = assemblies_.size() > 1 ? 1 : 0;
    return true;
  } catch (const std::exception& e) { error_ = e.what(); return false; }
}

bool FusionBundle::Save() {
  try {
    if (!HasBundle()) throw std::runtime_error("Load a Fusion manifest first.");
    json state;
    state["schemaVersion"] = 1;
    state["designName"] = design_name_;
    state["assemblies"] = json::array();
    for (size_t i = 1; i < assemblies_.size(); ++i) {
      const auto& g = assemblies_[i];
      state["assemblies"].push_back({{"name", g.name}, {"kind", KindLabel(g.kind)},
                                      {"pivotMeters", g.pivot}, {"axis", g.axis}});
    }
    state["partAssignments"] = json::object();
    state["partOverrides"] = json::object();
    for (const auto& part : parts_) {
      state["partAssignments"][part.file] = part.assembly;
      state["partOverrides"][part.file] = {{"massKg", part.mass},
        {"frictionEstimate", part.friction}, {"colorRgba", part.color},
        {"previewVisible", part.preview_visible}};
    }
    std::ofstream output(manifest_.parent_path() / "combatsim_setup.json", std::ios::trunc);
    if (!output) throw std::runtime_error("Cannot write combatsim_setup.json beside the manifest.");
    output << state.dump(2) << '\n';
    if (!output) throw std::runtime_error("Failed writing combatsim_setup.json.");
    error_.clear();
    return true;
  } catch (const std::exception& e) { error_ = e.what(); return false; }
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
    std::ostringstream xml; xml.imbue(std::locale::classic());
    xml << "<mujoco model=\"CAD setup\"><asset>\n";
    for (size_t i = 0; i < parts_.size(); ++i) {
      const auto& part = parts_[i];
      xml << "<mesh name=\"cad_mesh_" << i << "\" file=\"" << Escape(part.file)
          << "\" scale=\"" << Text({part.scale, part.scale, part.scale}) << "\"/>\n";
    }
    xml << "</asset><worldbody><body name=\"CAD_Preview\">\n";
    for (size_t i = 0; i < parts_.size(); ++i) {
      const auto& part = parts_[i];
      xml << "<geom name=\"cad_part_" << i << "\" type=\"mesh\" mesh=\"cad_mesh_" << i
          << "\" pos=\"" << Text(part.position) << "\" quat=\"" << Text4(part.rotation)
          << "\" rgba=\"" << Text4(part.color) << "\"/>\n";
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

bool FusionBundle::Generate(std::filesystem::path& output) {
  try {
    if (!HasBundle()) throw std::runtime_error("Load a Fusion manifest first.");
    for (const auto& part : parts_) if (part.assembly >= 0 && !ValidMass(part.mass))
      throw std::runtime_error("Set a positive mass for included body: " + part.name + " (" + part.file + ")");
    const auto frame = GroupMass(parts_, 0, {});
    if (!ValidMass(frame.mass)) throw std::runtime_error("Assign at least one body with Fusion mass to Main frame.");
    Vec3 low, high;
    low.fill(std::numeric_limits<double>::infinity()); high.fill(-std::numeric_limits<double>::infinity());
    for (const auto& part : parts_) if (part.assembly >= 0) {
      Include(part.min_bound, low, high); Include(part.max_bound, low, high);
    }
    const Vec3 center = Mul(Add(low, high), 0.5);
    const Vec3 spawn{-0.62-center[0], -center[1], 0.012-low[2]};
    std::ostringstream xml; xml.imbue(std::locale::classic());
    xml << "<mujoco model=\"" << Escape(design_name_) << "\">\n"
        << "<compiler angle=\"radian\" autolimits=\"true\"/>\n"
        << "<option timestep=\"0.0004\" gravity=\"0 0 -9.81\" integrator=\"implicitfast\"/>\n"
        << "<asset>\n";
    for (size_t i = 0; i < parts_.size(); ++i) if (parts_[i].assembly >= 0) {
      const auto& part = parts_[i];
      xml << "<mesh name=\"cad_mesh_" << i << "\" file=\"" << Escape(part.file)
          << "\" scale=\"" << Text({part.scale, part.scale, part.scale}) << "\"/>\n";
    }
    xml << "</asset>\n<worldbody>\n"
        << "<geom name=\"floor\" type=\"plane\" size=\"1 1 0.02\" friction=\"1.4 0.02 0.001\" rgba=\"0.18 0.22 0.27 1\"/>\n"
        << "<geom name=\"north_wall\" type=\"box\" pos=\"0 1.01 0.5\" size=\"1.02 0.01 0.5\" rgba=\"0.4 0.67 0.82 0.18\"/>\n"
        << "<geom name=\"south_wall\" type=\"box\" pos=\"0 -1.01 0.5\" size=\"1.02 0.01 0.5\" rgba=\"0.4 0.67 0.82 0.18\"/>\n"
        << "<geom name=\"east_wall\" type=\"box\" pos=\"1.01 0 0.5\" size=\"0.01 1 0.5\" rgba=\"0.4 0.67 0.82 0.18\"/>\n"
        << "<geom name=\"west_wall\" type=\"box\" pos=\"-1.01 0 0.5\" size=\"0.01 1 0.5\" rgba=\"0.4 0.67 0.82 0.18\"/>\n"
        << "<geom name=\"roof\" type=\"box\" pos=\"0 0 1.01\" size=\"1.02 1.02 0.01\" rgba=\"0.4 0.67 0.82 0.12\"/>\n"
        << "<body name=\"Player_Robot\" pos=\"" << Text(spawn) << "\">\n"
        << "<freejoint name=\"robot_free\"/>\n"
        << "<inertial pos=\"" << Text(frame.center) << "\" mass=\"" << Number(frame.mass)
        << "\" diaginertia=\"" << Text(frame.inertia) << "\"/>\n";
    auto geom = [&](size_t i, Vec3 origin) {
      const auto& p = parts_[i];
      xml << "<geom name=\"cad_part_" << i << "\" type=\"mesh\" mesh=\"cad_mesh_" << i
          << "\" pos=\"" << Text(Sub(p.position, origin)) << "\" quat=\"" << Text4(p.rotation)
          << "\" mass=\"0\" friction=\"" << Number(p.friction)
          << " 0.02 0.001\" rgba=\"" << Text4(p.color) << "\"/>\n";
    };
    for (size_t i = 0; i < parts_.size(); ++i) if (parts_[i].assembly == 0) geom(i, {});
    for (size_t group = 1; group < assemblies_.size(); ++group) {
      const auto& a = assemblies_[group];
      const auto props = GroupMass(parts_, int(group), a.pivot);
      if (props.mass <= 0) continue;
      xml << "<body name=\"cad_assembly_" << group << "\" pos=\"" << Text(a.pivot) << "\">\n"
          << "<inertial pos=\"" << Text(props.center) << "\" mass=\"" << Number(props.mass)
          << "\" diaginertia=\"" << Text(props.inertia) << "\"/>\n";
      if (a.kind != AssemblyKind::Fixed) {
        Vec3 axis = a.axis;
        const double length = Length(axis);
        if (length < 1e-8) throw std::runtime_error("Rotation axis cannot be zero: " + a.name);
        axis = Mul(axis, 1.0/length);
        xml << "<joint name=\"cad_joint_" << group << "\" type=\"hinge\" axis=\""
            << Text(axis) << "\" damping=\"0.00001\"/>\n";
      }
      for (size_t i = 0; i < parts_.size(); ++i) if (parts_[i].assembly == int(group)) geom(i, a.pivot);
      xml << "</body>\n";
    }
    xml << "</body>\n"
        << "<body name=\"Target_Robot\" pos=\"0.55 0 0.08\" quat=\"0 0 0 1\">"
        << "<freejoint/><geom name=\"target_chassis\" type=\"box\" size=\"0.17 0.13 0.045\" mass=\"0.5\" rgba=\"0.82 0.16 0.14 1\"/>"
        << "</body>\n</worldbody>\n<actuator>\n";
    for (size_t group = 1; group < assemblies_.size(); ++group) {
      const auto& a = assemblies_[group];
      if (a.kind == AssemblyKind::Fixed || GroupMass(parts_, int(group), a.pivot).mass <= 0) continue;
      const double force = a.kind == AssemblyKind::Weapon ? 0.25 : 0.08;
      const double gain = a.kind == AssemblyKind::Weapon ? 0.001 : 0.004;
      xml << "<velocity name=\"cad_assembly_" << group << "_drive\" joint=\"cad_joint_" << group
          << "\" kv=\"" << Number(gain) << "\" forcerange=\"-" << Number(force)
          << ' ' << Number(force) << "\"/>\n";
    }
    xml << "</actuator>\n</mujoco>\n";
    output = manifest_.parent_path() / "combatsim_generated.xml";
    std::ofstream file(output, std::ios::trunc);
    if (!file) throw std::runtime_error("Cannot write generated MJCF beside manifest.");
    file << xml.str();
    if (!file) throw std::runtime_error("Failed to write generated MJCF.");
    error_.clear();
    return true;
  } catch (const std::exception& e) { error_ = e.what(); return false; }
}

void FusionBundle::BindControls(const mjModel* model, std::vector<int>& left,
                                std::vector<int>& right, std::vector<int>& weapon) const {
  for (size_t group = 1; group < assemblies_.size(); ++group) {
    const auto kind = assemblies_[group].kind;
    const auto name = "cad_assembly_" + std::to_string(group) + "_drive";
    const int id = mj_name2id(model, mjOBJ_ACTUATOR, name.c_str());
    if (id < 0) continue;
    if (kind == AssemblyKind::LeftWheel) left.push_back(id);
    if (kind == AssemblyKind::RightWheel) right.push_back(id);
    if (kind == AssemblyKind::Weapon) weapon.push_back(id);
  }
}

void FusionBundle::PickFace(int part_index, const std::array<double, 3>& cad_point) {
  if (part_index < 0 || part_index >= int(parts_.size())) return;
  selected_part_ = part_index;
  const auto& part = parts_[part_index];
  if (part.assembly <= 0) {
    pick_message_ = "Selected " + part.name + ". Assign it to a weapon or wheel assembly first.";
    return;
  }
  std::ifstream file(part.absolute_file, std::ios::binary);
  file.seekg(80);
  uint32_t count = 0; file.read(reinterpret_cast<char*>(&count), 4);
  double nearest = std::numeric_limits<double>::infinity();
  Vec3 center{};
  for (uint32_t t = 0; t < count && file; ++t) {
    char triangle[50]; file.read(triangle, sizeof(triangle));
    if (!file) break;
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
  pick_message_ = "Selected " + parts_[part_index].name + ". Choose its rigid assembly below.";
}

void FusionBundle::UpdateHover(int part_index, const std::array<double, 3>& hit) {
  hovered_part_ = PreviewVisible(part_index) ? part_index : -1;
  hovered_loop_.clear();
  if (!axis_picking_ || hovered_part_ < 0 ||
      parts_[hovered_part_].assembly != axis_assembly_) return;
  auto [found, inserted] = hole_cache_.try_emplace(hovered_part_);
  if (inserted) found->second = ReadFeatureLoops(parts_[hovered_part_]);
  const double limit = std::max(0.002, 0.035*Length(Sub(
    parts_[hovered_part_].max_bound, parts_[hovered_part_].min_bound)));
  double nearest = limit;
  for (const auto& loop : found->second) {
    const double distance = DistanceToLoop(loop, hit);
    if (distance < nearest) { nearest = distance; hovered_loop_ = loop; }
  }
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
  if (assembly <= 0 || assembly >= int(assemblies_.size()) ||
      assemblies_[assembly].kind == AssemblyKind::Fixed) {
    pick_message_ = "Choose a weapon or wheel assembly for the rotation axis.";
    return;
  }
  axis_assembly_ = assembly;
  axis_points_.clear(); axis_picking_ = true;
  pick_message_ = "Hover a hole rim until its complete loop lights up, then click once.";
}

bool FusionBundle::AddAxisPoint(int part_index, const std::array<double, 3>& cad_point,
                                bool snap_to_mesh_edge) {
  if (!axis_picking_ || part_index < 0 || part_index >= int(parts_.size())) return false;
  selected_part_ = part_index;
  if (axis_assembly_ <= 0 || axis_assembly_ >= int(assemblies_.size()) ||
      parts_[part_index].assembly != axis_assembly_) {
    pick_message_ = "Assign this part to the chosen axis assembly before selecting its hole.";
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
                  " points. Click around the rim, then press Fit axis through hole.";
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
  pick_message_ = "Fitted " + assembly.name + " axis through the hole center using " +
                  std::to_string(axis_points_.size()) + " points.";
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
  std::ifstream file(part.absolute_file, std::ios::binary);
  file.seekg(80);
  uint32_t count = 0; file.read(reinterpret_cast<char*>(&count), 4);
  double nearest = std::numeric_limits<double>::infinity();
  Vec3 midpoint{}, direction{};
  for (uint32_t t = 0; t < count && file; ++t) {
    char triangle[50]; file.read(triangle, sizeof(triangle));
    if (!file) break;
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

void FusionBundle::AddAxisMarkers(const mjModel* model, const mjData* data, mjvScene& scene) const {
  for (size_t group = 1; group < assemblies_.size(); ++group) {
    const auto& assembly = assemblies_[group];
    const bool assigned = std::any_of(parts_.begin(), parts_.end(), [group](const CadPart& part) {
      return part.assembly == int(group);
    });
    if (!assigned) continue;
    if (assembly.kind == AssemblyKind::Fixed || scene.ngeom + 2 > scene.maxgeom) continue;
    const std::string body_name = "cad_assembly_" + std::to_string(group);
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
    const float* color = assembly.kind == AssemblyKind::Weapon ? weapon_color : wheel_color;
    mjv_initGeom(&scene.geoms[scene.ngeom], mjGEOM_LINE, nullptr, nullptr, nullptr, color);
    mjv_connector(&scene.geoms[scene.ngeom++], mjGEOM_LINE, 3, start, end);
    const mjtNum size[3] = {0.009, 0.009, 0.009};
    mjv_initGeom(&scene.geoms[scene.ngeom++], mjGEOM_SPHERE, size, origin, nullptr, color);
  }
}

void FusionBundle::AddSetupAxisMarkers(mjvScene& scene) const {
  if (axis_picking_ && hovered_loop_.size() >= 3) {
    const float hover_color[4] = {0.2f, 1.0f, 0.85f, 1.0f};
    for (size_t i = 0; i < hovered_loop_.size() && scene.ngeom < scene.maxgeom; ++i) {
      const auto& first = hovered_loop_[i];
      const auto& second = hovered_loop_[(i+1)%hovered_loop_.size()];
      mjtNum a[3] = {first[0], first[1], first[2]};
      mjtNum b[3] = {second[0], second[1], second[2]};
      mjv_initGeom(&scene.geoms[scene.ngeom], mjGEOM_LINE, nullptr, nullptr, nullptr, hover_color);
      mjv_connector(&scene.geoms[scene.ngeom++], mjGEOM_LINE, 3, a, b);
    }
  }
  const float picked_color[4] = {0.25f, 1.0f, 0.3f, 1.0f};
  const mjtNum point_size[3] = {0.005, 0.005, 0.005};
  for (const auto& point : axis_points_) {
    if (scene.ngeom >= scene.maxgeom) break;
    mjtNum pos[3] = {point[0], point[1], point[2]};
    mjv_initGeom(&scene.geoms[scene.ngeom++], mjGEOM_SPHERE, point_size, pos, nullptr, picked_color);
  }
  for (size_t i = 1; i < axis_points_.size() && scene.ngeom < scene.maxgeom; ++i) {
    mjtNum a[3] = {axis_points_[i-1][0], axis_points_[i-1][1], axis_points_[i-1][2]};
    mjtNum b[3] = {axis_points_[i][0], axis_points_[i][1], axis_points_[i][2]};
    mjv_initGeom(&scene.geoms[scene.ngeom], mjGEOM_LINE, nullptr, nullptr, nullptr, picked_color);
    mjv_connector(&scene.geoms[scene.ngeom++], mjGEOM_LINE, 2, a, b);
  }
  if (axis_points_.size() >= 3 && scene.ngeom < scene.maxgeom) {
    const auto& last = axis_points_.back();
    const auto& first = axis_points_.front();
    mjtNum a[3] = {last[0], last[1], last[2]};
    mjtNum b[3] = {first[0], first[1], first[2]};
    mjv_initGeom(&scene.geoms[scene.ngeom], mjGEOM_LINE, nullptr, nullptr, nullptr, picked_color);
    mjv_connector(&scene.geoms[scene.ngeom++], mjGEOM_LINE, 2, a, b);
  }
  for (size_t group = 1; group < assemblies_.size(); ++group) {
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
    const float* color = assembly.kind == AssemblyKind::Weapon ? weapon_color : wheel_color;
    mjv_initGeom(&scene.geoms[scene.ngeom], mjGEOM_LINE, nullptr, nullptr, nullptr, color);
    mjv_connector(&scene.geoms[scene.ngeom++], mjGEOM_LINE, 3, start, end);
    const mjtNum size[3] = {0.009, 0.009, 0.009};
    mjv_initGeom(&scene.geoms[scene.ngeom++], mjGEOM_SPHERE, size, center, nullptr, color);
  }
}

void FusionBundle::DrawPart(size_t index) {
  auto& part = parts_[index];
  ImGui::PushID(int(index));
  const std::string label = part.name + "##part";
  if (ImGui::TreeNode(label.c_str())) {
    ImGui::Checkbox("Show in preview", &part.preview_visible);
    ImGui::TextWrapped("STL: %s", part.file.c_str());
    ImGui::TextWrapped("Fusion material: %s", part.material.c_str());
    if (!part.appearance.empty()) ImGui::TextWrapped("Appearance: %s", part.appearance.c_str());
    if (part.roughness >= 0) ImGui::Text("Fusion roughness: %.2f", part.roughness);
    ImGui::Text("Density: %.0f kg/m3", part.density);
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
        }
      }
      ImGui::EndCombo();
    }
    ImGui::Text("Fusion weight: %.2f g", part.fusion_mass * 1000.0);
    double mass_grams = part.mass * 1000.0;
    if (ImGui::InputDouble("Weight (g)", &mass_grams, 0.1, 1.0, "%.2f"))
      part.mass = std::max(0.0, mass_grams / 1000.0);
    ImGui::SameLine();
    if (ImGui::SmallButton("Use Fusion")) part.mass = part.fusion_mass;
    ImGui::DragScalar("Friction estimate", ImGuiDataType_Double, &part.friction, 0.01, nullptr, nullptr, "%.2f");
    part.mass = std::max(0.0, part.mass);
    part.friction = std::max(0.01, part.friction);
    float color[4] = {float(part.color[0]), float(part.color[1]), float(part.color[2]), float(part.color[3])};
    if (ImGui::ColorEdit4("Color", color)) for (int i = 0; i < 4; ++i) part.color[i] = color[i];
    ImGui::TreePop();
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
    if (ImGui::TreeNode(child.c_str())) {
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
      if (ImGui::BeginCombo("Assign this folder", "Choose assembly...")) {
        for (int group = -1; group < int(assemblies_.size()); ++group) {
          const char* label = group < 0 ? "Exclude" : assemblies_[group].name.c_str();
          if (ImGui::Selectable(label)) {
            for (auto& part : parts_) {
              const auto& part_path = part.occurrence_path;
              if (part_path.size() >= path.size() && std::equal(path.begin(), path.end(), part_path.begin()))
                part.assembly = group;
            }
            if (group > 0) assemblies_[group].pivot = GroupCenter(group);
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
  ImGui::Text("%s: %d STL bodies", design_name_.c_str(), int(parts_.size()));
  if (selected_part_ >= 0 && selected_part_ < int(parts_.size())) {
    auto& selected = parts_[selected_part_];
    ImGui::SeparatorText("Selected in 3D preview");
    ImGui::TextWrapped("%s", selected.name.c_str());
    if (ImGui::Button(selected.preview_visible ? "Hide selected" : "Show selected"))
      selected.preview_visible = !selected.preview_visible;
    ImGui::SameLine();
    if (ImGui::Button("Deselect part")) selected_part_ = -1;
    const char* chosen = selected.assembly < 0 ? "Exclude" : assemblies_[selected.assembly].name.c_str();
    if (ImGui::BeginCombo("Assign selected part", chosen)) {
      if (ImGui::Selectable("Exclude", selected.assembly < 0)) selected.assembly = -1;
      for (size_t i = 0; i < assemblies_.size(); ++i)
        if (ImGui::Selectable(assemblies_[i].name.c_str(), selected.assembly == int(i))) {
          selected.assembly = int(i);
          if (i > 0) assemblies_[i].pivot = GroupCenter(int(i));
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
  double total = 0;
  for (const auto& p : parts_) if (p.assembly >= 0) total += p.mass;
  ImGui::Text("Assigned mass: %.1f g / 500 g", total * 1000.0);
  if (total > 0.5) ImGui::TextWrapped("Mass exceeds the 500 g class. Check for duplicate CAD bodies or adjust overrides.");
  if (ImGui::Button("Save setup")) Save();
  ImGui::SameLine();
  if (ImGui::Button("Apply and preview")) {
    if (Save()) apply_requested = true;
  }
  ImGui::SeparatorText("Rigid assemblies");
  ImGui::TextWrapped("Assign every screw, blade, and hub to the same weapon or wheel group. Each group gets one joint and one axis.");
  for (size_t i = 1; i < assemblies_.size(); ++i) {
    auto& group = assemblies_[i];
    ImGui::PushID(int(i));
    if (ImGui::TreeNodeEx(group.name.c_str(), ImGuiTreeNodeFlags_DefaultOpen)) {
      char name[128]; std::snprintf(name, sizeof(name), "%s", group.name.c_str());
      if (ImGui::InputText("Name", name, sizeof(name))) group.name = name;
      if (ImGui::BeginCombo("Type", KindLabel(group.kind))) {
        for (const auto kind : {AssemblyKind::Fixed, AssemblyKind::LeftWheel,
                                AssemblyKind::RightWheel, AssemblyKind::Weapon})
          if (ImGui::Selectable(KindLabel(kind), group.kind == kind)) group.kind = kind;
        ImGui::EndCombo();
      }
      ImGui::Text("Rotation center (meters)");
      ImGui::InputScalarN("Center X/Y/Z", ImGuiDataType_Double, group.pivot.data(), 3);
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
}

void FusionBundle::DrawAxisSelectionControls() {
  if (!HasBundle()) return;
  ImGui::SeparatorText("Select rotation axis");
  const char* axis_name = axis_assembly_ > 0 && axis_assembly_ < int(assemblies_.size()) ?
    assemblies_[axis_assembly_].name.c_str() : "Choose assembly";
  ImGui::SetNextItemWidth(std::max(180.0f, ImGui::GetContentRegionAvail().x * 0.65f));
  if (ImGui::BeginCombo("Axis target (weapon / wheel)", axis_name)) {
    for (size_t i = 1; i < assemblies_.size(); ++i) {
      if (assemblies_[i].kind == AssemblyKind::Fixed) continue;
      const std::string label = assemblies_[i].name + " (" + KindLabel(assemblies_[i].kind) + ")";
      if (ImGui::Selectable(label.c_str(), int(i) == axis_assembly_)) {
        axis_assembly_ = int(i); axis_points_.clear();
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
