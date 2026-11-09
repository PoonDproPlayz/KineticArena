#include "FusionBundle.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace {
std::string arena_xml_escape(const std::string& value) {
  std::string result;
  for (const char character : value) {
    switch (character) {
      case '&': result += "&amp;"; break;
      case '"': result += "&quot;"; break;
      case '<': result += "&lt;"; break;
      case '>': result += "&gt;"; break;
      default: result += character; break;
    }
  }
  return result;
}

std::string arena_number(double value) {
  std::ostringstream output;
  output.imbue(std::locale::classic());
  output << std::setprecision(12) << value;
  return output.str();
}

std::string attribute_value(const std::string& tag, const std::string& name,
                            const std::string& fallback = {}) {
  const std::string prefix = name + "=\"";
  const size_t begin = tag.find(prefix);
  if (begin == std::string::npos) return fallback;
  const size_t value_begin = begin + prefix.size();
  const size_t end = tag.find('"', value_begin);
  return end == std::string::npos ? fallback :
    tag.substr(value_begin, end - value_begin);
}

int integer_attribute(const std::string& tag, const std::string& name,
                      int fallback = 1) {
  const std::string value = attribute_value(tag, name);
  if (value.empty()) return fallback;
  try { return std::stoi(value); }
  catch (const std::exception&) { return fallback; }
}

double geom_sliding_friction(const std::string& tag) {
  const std::string value = attribute_value(tag, "friction");
  if (value.empty()) return 1.0;  // MuJoCo's default sliding coefficient.
  std::istringstream input(value);
  double sliding = 1.0;
  if (!(input >> sliding) || !std::isfinite(sliding) || sliding < 0.0) return 1.0;
  return std::clamp(sliding, 0.0, 5.0);
}

struct ArenaSurface {
  std::string geom_name;
  double friction = 0.6;
};

template <size_t N>
std::string arena_values(const std::array<double, N>& values) {
  std::ostringstream output;
  output.imbue(std::locale::classic());
  output << std::setprecision(12);
  for (size_t index = 0; index < N; ++index) {
    if (index) output << ' ';
    output << values[index];
  }
  return output.str();
}

void set_attribute(std::string& tag, const std::string& name,
                   const std::string& value) {
  const std::string prefix = name + "=\"";
  const size_t found = tag.find(prefix);
  if (found != std::string::npos) {
    const size_t begin = found + prefix.size();
    const size_t end = tag.find('"', begin);
    if (end != std::string::npos) {
      tag.replace(begin, end - begin, value);
      return;
    }
  }
  const size_t end = tag.rfind("/>");
  if (end == std::string::npos)
    throw std::runtime_error("Cannot edit built-in arena geometry.");
  tag.insert(end, " " + name + "=\"" + value + "\"");
}

void disable_default_geom(std::string& xml, const std::string& name,
                          bool hide_visual) {
  const size_t begin = xml.find("<geom name=\"" + name + "\"");
  if (begin == std::string::npos) return;
  const size_t end = xml.find("/>", begin);
  if (end == std::string::npos)
    throw std::runtime_error("Cannot find built-in arena geom " + name + ".");
  std::string tag = xml.substr(begin, end + 2 - begin);
  set_attribute(tag, "contype", "0");
  set_attribute(tag, "conaffinity", "0");
  if (hide_visual) set_attribute(tag, "group", "4");
  xml.replace(begin, end + 2 - begin, tag);
}
}  // namespace

bool FusionBundle::InjectArenaIntoMjcf(
    std::string& model_xml, std::vector<MemoryBundleAsset>* assets) {
  try {
    if (!HasBundle()) throw std::runtime_error("Load a Fusion arena manifest first.");
    if (model_xml.find("arena_collider_") != std::string::npos)
      throw std::runtime_error("This model already contains the imported arena.");
    if (model_xml.find("</asset>") == std::string::npos ||
        model_xml.find("</worldbody>") == std::string::npos)
      throw std::runtime_error("The MuJoCo model is missing an asset or worldbody section.");

    // Replace the stock floor and wall collision proxies with imported CAD.
    for (const char* name : {"floor", "north_wall", "south_wall", "east_wall",
                             "west_wall", "roof"})
      disable_default_geom(model_xml, name, false);
    for (const char* name : {"floor_visual", "north_wall_acrylic", "south_wall_acrylic",
                             "east_wall_acrylic", "west_wall_acrylic", "roof_acrylic"})
      disable_default_geom(model_xml, name, true);

    // Keep the original robot-side coefficient so explicit arena contacts can
    // combine both surfaces instead of letting arena geom priority replace it.
    std::vector<std::pair<std::string, double>> robot_geoms;
    const size_t world_begin = model_xml.find("<worldbody>");
    const size_t world_end = model_xml.find("</worldbody>", world_begin);
    if (world_begin == std::string::npos || world_end == std::string::npos)
      throw std::runtime_error("The MuJoCo model has an invalid worldbody section.");
    size_t geom_begin = world_begin;
    while ((geom_begin = model_xml.find("<geom", geom_begin)) != std::string::npos &&
           geom_begin < world_end) {
      const size_t geom_end = model_xml.find("/>", geom_begin);
      if (geom_end == std::string::npos || geom_end >= world_end)
        throw std::runtime_error("Cannot read a geom in the MuJoCo worldbody.");
      const std::string tag = model_xml.substr(geom_begin, geom_end + 2 - geom_begin);
      const std::string name = attribute_value(tag, "name");
      const int contype = integer_attribute(tag, "contype");
      const int conaffinity = integer_attribute(tag, "conaffinity");
      if (!name.empty() && (contype != 0 || conaffinity != 0))
        robot_geoms.emplace_back(name, geom_sliding_friction(tag));
      geom_begin = geom_end + 2;
    }

    // Explicit pairs bypass geom masks, so remove pairs tied to disabled stock
    // arena surfaces before adding contacts for the imported arena.
    const auto is_default_arena_geom = [](const std::string& name) {
      for (const char* candidate : {"floor", "north_wall", "south_wall",
                                    "east_wall", "west_wall", "roof"})
        if (name == candidate) return true;
      return false;
    };
    const size_t contact_begin = model_xml.find("<contact>");
    size_t contact_end = contact_begin == std::string::npos
      ? std::string::npos : model_xml.find("</contact>", contact_begin);
    if (contact_begin != std::string::npos && contact_end != std::string::npos) {
      size_t pair_begin = contact_begin;
      while ((pair_begin = model_xml.find("<pair", pair_begin)) != std::string::npos &&
             pair_begin < contact_end) {
        const size_t pair_end = model_xml.find("/>", pair_begin);
        if (pair_end == std::string::npos || pair_end >= contact_end) break;
        const std::string tag = model_xml.substr(pair_begin, pair_end + 2 - pair_begin);
        if (is_default_arena_geom(attribute_value(tag, "geom1")) ||
            is_default_arena_geom(attribute_value(tag, "geom2"))) {
          model_xml.erase(pair_begin, pair_end + 2 - pair_begin);
          contact_end -= pair_end + 2 - pair_begin;
        } else {
          pair_begin = pair_end + 2;
        }
      }
    }

    std::ostringstream asset_xml;
    std::ostringstream world_xml;
    std::ostringstream contact_xml;
    asset_xml.imbue(std::locale::classic());
    world_xml.imbue(std::locale::classic());
    contact_xml.imbue(std::locale::classic());
    std::vector<ArenaSurface> arena_surfaces;
    for (int index = 0; index < PartCount(); ++index) {
      const CadPart* part = PartAt(index);
      if (!part || !part->collision_enabled) continue;
      const std::string mesh_name = "arena_mesh_" + std::to_string(index);
      std::string mesh_file;
      if (assets) {
        mesh_file = "arena_asset_" + std::to_string(index) + ".stl";
        std::vector<unsigned char> bytes = part->mesh_data;
        if (bytes.empty()) {
          std::ifstream input(part->absolute_file, std::ios::binary);
          if (!input) throw std::runtime_error("Cannot read arena STL: " + part->file);
          bytes.assign(std::istreambuf_iterator<char>(input),
                       std::istreambuf_iterator<char>());
        }
        if (bytes.empty()) throw std::runtime_error("Arena STL is empty: " + part->file);
        assets->push_back({mesh_file, std::move(bytes)});
      } else {
        if (part->absolute_file.empty() ||
            !std::filesystem::is_regular_file(part->absolute_file))
          throw std::runtime_error("Arena STL is missing: " + part->file);
        mesh_file = part->absolute_file.generic_u8string();
      }
      const std::string material_name = "arena_surface_" + std::to_string(index);
      asset_xml << "<mesh name=\"" << mesh_name << "\" file=\""
                << arena_xml_escape(mesh_file) << "\" scale=\""
                << arena_values(std::array<double, 3>{part->scale, part->scale, part->scale})
                << "\" smoothnormal=\"true\"/>\n"
                << "<material name=\"" << material_name << "\" rgba=\""
                << arena_values(part->color) << "\" specular=\"0.25\" shininess=\"0.25\"";
#if defined(COMBATSIM_USE_FILAMENT)
      asset_xml << " metallic=\"0\" roughness=\""
                << arena_number(std::isfinite(part->roughness) && part->roughness >= 0.0
                                  ? part->roughness : 0.65)
                << '"';
#endif
      asset_xml << "/>\n";
      const double friction = std::clamp(
        std::isfinite(part->friction) ? part->friction : 0.6, 0.0, 5.0);
      arena_surfaces.push_back({"arena_collider_" + std::to_string(index), friction});
      world_xml << "<geom name=\"arena_collider_" << index
                << "\" type=\"mesh\" mesh=\"" << mesh_name
                << "\" pos=\"" << arena_values(part->position)
                << "\" quat=\"" << arena_values(part->rotation)
                << "\" mass=\"0\" friction=\"" << arena_number(friction)
                << " 0.005 0.0001\" priority=\"1\" group=\"5\""
                << " contype=\"2\" conaffinity=\"5\" rgba=\""
                << arena_values(part->color) << "\"/>\n"
                << "<geom name=\"arena_visual_" << index
                << "\" type=\"mesh\" mesh=\"" << mesh_name
                << "\" pos=\"" << arena_values(part->position)
                << "\" quat=\"" << arena_values(part->rotation)
                << "\" material=\"" << material_name
                << "\" mass=\"0\" contype=\"0\" conaffinity=\"0\"/>\n";
    }
    if (arena_surfaces.empty())
      throw std::runtime_error("The Fusion arena has no bodies enabled for collision.");

    // Use an explicit per-pair estimate so the robot coefficient participates
    // alongside the arena material instead of being discarded by geom priority.
    for (const ArenaSurface& surface : arena_surfaces) {
      for (const auto& [robot_name, robot_friction] : robot_geoms) {
        const double pair_friction = std::sqrt(surface.friction * robot_friction);
        contact_xml << "<pair geom1=\"" << surface.geom_name << "\" geom2=\""
                    << arena_xml_escape(robot_name)
                    << "\" condim=\"3\" friction=\""
                    << arena_number(pair_friction) << ' ' << arena_number(pair_friction)
                    << " 0.02 0.001 0.001\" solref=\"0.003 1\""
                       " solimp=\"0.95 0.99 0.001\"/>\n";
      }
    }
    model_xml.insert(model_xml.find("</asset>"), asset_xml.str());
    model_xml.insert(model_xml.find("</worldbody>"), world_xml.str());
    const size_t updated_contact_begin = model_xml.find("<contact>");
    if (updated_contact_begin == std::string::npos) {
      const size_t model_end = model_xml.rfind("</mujoco>");
      if (model_end == std::string::npos)
        throw std::runtime_error("The MuJoCo model has no closing mujoco element.");
      model_xml.insert(model_end, "<contact>\n" + contact_xml.str() + "</contact>\n");
    } else {
      const size_t updated_contact_end = model_xml.find("</contact>", updated_contact_begin);
      if (updated_contact_end == std::string::npos)
        throw std::runtime_error("The MuJoCo model has an invalid contact section.");
      model_xml.insert(updated_contact_end, contact_xml.str());
    }
    error_.clear();
    return true;
  } catch (const std::exception& error) {
    error_ = error.what();
    return false;
  }
}
