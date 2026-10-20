#pragma once

#include <mujoco/mujoco.h>
#include <SDL3/SDL.h>
#include <array>
#include <filesystem>
#include <mutex>
#include <string>
#include <vector>

enum class AssemblyKind { Frame, Fixed, LeftWheel, RightWheel, Weapon };

struct CadPart {
  std::string name;
  std::string file;
  std::filesystem::path absolute_file;
  std::vector<std::string> occurrence_path;
  std::string material;
  std::string appearance;
  std::vector<std::string> material_properties;
  double density = 0.0;
  double mass = 0.0;
  double fusion_mass = 0.0;
  double friction = 0.6;
  double roughness = -1.0;
  double scale = 0.001;
  std::array<double, 3> position{};
  std::array<double, 4> rotation{1, 0, 0, 0};
  std::array<double, 4> color{0.7, 0.72, 0.75, 1};
  std::array<double, 3> min_bound{};
  std::array<double, 3> max_bound{};
  int assembly = 0;  // -1 excludes this body.
  bool preview_visible = true;
};

struct CadAssembly {
  std::string name;
  AssemblyKind kind = AssemblyKind::Fixed;
  std::array<double, 3> pivot{};
  std::array<double, 3> axis{0, 1, 0};
};

class FusionBundle {
 public:
  bool Load(const std::filesystem::path& manifest);
  bool Save();
  bool Generate(std::filesystem::path& output);
  bool GeneratePreview(std::filesystem::path& output);
  void Draw(SDL_Window* window, bool& load_requested, bool& apply_requested);
  void DrawAxisSelectionControls();
  void SelectPart(int part_index);
  void ClearSelection() { selected_part_ = -1; }
  bool PreviewVisible(int part_index) const;
  bool AxisPicking() const { return axis_picking_; }
  void StartAxis(int assembly);
  bool AddAxisPoint(int part_index, const std::array<double, 3>& cad_point,
                    bool snap_to_mesh_edge = true);
  bool FinishAxis();
  void CancelAxis();
  int AxisPointCount() const { return int(axis_points_.size()); }
  int AxisAssembly() const { return axis_assembly_; }
  const CadAssembly& Assembly(int index) const { return assemblies_.at(index); }
  void PickEdge(int part_index, const std::array<double, 3>& cad_point);
  int SelectedPart() const { return selected_part_; }
  std::array<double, 3> PreviewCenter() const;
  double PreviewDistance() const;
  void BindControls(const mjModel* model, std::vector<int>& left,
                    std::vector<int>& right, std::vector<int>& weapon) const;
  void PickFace(int part_index, const std::array<double, 3>& cad_point);
  void AddAxisMarkers(const mjModel* model, const mjData* data, mjvScene& scene) const;
  void AddSetupAxisMarkers(mjvScene& scene) const;
  bool HasBundle() const { return !parts_.empty(); }
  const std::string& Error() const { return error_; }

 private:
  void DrawPart(size_t index);
  void DrawTree(const std::vector<std::string>& parent);
  std::array<double, 3> GroupCenter(int assembly) const;
  std::filesystem::path manifest_;
  std::string design_name_;
  std::string error_;
  std::vector<CadPart> parts_;
  std::vector<CadAssembly> assemblies_;
  char path_buffer_[1024]{};
  int selected_part_ = -1;
  bool axis_picking_ = false;
  int axis_assembly_ = 1;
  std::vector<std::array<double, 3>> axis_points_;
  std::string pick_message_;
  std::mutex dialog_mutex_;
  std::string pending_path_;
};
