#pragma once

#include <mujoco/mujoco.h>
#include <SDL3/SDL.h>
#include <array>
#include <filesystem>
#include <mutex>
#include <map>
#include <string>
#include <utility>
#include <vector>

enum class AssemblyKind { Frame, Fixed, LeftWheel, RightWheel, Weapon, GravityFork };

struct MotorSpec {
  bool enabled = false;
  int drive_group = 0;                // Wheel assemblies sharing one motor bank.
  int motor_count = 1;
  double kv_rpm_per_volt = 0.0;
  double peak_current_a = 0.0;
  double phase_resistance_ohm = 0.0;  // Optional terminal-to-terminal resistance.
  double esc_current_limit_a = 0.0;  // Optional per-motor phase-current limit.
  double gear_ratio = 1.0;             // Motor revolutions per output revolution.
  std::string gear_ratio_text = "1:1"; // User-facing input:output form.
  double gearbox_efficiency = 1.0;    // Fraction, e.g. 0.9 for 90%.
};

double MotorNoLoadOutputRpm(const MotorSpec& motor, double battery_voltage);
double MotorOutputTorque(const MotorSpec& motor, double battery_voltage,
                         double desired_output_speed, double actual_output_speed);
double EstimateMotorSpinupSeconds(const MotorSpec& motor, double battery_voltage,
                                  double axis_inertia, double target_fraction = 0.9);

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
  std::array<double, 3> center_of_mass{};
  bool has_fusion_center_of_mass = false;
  // Centroidal inertia in the body's native Fusion frame, packed as
  // Ixx, Iyy, Izz, Ixy, Ixz, Iyz in kg*m^2.
  std::array<double, 6> fusion_inertia_tensor{};
  bool has_fusion_inertia_tensor = false;
  int assembly = 0;  // -1 excludes this body.
  bool preview_visible = true;
  bool collision_enabled = true;
};

struct CadAssembly {
  std::string name;
  AssemblyKind kind = AssemblyKind::Fixed;
  std::array<double, 3> pivot{};
  std::array<double, 3> axis{0, 1, 0};
  // Passive fork hinge limits relative to the imported CAD pose.
  std::array<double, 2> fork_range_degrees{-90, 90};
  bool fork_collides_with_frame = true;
  MotorSpec motor;
};

class FusionBundle {
 public:
  bool Load(const std::filesystem::path& manifest);
  bool Save();
  bool Generate(std::filesystem::path& output);
  bool GeneratePreview(std::filesystem::path& output);
  bool SetForkCount(int count);
  void Draw(SDL_Window* window, bool& load_requested, bool& apply_requested);
  void DrawAxisSelectionControls();
  void SelectPart(int part_index);
  void UpdateHover(int part_index, const std::array<double, 3>& hit);
  bool SelectHoveredHole();
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
  int AssemblyCount() const { return int(assemblies_.size()); }
  int ForkCount() const;
  int PartCount() const { return int(parts_.size()); }
  double EstimatedWeaponSpinupSeconds() const;
  const CadAssembly& Assembly(int index) const { return assemblies_.at(index); }
  void PickEdge(int part_index, const std::array<double, 3>& cad_point);
  int SelectedPart() const { return selected_part_; }
  int HoveredPart() const { return hovered_part_; }
  int PartAssembly(int part_index) const {
    return part_index >= 0 && part_index < int(parts_.size()) ? parts_[part_index].assembly : -2;
  }
  size_t HoveredLoopPoints() const { return hovered_loop_.size(); }
  std::array<double, 3> PreviewCenter() const;
  double PreviewDistance() const;
  void BindControls(const mjModel* model, std::vector<std::pair<int, double>>& left,
                    std::vector<std::pair<int, double>>& right,
                    std::vector<int>& weapon, std::map<int, MotorSpec>& motor_models,
                    std::map<int, MotorSpec>& weapon_motor_models,
                    std::map<int, int>& manual_velocity_actuators,
                    double& battery_voltage) const;
  void PickFace(int part_index, const std::array<double, 3>& cad_point);
  void AddAxisMarkers(const mjModel* model, const mjData* data, mjvScene& scene) const;
  void AddSetupAxisMarkers(mjvScene& scene,
                           std::array<double, 3> camera_position = {}) const;
  bool HasBundle() const { return !parts_.empty(); }
  const std::string& Error() const { return error_; }
  const std::string& PickMessage() const { return pick_message_; }
  double& BatteryVoltage() { return battery_voltage_; }

 private:
  void DrawPart(size_t index);
  void DrawTree(const std::vector<std::string>& parent);
  std::array<double, 3> GroupCenter(int assembly) const;
  std::filesystem::path manifest_;
  std::string design_name_;
  std::string error_;
  std::vector<CadPart> parts_;
  std::vector<CadAssembly> assemblies_;
  double battery_voltage_ = 0.0;
  char path_buffer_[1024]{};
  int selected_part_ = -1;
  int hovered_part_ = -1;
  std::vector<std::array<double, 3>> hovered_loop_;
  std::map<int, std::vector<std::vector<std::array<double, 3>>>> hole_cache_;
  bool axis_picking_ = false;
  int axis_assembly_ = 1;
  std::vector<std::array<double, 3>> axis_points_;
  std::string pick_message_;
  std::string fork_message_;
  std::mutex dialog_mutex_;
  std::string pending_path_;
};
