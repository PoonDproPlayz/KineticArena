#include <mujoco/mujoco.h>
#include <SDL3/SDL.h>
#include <SDL3/SDL_opengl.h>
#include "imgui.h"
#include "backends/imgui_impl_sdl3.h"
#include "backends/imgui_impl_opengl3.h"
#include "FusionBundle.h"
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace {
constexpr double pi = 3.14159265358979323846;
using json = nlohmann::json;
constexpr int max_input_channels = 16;

struct InputChannel {
  bool button = false;
  int source_index = 0;
  bool inverted = false;
  float deadzone = 0.12f;
};

struct InputMixer {
  bool enabled = false;
  int channel_count = 3;
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

void initialize_default_mixer(InputMixer& mixer) {
  mixer = InputMixer{};
  mixer.enabled = false;
  mixer.channel_count = 3;
  mixer.channels[0] = {false, 1, true, 0.12f};  // stick up is negative on common USB pads.
  mixer.channels[1] = {false, 0, false, 0.12f};
  mixer.channels[2] = {true, 0, false, 0.0f};
}

void load_mixer_settings(InputMixer& mixer) {
  initialize_default_mixer(mixer);
  char* preference_path = SDL_GetPrefPath("CombatSim", "CombatSim");
  if (!preference_path) return;
  const std::filesystem::path path = std::filesystem::path(preference_path) / "input_mixer.json";
  SDL_free(preference_path);
  try {
    std::ifstream input(path);
    if (!input) return;
    const json saved = json::parse(input);
    mixer.enabled = saved.value("enabled", mixer.enabled);
    mixer.preferred_guid = saved.value("preferredGuid", std::string{});
    mixer.channel_count = std::clamp(saved.value("channelCount", mixer.channel_count), 1,
                                      max_input_channels);
    if (saved.contains("channels") && saved["channels"].is_array()) {
      const auto& channels = saved["channels"];
      for (int i = 0; i < std::min<int>(max_input_channels, channels.size()); ++i) {
        const auto& item = channels[i];
        if (!item.is_object()) continue;
        mixer.channels[i].button = item.value("button", mixer.channels[i].button);
        mixer.channels[i].source_index = std::max(0, item.value("sourceIndex", mixer.channels[i].source_index));
        mixer.channels[i].inverted = item.value("inverted", mixer.channels[i].inverted);
        mixer.channels[i].deadzone = std::clamp(item.value("deadzone", mixer.channels[i].deadzone), 0.0f, 0.95f);
      }
    }
    if (saved.contains("outputs") && saved["outputs"].is_object()) {
      for (auto it = saved["outputs"].begin(); it != saved["outputs"].end(); ++it) {
        if (!it.value().is_array()) continue;
        std::array<float, max_input_channels> weights{};
        for (int i = 0; i < std::min<int>(max_input_channels, it.value().size()); ++i)
          if (it.value()[i].is_number()) weights[i] = std::clamp(it.value()[i].get<float>(), -1.0f, 1.0f);
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
  } catch (const std::exception&) {
    // A malformed preference file should not prevent the simulator from starting.
  }
}

bool save_mixer_settings(const InputMixer& mixer) {
  char* preference_path = SDL_GetPrefPath("CombatSim", "CombatSim");
  if (!preference_path) return false;
  const std::filesystem::path path = std::filesystem::path(preference_path) / "input_mixer.json";
  SDL_free(preference_path);
  json saved;
  saved["enabled"] = mixer.enabled;
  saved["preferredGuid"] = mixer.preferred_guid;
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

void apply_wheel_drive_targets(const mjModel* model, mjData* data, const Controls& controls,
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
    const double desired_joint_speed = forward_sign * desired_forward_speed;
    const auto motor = controls.motor_models.find(id);
    const auto manual_actuator = controls.manual_velocity_actuators.find(id);
    if (motor == controls.motor_models.end()) {
      data->ctrl[id] = desired_joint_speed;
      return;
    }
    if (controls.manual_wheel_rpm_override &&
        manual_actuator != controls.manual_velocity_actuators.end()) {
      data->ctrl[id] = 0.0;
      data->ctrl[manual_actuator->second] = desired_joint_speed;
      return;
    }
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

void draw_mixer_dialog(InputMixer& mixer, SDL_Joystick*& joystick,
                       SDL_JoystickID& joystick_id, const Controls& controls) {
  bool changed = false;
  const auto devices = joystick_devices();
  const auto outputs = mixer_outputs(controls);
  const char* joystick_name = joystick ? SDL_GetJoystickName(joystick) : nullptr;
  std::string selected_name = joystick_name ? joystick_name : "";
  if (selected_name.empty()) selected_name = mixer.preferred_guid == "none"
    ? "No joystick selected" : "Choose USB joystick";

  ImGui::BeginChild("Controller input setup", ImVec2(376.0f, 0.0f),
                    ImGuiChildFlags_Borders | ImGuiChildFlags_AlwaysUseWindowPadding,
                    ImGuiWindowFlags_AlwaysVerticalScrollbar);
  ImGui::TextColored(ImVec4(0.32f, 0.88f, 0.89f, 1.0f), "INPUT SOURCES");
  if (ImGui::BeginCombo("USB joystick", selected_name.c_str())) {
    if (ImGui::Selectable("No joystick", mixer.preferred_guid == "none")) {
      mixer.preferred_guid = "none";
      changed = true;
      refresh_joystick(joystick, joystick_id, mixer);
    }
    for (const auto& device : devices) {
      const bool selected = mixer.preferred_guid == device.guid ||
        (mixer.preferred_guid.empty() && joystick_id == device.id);
      const std::string label = device.name + "  [" + device.guid.substr(0, 8) + "]";
      if (ImGui::Selectable(label.c_str(), selected)) {
        mixer.preferred_guid = device.guid;
        changed = true;
        refresh_joystick(joystick, joystick_id, mixer);
      }
    }
    ImGui::EndCombo();
  }
  if (joystick) {
    ImGui::Text("Connected: %s  |  %d axes, %d buttons", SDL_GetJoystickName(joystick),
      SDL_GetNumJoystickAxes(joystick), SDL_GetNumJoystickButtons(joystick));
  } else {
    ImGui::TextDisabled("No selected USB joystick is connected.");
  }
  if (ImGui::Checkbox("Enable custom joystick mixer", &mixer.enabled)) changed = true;
  ImGui::TextDisabled("RC default: left = Ch1 - Ch2; right = Ch1 + Ch2; weapon = Ch3.");
  if (mixer.enabled && !joystick)
    ImGui::TextColored(ImVec4(1.0f, 0.70f, 0.34f, 1.0f),
      "Mixer is enabled but no joystick is connected; keyboard and mapped gamepad controls remain available.");

  ImGui::SeparatorText("Input channels");
  if (ImGui::SmallButton("- channel") && mixer.channel_count > 1) {
    --mixer.channel_count; changed = true;
  }
  ImGui::SameLine();
  if (ImGui::SmallButton("+ channel") && mixer.channel_count < max_input_channels) {
    const int new_index = mixer.channel_count++;
    mixer.channels[new_index] = {false, new_index, false, 0.12f};
    changed = true;
  }
  ImGui::SameLine();
  ImGui::TextDisabled("%d / %d channels", mixer.channel_count, max_input_channels);

  const auto live_channels = read_input_channels(joystick, mixer);
  if (ImGui::BeginTable("##joystick_input_channels", 6,
      ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollX |
      ImGuiTableFlags_SizingFixedFit,
      ImVec2(0.0f, 210.0f))) {
    ImGui::TableSetupColumn("Channel", ImGuiTableColumnFlags_WidthFixed, 55.0f);
    ImGui::TableSetupColumn("Source", ImGuiTableColumnFlags_WidthFixed, 86.0f);
    ImGui::TableSetupColumn("Index", ImGuiTableColumnFlags_WidthFixed, 54.0f);
    ImGui::TableSetupColumn("Reverse", ImGuiTableColumnFlags_WidthFixed, 62.0f);
    ImGui::TableSetupColumn("Deadzone", ImGuiTableColumnFlags_WidthFixed, 82.0f);
    ImGui::TableSetupColumn("Live", ImGuiTableColumnFlags_WidthFixed, 75.0f);
    ImGui::TableHeadersRow();
    for (int i = 0; i < mixer.channel_count; ++i) {
      auto& channel = mixer.channels[i];
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0); ImGui::Text("Ch %d", i + 1);
      ImGui::TableSetColumnIndex(1);
      ImGui::PushID(3000 + i);
      const char* source_name = channel.button ? "Button" : "Axis";
      if (ImGui::BeginCombo("##source_type", source_name)) {
        if (ImGui::Selectable("Axis", !channel.button)) { channel.button = false; changed = true; }
        if (ImGui::Selectable("Button", channel.button)) { channel.button = true; changed = true; }
        ImGui::EndCombo();
      }
      ImGui::TableSetColumnIndex(2);
      ImGui::SetNextItemWidth(-1.0f);
      if (ImGui::InputInt("##source_index", &channel.source_index, 1, 1)) changed = true;
      channel.source_index = std::clamp(channel.source_index, 0, channel.button ? 127 : 63);
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
      ImGui::PopID();
    }
    ImGui::EndTable();
  }
  ImGui::EndChild();

  ImGui::SameLine(0.0f, 12.0f);
  ImGui::BeginChild("Controller output mixer", ImVec2(0.0f, 0.0f),
                    ImGuiChildFlags_Borders | ImGuiChildFlags_AlwaysUseWindowPadding,
                    ImGuiWindowFlags_AlwaysVerticalScrollbar);
  ImGui::TextColored(ImVec4(0.32f, 0.88f, 0.89f, 1.0f), "OUTPUT MIXES");
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
  if (ImGui::BeginCombo("Output function", selected_output_name)) {
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
    if (ImGui::SmallButton("Reset selected output")) {
      weights = default_output_weights(output);
      offsets.fill(0.0f);
      expos.fill(0.0f);
      changed = true;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("Each channel is a mix line: input x weight + offset, with optional expo.");
    if (ImGui::BeginTable("##output_mix_lines", 5,
        ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit,
        ImVec2(0.0f, 300.0f))) {
      ImGui::TableSetupColumn("Input", ImGuiTableColumnFlags_WidthFixed, 58.0f);
      ImGui::TableSetupColumn("Live", ImGuiTableColumnFlags_WidthFixed, 70.0f);
      ImGui::TableSetupColumn("Weight", ImGuiTableColumnFlags_WidthFixed, 108.0f);
      ImGui::TableSetupColumn("Offset", ImGuiTableColumnFlags_WidthFixed, 108.0f);
      ImGui::TableSetupColumn("Expo", ImGuiTableColumnFlags_WidthFixed, 108.0f);
      ImGui::TableHeadersRow();
      for (int i = 0; i < mixer.channel_count; ++i) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0); ImGui::Text("Ch %d", i + 1);
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
    const std::string graph_channel_name = "Ch " + std::to_string(graph_channel + 1);
    if (ImGui::BeginCombo("Graph input", graph_channel_name.c_str())) {
      for (int i = 0; i < mixer.channel_count; ++i) {
        const std::string label = "Ch " + std::to_string(i + 1);
        if (ImGui::Selectable(label.c_str(), graph_channel == i)) graph_channel = i;
      }
      ImGui::EndCombo();
    }
    const float live_output = output_value(mixer, output, live_channels);
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(0.32f, 0.88f, 0.89f, 1.0f),
                       "Output %+.0f%%", live_output * 100.0f);
    const ImVec2 graph_size(std::max(260.0f, ImGui::GetContentRegionAvail().x), 220.0f);
    const ImVec2 graph_min = ImGui::GetCursorScreenPos();
    ImGui::InvisibleButton("##mixer_response_graph", graph_size);
    const ImVec2 graph_max(graph_min.x + graph_size.x, graph_min.y + graph_size.y);
    ImDrawList* draw = ImGui::GetWindowDrawList();
    const ImU32 graph_bg = ImGui::GetColorU32(ImGuiCol_FrameBg);
    const ImU32 grid_color = ImGui::GetColorU32(ImGuiCol_Border);
    const ImU32 axis_color = ImGui::GetColorU32(ImGuiCol_TextDisabled);
    const ImU32 curve_color = ImGui::GetColorU32(ImVec4(0.20f, 0.82f, 0.88f, 1.0f));
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
      auto sample = live_channels;
      sample[graph_channel] = x;
      return output_value(mixer, output, sample);
    };
    ImVec2 previous = map_point(input_min, evaluate_graph(input_min));
    constexpr int curve_samples = 64;
    for (int i = 1; i <= curve_samples; ++i) {
      const float x = input_min + (input_max - input_min) * i / curve_samples;
      const ImVec2 next = map_point(x, evaluate_graph(x));
      draw->AddLine(previous, next, curve_color, 2.5f);
      previous = next;
    }
    const ImVec2 dot = map_point(live_channels[graph_channel], live_output);
    draw->AddCircleFilled(dot, 7.0f, ImGui::GetColorU32(ImVec4(1.0f, 0.72f, 0.18f, 1.0f)));
    draw->AddCircle(dot, 9.0f, ImGui::GetColorU32(ImVec4(1.0f, 1.0f, 1.0f, 0.95f)), 16, 1.5f);
  }
  ImGui::TextDisabled("Drive outputs clamp to -100..100%%. Weapon uses 0..100%%. Positive expo softens the center; negative expo sharpens it.");
  ImGui::EndChild();
  if (changed) {
    if (!save_mixer_settings(mixer))
      ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.4f, 1.0f), "Could not save mixer settings.");
  }
}

void drive(const mjModel* model, mjData* data, Controls& c, SDL_Gamepad* pad,
           SDL_Joystick* joystick, const InputMixer& mixer, bool keyboard_enabled) {
  const bool* keys = SDL_GetKeyboardState(nullptr);
  float throttle = keyboard_enabled ? float(keys[SDL_SCANCODE_W]) - float(keys[SDL_SCANCODE_S]) : 0.0f;
  float turn = keyboard_enabled ? float(keys[SDL_SCANCODE_A]) - float(keys[SDL_SCANCODE_D]) : 0.0f;
  const bool custom_input = mixer.enabled && joystick;
  std::array<float, max_input_channels> channels{};
  float weapon_throttle = 0.0f;
  if (custom_input) {
    channels = read_input_channels(joystick, mixer);
  } else {
    const float pad_throttle = -axis_with_deadzone(pad, SDL_GAMEPAD_AXIS_LEFTY, c.deadzone);
    const float pad_turn = -axis_with_deadzone(pad, SDL_GAMEPAD_AXIS_LEFTX, c.deadzone);
    if (pad_throttle != 0.0f) throttle = pad_throttle;
    if (pad_turn != 0.0f) turn = pad_turn;
    weapon_throttle = ((keyboard_enabled && keys[SDL_SCANCODE_SPACE]) ||
      (pad && (SDL_GetGamepadButton(pad, SDL_GAMEPAD_BUTTON_LEFT_SHOULDER) ||
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

int main(int argc, char** argv) {
  FusionBundle bundle;
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
    if (std::abs(left - 0.35f) > 1e-5f || std::abs(right - 0.85f) > 1e-5f ||
        std::abs(weapon - 1.0f) > 1e-5f ||
        std::abs(shaped_line - 0.508f) > 1e-5f ||
        std::abs(capped - 1.0f) > 1e-5f || std::abs(weapon_off) > 1e-6f ||
        !motor_a_output || !motor_b_output || !weapon_output ||
        std::abs(output_value(defaults, *motor_a_output, input) - 0.35f) > 1e-5f ||
        std::abs(output_value(defaults, *motor_b_output, input) - 0.85f) > 1e-5f ||
        std::abs(output_value(defaults, *weapon_output, input) - 1.0f) > 1e-5f ||
        std::abs(normalize_axis(0, 0.12f)) > 1e-6f ||
        std::abs(normalize_axis(32767, 0.12f) - 1.0f) > 1e-6f ||
        std::abs(normalize_axis(-32768, 0.12f) + 1.0f) > 1e-6f) {
      std::fprintf(stderr, "Input mixer failed: elevon sums, weight/offset/expo, weapon range, or axis calibration are incorrect.\n");
      return 1;
    }
    std::puts("Input mixer passed: motor outputs, elevon sums, weight/offset/expo, weapon channel, and axis normalization.");
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
    std::puts("Drive mixing passed: zero-throttle turns counter-rotate both sides; W/S drive both sides together.");
    return 0;
  }
  if (argc >= 2 && std::string(argv[1]) == "--test-straight-drive") {
    const bool arena_mode = argc >= 3 && std::string(argv[2]) == "--arena";
    const int model_arg = arena_mode ? 3 : 2;
    const std::filesystem::path test_model = argc > model_arg ? argv[model_arg] :
      std::filesystem::path("Assets/CombatSim/Models/CombatRobot500g.xml");
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
    }
    const double start_x = checked_data->qpos[root_qpos];
    const double start_y = checked_data->qpos[root_qpos + 1];
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
    }
    const double yaw_change = std::remainder(yaw_at(checked_data) - start_yaw, 2.0 * pi);
    std::printf("CAD straight run (floor and bot only, 1.0 s): dx=%.4f m dy=%.4f m yaw=%.5f rad (%.3f deg)\n",
      checked_data->qpos[root_qpos] - start_x, checked_data->qpos[root_qpos + 1] - start_y,
      yaw_change, yaw_change * 180.0 / pi);
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
    const int floor_geom = mj_name2id(checked, mjOBJ_GEOM, "floor");
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
      std::filesystem::path("Assets/CombatSim/Models/CombatRobot500g.xml");
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
    if (std::abs(ideal_stall - expected_stall) > 1e-8 ||
        std::abs(resistive_stall - expected_stall) > 1e-8 ||
        std::abs(2.0 * shared_wheel_torque - expected_stall) > 1e-8 ||
        std::abs(estimated_no_load_rpm - 1110.0) > 1e-8 ||
        std::abs(ideal_free) > 1e-8 || std::abs(resistive_free) > 1e-8) {
      std::fprintf(stderr, "Motor torque model failed its stall/free-speed checks.\n");
      return 1;
    }
    std::printf("Motor torque and RPM estimate passed: %.4f Nm total stall torque; %.0f output RPM no-load; zero torque at no-load speed.\n",
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
    mj_forward(checked, checked_data);
    const int actuator_id = motor_it->first;
    const int joint_id = checked->actuator_trnid[2 * actuator_id];
    const MotorSpec& motor = motor_it->second;
    const double free_output_speed = motor.kv_rpm_per_volt * voltage *
      2.0 * pi / 60.0 / motor.gear_ratio;
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
      std::filesystem::path("Assets/CombatSim/Models/CombatRobot500g.xml");
    auto run_wall_case = [&](double timestep, double impact_speed) {
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
      checked_data->qvel[player_dof] = impact_speed;
      mj_forward(checked, checked_data);
      double peak_x = checked_data->qpos[player_qpos];
      const int steps = int(std::ceil(0.12 / timestep));
      for (int i = 0; i < steps; ++i) {
        mj_step(checked, checked_data);
        peak_x = std::max(peak_x, checked_data->qpos[player_qpos]);
      }
      const double end_x = checked_data->qpos[player_qpos];
      std::printf("Wall impact at %.0f Hz and %.1f m/s: peak center X=%.4f m, final X=%.4f m\n",
        1.0 / timestep, impact_speed, peak_x, end_x);
      mj_deleteData(checked_data); mj_deleteModel(checked);
      return peak_x;
    };
    const double at_2500 = run_wall_case(0.0004, 24.0);
    const double at_10000 = run_wall_case(0.0001, 24.0);
    const double at_2500_fast = run_wall_case(0.0004, 100.0);
    const double at_10000_fast = run_wall_case(0.0001, 100.0);
    if (at_2500 > 1.25 || at_10000 > 1.25 ||
        at_2500_fast > 0.84 || at_10000_fast > 0.84) {
      std::fprintf(stderr, "Robot exceeded the acceptable wall-impact penetration envelope.\n");
      return 1;
    }
    std::puts("Wall impact stayed inside the arena at both update rates.");
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
    if (bundle.HoveredPart() != -1) {
      std::fprintf(stderr, "Axis picking accepted a body outside the target assembly.\n");
      return 1;
    }
    bundle.UpdateHover(0, {0.010, 0.0, 0.004});
    if (bundle.HoveredPart() != 0 || !bundle.SelectHoveredHole()) {
      std::fprintf(stderr, "Synthetic hole hover/selection failed: %s\n", bundle.PickMessage().c_str());
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
        reloaded.PartAssembly(0) != 1 ||
        std::abs(reloaded.Assembly(1).pivot[2] - axis.pivot[2]) > 0.0005) {
      std::fprintf(stderr, "Saved folder assignment, axis target, or fitted axis did not persist.\n");
      return 1;
    }
    std::printf("Synthetic hole rim recognized and fitted: center=(%.4f, %.4f, %.4f) m\n",
                axis.pivot[0], axis.pivot[1], axis.pivot[2]);
    return 0;
  }
  if (argc >= 4 && std::string(argv[1]) == "--test-axis-hover") {
    if (!bundle.Load(argv[3])) { std::fprintf(stderr, "%s\n", bundle.Error().c_str()); return 1; }
    const int target = bundle.AxisAssembly();
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
    bool fit = false;
    bundle.StartAxis(target);
    for (int elevation : elevations) {
      for (int azimuth : azimuths) {
        checked_camera.azimuth = azimuth;
        checked_camera.elevation = elevation;
        mjv_updateScene(checked, checked_data, &checked_options, nullptr, &checked_camera,
                        mjCAT_ALL, &checked_scene);
        for (int y = 2; y < 62 && !fit; ++y) for (int x = 2; x < 62; ++x) {
          mjtNum point[3]; int geom = -1, flex = -1, skin = -1;
          mjv_select(checked, checked_data, &checked_options, 1.3,
                     x/64.0, y/64.0, &checked_scene, point, &geom, &flex, &skin);
          ++rays;
          const char* name = geom >= 0 ? mj_id2name(checked, mjOBJ_GEOM, geom) : nullptr;
          if (!name || std::string(name).rfind("cad_part_", 0) != 0) continue;
          const int part = std::stoi(std::string(name).substr(9));
          if (bundle.PartAssembly(part) != target) continue;
          bundle.UpdateHover(part, {point[0], point[1], point[2]});
          if (bundle.HoveredLoopPoints() >= 6) {
            found_part = part;
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
      std::printf("Real bundle rim hit part %d after %d rays; click-to-axis fit %s; center %.5f %.5f %.5f m\n",
                  found_part, rays, fit ? "passed" : "failed", axis.pivot[0], axis.pivot[1], axis.pivot[2]);
    } else {
      std::fprintf(stderr, "No complete target-assembly rim hit in %d MuJoCo preview rays. Last status: %s\n",
                   rays, bundle.PickMessage().c_str());
    }
    mjv_freeScene(&checked_scene); mj_deleteData(checked_data); mj_deleteModel(checked);
    return fit ? 0 : 1;
  }
  if (argc >= 3 && std::string(argv[1]) == "--validate-bundle") {
    if (!bundle.Load(argv[2])) { std::fprintf(stderr, "%s\n", bundle.Error().c_str()); return 1; }
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
    std::printf("CAD gravity forks: %d; explicit fork-frame pairs: %d; initial fork-frame contacts: %d; max passive fork angle after 1.0 s: %.5f rad\n",
                int(fork_bodies.size()), checked->npair, initial_fork_frame_contacts,
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
      mj_deleteModel(checked);
      std::fprintf(stderr, "Axis test requires at least one body assigned to rotating assembly 1.\n");
      return 1;
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
  const std::filesystem::path model_path = argc > 1 ? argv[1] :
    std::filesystem::path("Assets/CombatSim/Models/CombatRobot500g.xml");
  std::array<char, 4096> load_error{};
  mjModel* model = mj_loadXML(model_path.string().c_str(), nullptr, load_error.data(), load_error.size());
  if (!model) {
    std::fprintf(stderr, "MuJoCo model load failed: %s\n", load_error.data());
    return 1;
  }
  Controls controls;
  bind_stock_actuators(model, controls);
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
  load_mixer_settings(input_mixer);
  SDL_Joystick* joystick = nullptr;
  SDL_JoystickID joystick_id = 0;
  const std::string saved_guid = input_mixer.preferred_guid;
  refresh_joystick(joystick, joystick_id, input_mixer);
  if (saved_guid != input_mixer.preferred_guid) save_mixer_settings(input_mixer);

  mjvCamera camera; mjv_defaultCamera(&camera);
  camera.distance = 3.5; camera.elevation = -55; camera.azimuth = 90;
  mjvCamera setup_camera; mjv_defaultCamera(&setup_camera);
  setup_camera.distance = 1.0; setup_camera.elevation = -45; setup_camera.azimuth = 45;
  mjvOption options; mjv_defaultOption(&options);
  mjvScene scene; mjv_defaultScene(&scene); mjv_makeScene(model, &scene, 2000);
  mjModel* setup_model = nullptr;
  mjData* setup_data = nullptr;
  mjvScene setup_scene; mjv_defaultScene(&setup_scene);
  mjrContext setup_context; mjr_defaultContext(&setup_context);
  model->vis.quality.offsamples = 0;
  mjrContext context; mjr_defaultContext(&context); mjr_makeContext(model, &context, mjFONTSCALE_100);
  GLuint preview_texture = 0;
  glGenTextures(1, &preview_texture);
  int texture_width = 0, texture_height = 0;
  std::vector<unsigned char> preview_pixels;
  IMGUI_CHECKVERSION(); ImGui::CreateContext();
  ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  ImGuiStyle& ui = ImGui::GetStyle();
  ui.WindowRounding = 10.0f; ui.ChildRounding = 8.0f; ui.FrameRounding = 6.0f;
  ui.PopupRounding = 6.0f; ui.ScrollbarRounding = 8.0f; ui.GrabRounding = 6.0f;
  ui.WindowPadding = ImVec2(18, 16); ui.FramePadding = ImVec2(10, 7);
  ui.ItemSpacing = ImVec2(10, 9); ui.WindowBorderSize = 0.0f;
  ImVec4* colors = ui.Colors;
  colors[ImGuiCol_WindowBg] = ImVec4(0.055f, 0.075f, 0.105f, 0.96f);
  colors[ImGuiCol_ChildBg] = ImVec4(0.075f, 0.098f, 0.132f, 1.0f);
  colors[ImGuiCol_PopupBg] = ImVec4(0.075f, 0.098f, 0.132f, 1.0f);
  colors[ImGuiCol_FrameBg] = ImVec4(0.12f, 0.15f, 0.20f, 1.0f);
  colors[ImGuiCol_FrameBgHovered] = ImVec4(0.17f, 0.22f, 0.29f, 1.0f);
  colors[ImGuiCol_FrameBgActive] = ImVec4(0.20f, 0.27f, 0.35f, 1.0f);
  colors[ImGuiCol_Button] = ImVec4(0.12f, 0.20f, 0.27f, 1.0f);
  colors[ImGuiCol_ButtonHovered] = ImVec4(0.16f, 0.38f, 0.47f, 1.0f);
  colors[ImGuiCol_ButtonActive] = ImVec4(0.10f, 0.54f, 0.62f, 1.0f);
  colors[ImGuiCol_Header] = ImVec4(0.11f, 0.23f, 0.30f, 1.0f);
  colors[ImGuiCol_HeaderHovered] = ImVec4(0.14f, 0.34f, 0.42f, 1.0f);
  colors[ImGuiCol_HeaderActive] = ImVec4(0.10f, 0.43f, 0.51f, 1.0f);
  colors[ImGuiCol_CheckMark] = ImVec4(0.25f, 0.83f, 0.83f, 1.0f);
  colors[ImGuiCol_SliderGrab] = ImVec4(0.20f, 0.69f, 0.75f, 1.0f);
  colors[ImGuiCol_SliderGrabActive] = ImVec4(0.32f, 0.86f, 0.87f, 1.0f);
  ImGui_ImplSDL3_InitForOpenGL(window, gl);
  ImGui_ImplOpenGL3_Init("#version 330");

  enum class AppScreen { Home, Drive, Configure };
  AppScreen screen = AppScreen::Home;
  bool running = true, paused = true;
  bool mixer_popup_open = false;
  std::string runtime_error;
  double accumulator = 0.0;
  auto last = std::chrono::steady_clock::now();
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
        if (previous_guid != input_mixer.preferred_guid) save_mixer_settings(input_mixer);
      }

      if (event.type == SDL_EVENT_MOUSE_WHEEL && !ImGui::GetIO().WantCaptureMouse)
        camera.distance = std::clamp(camera.distance * std::pow(0.9, event.wheel.y), 0.5, 15.0);
      if (event.type == SDL_EVENT_MOUSE_MOTION && !ImGui::GetIO().WantCaptureMouse &&
          (event.motion.state & SDL_BUTTON_LMASK)) {
        camera.azimuth += event.motion.xrel * 0.4;
        camera.elevation = std::clamp(camera.elevation - event.motion.yrel * 0.4, -89.0, 89.0);
      }

    }
    const auto now = std::chrono::steady_clock::now();
    accumulator += std::min(0.1, std::chrono::duration<double>(now - last).count());
    last = now;
    if (paused || screen != AppScreen::Drive) accumulator = 0.0;
    int steps = 0;
    while (!paused && screen == AppScreen::Drive && accumulator >= model->opt.timestep && steps++ < 250) {
      drive(model, data, controls, pad, joystick, input_mixer,
            !ImGui::GetIO().WantCaptureKeyboard);
      mj_step(model, data);
      accumulator -= model->opt.timestep;
    }
    if (steps >= 250) accumulator = 0.0;

    int width = 0, height = 0;
    SDL_GetWindowSizeInPixels(window, &width, &height);
    mjrRect viewport{0, 0, width, height};
    mjr_setBuffer(mjFB_WINDOW, &context);
    mjv_updateScene(model, data, &options, nullptr, &camera, mjCAT_ALL, &scene);
    bundle.AddAxisMarkers(model, data, scene);
    mjr_render(viewport, &scene, &context);
    ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplSDL3_NewFrame(); ImGui::NewFrame();
    bool load_requested = false, apply_requested = false, mixer_popup_requested = false;
    const ImVec2 display = ImGui::GetIO().DisplaySize;
    constexpr ImGuiWindowFlags fixed_window = ImGuiWindowFlags_NoTitleBar |
      ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
      ImGuiWindowFlags_NoDocking;

    ImGui::SetNextWindowPos(ImVec2(0, 0));
    ImGui::SetNextWindowSize(ImVec2(display.x, 62));
    ImGui::Begin("##main_navigation", nullptr,
      fixed_window | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::TextColored(ImVec4(0.32f, 0.88f, 0.89f, 1.0f), "COMBATSIM");
    ImGui::SameLine(152);
    auto nav_button = [&](const char* label, AppScreen target) {
      const bool active = screen == target;
      if (active) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.10f, 0.40f, 0.47f, 1.0f));
      const bool clicked = ImGui::Button(label, ImVec2(122, 36));
      if (active) ImGui::PopStyleColor();
      if (clicked) {
        screen = target;
        paused = target != AppScreen::Drive;
      }
    };
    nav_button("Home", AppScreen::Home);
    ImGui::SameLine(); nav_button("Drive", AppScreen::Drive);
    ImGui::SameLine(); nav_button("Bot workshop", AppScreen::Configure);
    ImGui::SameLine(display.x - 245);
    ImGui::TextColored((joystick || pad) ? ImVec4(0.35f, 0.84f, 0.58f, 1.0f) :
                             ImVec4(0.68f, 0.72f, 0.78f, 1.0f),
                       "%s", joystick ? "USB JOYSTICK CONNECTED" :
                         pad ? "GAMEPAD CONNECTED" : "KEYBOARD READY");
    ImGui::End();

    if (screen == AppScreen::Home) {
      ImGui::SetNextWindowPos(ImVec2(32, 92), ImGuiCond_Always);
      ImGui::SetNextWindowSize(ImVec2(std::min(520.0f, display.x - 64.0f),
                                      std::min(620.0f, display.y - 118.0f)), ImGuiCond_Always);
      ImGui::Begin("##home_screen", nullptr, fixed_window | ImGuiWindowFlags_NoScrollbar);
      ImGui::TextColored(ImVec4(0.32f, 0.88f, 0.89f, 1.0f), "ROBOT COMBAT PRACTICE");
      ImGui::Spacing();
      ImGui::Text("Build. Tune. Drive.");
      ImGui::TextWrapped("Set up your robot from Fusion 360, then test its handling and weapon in the MuJoCo arena.");
      ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
      ImGui::Text("QUICK START");
      ImGui::Spacing();
      if (ImGui::Button("Practice drive", ImVec2(-1, 58))) {
        screen = AppScreen::Drive; paused = false;
      }
      ImGui::TextDisabled("W/S drive    A/D turn    Space spin weapon");
      ImGui::Spacing();
      if (ImGui::Button("Configure bot", ImVec2(-1, 58))) {
        screen = AppScreen::Configure; paused = true;
      }
      ImGui::TextDisabled("Import a Fusion bundle, assign assemblies, set axes and forks.");
      ImGui::Spacing();
      if (ImGui::Button("Controller mixing", ImVec2(-1, 58))) {
        mixer_popup_requested = true;
      }
      ImGui::TextDisabled("Assign USB channels, weights, offsets and response curves.");
      ImGui::Spacing(); ImGui::Separator(); ImGui::Spacing();
      ImGui::Text("CURRENT SESSION");
      ImGui::Text("Physics: MuJoCo  |  %.0f Hz", 1.0 / model->opt.timestep);
      ImGui::Text("Model: %s", model_path.filename().string().c_str());
      ImGui::Text("CAD bundle: %s", bundle.HasBundle() ? "loaded" : "not loaded");
      if (bundle.HasBundle()) ImGui::Text("Imported bodies: %d", bundle.PartCount());
      if (!runtime_error.empty()) ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.4f, 1.0f),
                                                       "Setup message: %s", runtime_error.c_str());
      ImGui::Spacing();
      if (ImGui::Button("Quit", ImVec2(104, 38))) running = false;
      ImGui::End();
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
      ImGui::Begin("##drive_dashboard", nullptr, fixed_window);
      const float drive_content_height = ImGui::GetContentRegionAvail().y;
      ImGui::BeginChild("Drive sidebar contents", ImVec2(-8.0f, 0.0f),
                        ImGuiChildFlags_None, ImGuiWindowFlags_AlwaysVerticalScrollbar);
      ImGui::PushTextWrapPos(520.0f);
      ImGui::TextColored(ImVec4(0.32f, 0.88f, 0.89f, 1.0f), "PRACTICE DRIVE");
      ImGui::TextDisabled("MuJoCo simulation  |  %.0f Hz", 1.0 / model->opt.timestep);
      ImGui::Spacing();
      if (ImGui::Button(paused ? "Resume simulation" : "Pause simulation", ImVec2(166, 40)))
        paused = !paused;
      ImGui::SameLine();
      if (ImGui::Button("Reset robot", ImVec2(132, 40))) {
        mj_resetData(model, data); mj_forward(model, data); controls.weapon_command = 0.0;
      }
      if (ImGui::Button("Controller mixer", ImVec2(280.0f, 38.0f))) {
        mixer_popup_requested = true;
      }
      ImGui::Text("Simulation time  %.2f s", data->time);
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
        const auto left_estimate = side_motor_speed_estimate(controls.left,
          controls.motor_models, controls.battery_voltage);
        const auto right_estimate = side_motor_speed_estimate(controls.right,
          controls.motor_models, controls.battery_voltage);
        ImGui::Text("Motor no-load estimate  L %.0f | R %.0f RPM",
                    left_estimate.rpm, right_estimate.rpm);
        if (left_estimate.modeled_wheels == 0 || right_estimate.modeled_wheels == 0)
          ImGui::TextWrapped("A side without motor data uses the 424 RPM fallback. Configure its motor in Bot workshop for a motor-based estimate.");
        else
          ImGui::TextWrapped("Full throttle targets the Kv x battery voltage / gear ratio estimate. Loaded wheel RPM is calculated from motor torque and the bot's contact and inertia.");
      }
      const bool weapon_torque_enabled = weapon_uses_torque_model(controls);
      const double weapon_rpm_limit = weapon_torque_enabled
        ? weapon_no_load_rpm(controls) : controls.weapon_rpm;
      if (weapon_torque_enabled) {
        ImGui::TextWrapped("Weapon motor no-load estimate: %.0f RPM", weapon_rpm_limit);
        const double weapon_spinup_seconds = bundle.EstimatedWeaponSpinupSeconds();
        if (weapon_spinup_seconds > 0.0 && std::isfinite(weapon_spinup_seconds))
          ImGui::TextWrapped("Estimated 0-to-90%% no-load spin-up: %.2f s", weapon_spinup_seconds);
        else
          ImGui::TextWrapped("Estimated weapon spin-up is unavailable until its motor data and inertia are valid.");
        ImGui::TextWrapped("Estimate uses the motor torque curve and modeled weapon inertia; battery sag, impacts, and thermal derating are not included.");
        if (input_mixer.enabled && joystick)
          ImGui::TextWrapped("The controller mixer's Weapon output applies motor torque. Zero output lets the weapon coast down.");
        else
          ImGui::TextWrapped("Hold Space or the gamepad weapon button to apply motor torque. Spin-up comes from the configured motor and weapon assembly inertia; releasing the button lets it coast down.");
      } else {
        ImGui::SetNextItemWidth(280.0f);
        ImGui::SliderFloat("Weapon target RPM", &controls.weapon_rpm, 0, 30000, "%.0f RPM");
        ImGui::SetNextItemWidth(280.0f);
        ImGui::SliderFloat("Weapon spin-up", &controls.spinup_seconds, 0.1f, 5.0f, "%.1f s");
      }
      ImGui::SeparatorText("Live telemetry");
      ImGui::Text("Weapon actual");
      double weapon_actual = 0.0;
      int weapon_readings = 0;
      for (int id : controls.weapon) {
        const double rpm = actuator_rpm(model, data, id);
        if (std::isfinite(rpm)) { weapon_actual += std::abs(rpm); ++weapon_readings; }
      }
      if (weapon_readings) weapon_actual /= weapon_readings;
      ImGui::SameLine();
      if (weapon_readings) ImGui::TextColored(ImVec4(0.35f, 0.88f, 0.83f, 1.0f),
                                               "%.0f RPM", weapon_actual);
      else ImGui::TextDisabled("No weapon drive assigned");
      ImGui::ProgressBar(weapon_rpm_limit > 0.0 ?
        float(std::clamp(weapon_actual / weapon_rpm_limit, 0.0, 1.0)) : 0.0f,
        ImVec2(-1, 12), "");
      auto draw_wheel_telemetry = [&](const char* side,
                                      const std::vector<std::pair<int, double>>& wheels) {
        ImGui::Text("%s wheels  (%d)", side, int(wheels.size()));
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
            ImGui::TextColored(ImVec4(0.80f, 0.75f, 0.48f, 1.0f), "AIRBORNE");
          } else if (slip.speed > 0.18) {
            ImGui::TextColored(ImVec4(1.0f, 0.40f, 0.28f, 1.0f),
                               "SLIPPING  %.2f m/s", slip.speed);
          } else {
            ImGui::TextColored(ImVec4(0.36f, 0.84f, 0.58f, 1.0f),
                               "GRIP  %.2f m/s", slip.speed);
          }
        }
        if (valid) ImGui::TextColored(ImVec4(0.75f, 0.82f, 0.9f, 1.0f),
                                      "  Average: %+.0f RPM", sum / valid);
      };
      draw_wheel_telemetry("Left", controls.left);
      draw_wheel_telemetry("Right", controls.right);
      ImGui::TextWrapped("Wheel slip compares tread motion with the floor at each contact patch. Slip is flagged above 0.18 m/s.");
      ImGui::SeparatorText("Controls");
      ImGui::TextWrapped("W / S drive forward and reverse. A / D steer; with no throttle, both sides counter-rotate for a spot turn. Hold Space to spin the weapon.");
      ImGui::TextWrapped("Open Controller mixer to map USB joystick channels and tune motor/weapon mixes.");
      ImGui::Text("Drive joints: left %d  |  right %d",
                  int(controls.left.size()), int(controls.right.size()));
      if (controls.left.empty() || controls.right.empty())
        ImGui::TextWrapped("Assign at least one wheel assembly to each side in Bot workshop, then Apply and preview.");
      if (!runtime_error.empty()) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.55f, 0.4f, 1.0f));
        ImGui::TextWrapped("Model message: %s", runtime_error.c_str());
        ImGui::PopStyleColor();
      }
      ImGui::PopTextWrapPos();
      ImGui::EndChild();
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
    } else {
      ImGui::SetNextWindowPos(ImVec2(12, 72), ImGuiCond_Always);
      ImGui::SetNextWindowSize(ImVec2(display.x - 24, display.y - 84), ImGuiCond_Always);
      ImGui::Begin("##bot_workshop", nullptr, fixed_window);
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
      ImGui::BeginChild("Bot configuration", ImVec2(editor_width, 0),
                        ImGuiChildFlags_Borders, ImGuiWindowFlags_AlwaysVerticalScrollbar);
      ImGui::PushTextWrapPos(520.0f);
      ImGui::TextColored(ImVec4(0.32f, 0.88f, 0.89f, 1.0f), "BOT CONFIGURATION");
      ImGui::TextDisabled("Import and assign your robot components");
      ImGui::Separator();
      bundle.Draw(window, load_requested, apply_requested);
      if (!runtime_error.empty()) ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.4f, 1.0f),
                                                       "Model message: %s", runtime_error.c_str());
      ImGui::PopTextWrapPos();
      ImGui::EndChild();
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
      ImGui::BeginChild("Robot preview panel", ImVec2(0, 0), ImGuiChildFlags_Borders);
      ImGui::TextColored(ImVec4(0.32f, 0.88f, 0.89f, 1.0f), "ROBOT PREVIEW");
      bundle.DrawAxisSelectionControls();
      ImGui::TextWrapped("Hover to highlight a part. In axis mode, hover a hole rim until its edge loop highlights, then click. Left drag orbits; right drag pans; scroll zooms.");
      if (setup_model) {
        if (ImGui::SmallButton("Reset view")) {
          const auto center = bundle.PreviewCenter();
          for (int i = 0; i < 3; ++i) setup_camera.lookat[i] = center[i];
          setup_camera.distance = bundle.PreviewDistance();
          setup_camera.azimuth = 45; setup_camera.elevation = -45;
        }
        ImGui::SameLine();
        const double min_tilt = -89, max_tilt = 89;
        ImGui::SetNextItemWidth(180);
        ImGui::SliderScalar("Tilt", ImGuiDataType_Double, &setup_camera.elevation,
                            &min_tilt, &max_tilt, "%.0f deg");
      }
      if (setup_model) {
        const ImVec2 available = ImGui::GetContentRegionAvail();
        const int preview_width = std::max(1, int(available.x));
        const int preview_height = std::max(1, int(available.y));
        if (preview_width != setup_context.offWidth || preview_height != setup_context.offHeight)
          mjr_resizeOffscreen(preview_width, preview_height, &setup_context);
        if (preview_width != texture_width || preview_height != texture_height) {
          texture_width = preview_width; texture_height = preview_height;
          preview_pixels.resize(size_t(preview_width) * preview_height * 3);
          glBindTexture(GL_TEXTURE_2D, preview_texture);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
          glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, preview_width, preview_height, 0,
                       GL_RGB, GL_UNSIGNED_BYTE, nullptr);
        }
        for (int i = 0; i < setup_model->ngeom; ++i)
          setup_model->geom_group[i] = bundle.PreviewVisible(i) ? 0 : 5;
        options.geomgroup[5] = 0;
        mjv_updateScene(setup_model, setup_data, &options, nullptr, &setup_camera, mjCAT_ALL, &setup_scene);
        const ImVec2 canvas_min = ImGui::GetCursorScreenPos();
        const ImVec2 mouse = ImGui::GetIO().MousePos;
        const bool mouse_on_canvas = ImGui::IsWindowHovered() &&
          mouse.x >= canvas_min.x && mouse.y >= canvas_min.y &&
          mouse.x < canvas_min.x + preview_width && mouse.y < canvas_min.y + preview_height;
        if (mouse_on_canvas && !ImGui::IsMouseDragging(ImGuiMouseButton_Left) &&
            !ImGui::IsMouseDragging(ImGuiMouseButton_Right)) {
          mjtNum hit[3]; int geom_id = -1, flex_id = -1, skin_id = -1;
          mjv_select(setup_model, setup_data, &options, double(preview_width)/preview_height,
            (mouse.x-canvas_min.x)/preview_width, 1.0-(mouse.y-canvas_min.y)/preview_height,
            &setup_scene, hit, &geom_id, &flex_id, &skin_id);
          const char* picked = geom_id >= 0 ? mj_id2name(setup_model, mjOBJ_GEOM, geom_id) : nullptr;
          const int part = picked && std::string(picked).rfind("cad_part_", 0) == 0 ?
            std::stoi(std::string(picked).substr(9)) : -1;
          bundle.UpdateHover(part, {hit[0], hit[1], hit[2]});
        } else bundle.UpdateHover(-1, {});
        if (!bundle.AxisPicking()) {
          const int selected = bundle.SelectedPart();
          for (int highlight : {bundle.HoveredPart(), selected}) if (highlight >= 0) {
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
        bundle.AddSetupAxisMarkers(setup_scene,
          {camera_head[0], camera_head[1], camera_head[2]});
        mjr_setBuffer(mjFB_OFFSCREEN, &setup_context);
        mjr_render({0, 0, preview_width, preview_height}, &setup_scene, &setup_context);
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        mjr_readPixels(preview_pixels.data(), nullptr,
                       {0, 0, preview_width, preview_height}, &setup_context);
        mjr_setBuffer(mjFB_WINDOW, &setup_context);
        glBindTexture(GL_TEXTURE_2D, preview_texture);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, preview_width, preview_height,
                        GL_RGB, GL_UNSIGNED_BYTE, preview_pixels.data());
        ImGui::InvisibleButton("CAD canvas", ImVec2(float(preview_width), float(preview_height)),
                               ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
        const ImVec2 image_min = ImGui::GetItemRectMin();
        const ImVec2 image_max = ImGui::GetItemRectMax();
        ImGui::GetWindowDrawList()->AddImage((ImTextureID)(intptr_t)preview_texture,
                                             image_min, image_max, ImVec2(0, 1), ImVec2(1, 0));
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
          if (ImGui::IsMouseReleased(ImGuiMouseButton_Left) &&
              ImGui::GetIO().MouseDragMaxDistanceSqr[ImGuiMouseButton_Left] <= 16.0f) {
            if (bundle.AxisPicking()) bundle.SelectHoveredHole();
            else if (bundle.HoveredPart() >= 0) bundle.SelectPart(bundle.HoveredPart());
          }
        }
      } else ImGui::TextWrapped("Load a Fusion manifest in Bot configuration to preview and select its parts.");
      ImGui::EndChild();
      ImGui::End();
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
      if (ImGui::BeginPopupModal("Controller mixer setup", &mixer_popup_open,
          ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse)) {
        ImGui::TextColored(ImVec4(0.32f, 0.88f, 0.89f, 1.0f), "CONTROLLER MIXER");
        ImGui::TextDisabled("Set input channels, tune each output mix line, then move the controller to test its response graph.");
        ImGui::Separator();
        draw_mixer_dialog(input_mixer, joystick, joystick_id, controls);
        ImGui::Separator();
        ImGui::TextDisabled("Weights, offsets and expo save automatically and carry over when you load another bot.");
        ImGui::SameLine(ImGui::GetWindowWidth() - 118.0f);
        if (ImGui::Button("Close", ImVec2(96.0f, 34.0f))) {
          mixer_popup_open = false;
          ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
      }
    }
    if (load_requested) {
      std::filesystem::path preview_path;
      if (bundle.GeneratePreview(preview_path)) {
        std::array<char, 8192> diagnostic{};
        mjModel* next = mj_loadXML(preview_path.string().c_str(), nullptr, diagnostic.data(), diagnostic.size());
        mjData* next_data = next ? mj_makeData(next) : nullptr;
        if (next_data) {
          if (setup_model) {
            mjr_freeContext(&setup_context); mjv_freeScene(&setup_scene);
            mj_deleteData(setup_data); mj_deleteModel(setup_model);
          }
          setup_model = next; setup_data = next_data;
          mj_forward(setup_model, setup_data);
          setup_model->vis.quality.offsamples = 0;
          mjv_makeScene(setup_model, &setup_scene, 2000);
          mjr_makeContext(setup_model, &setup_context, mjFONTSCALE_100);
          const auto center = bundle.PreviewCenter();
          for (int i = 0; i < 3; ++i) setup_camera.lookat[i] = center[i];
          setup_camera.distance = bundle.PreviewDistance();
          runtime_error.clear();
        } else {
          runtime_error = next ? "Could not allocate CAD setup model." : diagnostic.data();
          if (next) mj_deleteModel(next);
        }
      } else runtime_error = bundle.Error();
    }
    if (apply_requested) {
      std::filesystem::path generated;
      if (bundle.Generate(generated)) {
        std::array<char, 8192> preview_error{};
        mjModel* next_model = mj_loadXML(generated.string().c_str(), nullptr, preview_error.data(), preview_error.size());
        mjData* next_data = next_model ? mj_makeData(next_model) : nullptr;
        if (!next_data) {
          runtime_error = next_model ? "MuJoCo could not allocate model data." : preview_error.data();
          if (next_model) mj_deleteModel(next_model);
        } else {
          mj_forward(next_model, next_data);
          mjr_freeContext(&context); mjv_freeScene(&scene);
          mj_deleteData(data); mj_deleteModel(model);
          model = next_model; data = next_data;
          model->vis.quality.offsamples = 0;
          mjv_makeScene(model, &scene, 2000); mjr_makeContext(model, &context, mjFONTSCALE_100);
          controls.left.clear(); controls.right.clear(); controls.weapon.clear();
          bundle.BindControls(model, controls.left, controls.right, controls.weapon,
                              controls.motor_models, controls.weapon_motor_models,
                              controls.manual_velocity_actuators,
                              controls.battery_voltage);
          controls.weapon_sign = 1.0; controls.weapon_command = 0.0;
          accumulator = 0.0; paused = true; runtime_error.clear();
        }
      } else runtime_error = bundle.Error();
    }
    ImGui::Render(); ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    SDL_GL_SwapWindow(window);
  }

  if (!save_mixer_settings(input_mixer))
    std::fprintf(stderr, "Could not save USB mixer settings on exit.\n");
  if (pad) SDL_CloseGamepad(pad);
  if (joystick) SDL_CloseJoystick(joystick);
  ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplSDL3_Shutdown(); ImGui::DestroyContext();
  glDeleteTextures(1, &preview_texture);
  if (setup_model) {
    mjr_freeContext(&setup_context); mjv_freeScene(&setup_scene);
    mj_deleteData(setup_data); mj_deleteModel(setup_model);
  }
  mjr_freeContext(&context); mjv_freeScene(&scene);
  SDL_GL_DestroyContext(gl); SDL_DestroyWindow(window); SDL_Quit();
  mj_deleteData(data); mj_deleteModel(model);
  return 0;
}


