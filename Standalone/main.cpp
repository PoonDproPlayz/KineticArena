#include <mujoco/mujoco.h>
#include <SDL3/SDL.h>
#include <SDL3/SDL_opengl.h>
#include "imgui.h"
#include "backends/imgui_impl_sdl3.h"
#include "backends/imgui_impl_opengl3.h"
#include "FusionBundle.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace {
constexpr double pi = 3.14159265358979323846;
struct Controls {
  float wheel_rpm = 424.0f;
  float weapon_rpm = 30000.0f;
  float spinup_seconds = 1.5f;
  float deadzone = 0.15f;
  double weapon_command = 0.0;
  std::vector<std::pair<int, double>> left, right;
  std::vector<int> weapon;
  std::map<int, MotorSpec> motor_models;
  double battery_voltage = 0.0;
  double weapon_sign = -1.0;
};

int actuator(const mjModel* model, const char* name) {
  return mj_name2id(model, mjOBJ_ACTUATOR, name);
}

std::array<double, 2> mix_drive(float throttle, float turn, double speed) {
  double left = throttle * speed, right = throttle * speed;
  if (throttle == 0.0f) {
    left = -turn * speed;
    right = turn * speed;
  } else if (turn != 0.0f) {
    if (turn * throttle > 0.0f) left = 0.0;
    else right = 0.0;
  }
  return {left, right};
}

void bind_stock_actuators(const mjModel* model, Controls& c) {
  c.left.clear(); c.right.clear(); c.weapon.clear();
  c.motor_models.clear(); c.battery_voltage = 0.0;
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

double motor_output_torque(const MotorSpec& motor, double battery_voltage,
                           double desired_output_speed, double actual_output_speed) {
  if (!motor.enabled || !std::isfinite(battery_voltage) || battery_voltage <= 0.0 ||
      motor.kv_rpm_per_volt <= 0.0 || motor.peak_current_a <= 0.0 ||
      motor.gear_ratio <= 0.0 || motor.motor_count <= 0) return 0.0;
  const double motor_speed_constant = motor.kv_rpm_per_volt * 2.0 * pi / 60.0;
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
    // Without winding resistance, use the linear current-limited torque/speed
    // curve from stall current to zero torque at the commanded back-EMF speed.
    const double normalized_speed = motor_speed / motor_no_load_speed;
    current = std::clamp(duty - normalized_speed, -1.0, 1.0) * current_limit;
  }
  const double torque_constant = 1.0 / motor_speed_constant;
  return torque_constant * current * motor.gear_ratio *
    motor.gearbox_efficiency * motor.motor_count;
}

double motor_group_torque_per_wheel(const MotorSpec& motor, double battery_voltage,
                                    double desired_output_speed,
                                    double actual_output_speed, size_t wheel_count) {
  if (wheel_count == 0) return 0.0;
  return motor_output_torque(motor, battery_voltage, desired_output_speed,
                             actual_output_speed) / static_cast<double>(wheel_count);
}

void apply_wheel_drive_targets(const mjModel* model, mjData* data, const Controls& controls,
                               double left_speed, double right_speed) {
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
    if (motor == controls.motor_models.end()) {
      data->ctrl[id] = desired_joint_speed;
      return;
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
  for (const auto& [id, direction] : controls.left)
    set_wheel_actuator(id, direction, left_speed);
  for (const auto& [id, direction] : controls.right)
    set_wheel_actuator(id, direction, right_speed);
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

void drive(const mjModel* model, mjData* data, Controls& c, SDL_Gamepad* pad, bool keyboard_enabled) {
  const bool* keys = SDL_GetKeyboardState(nullptr);
  float throttle = keyboard_enabled ? float(keys[SDL_SCANCODE_W]) - float(keys[SDL_SCANCODE_S]) : 0.0f;
  float turn = keyboard_enabled ? float(keys[SDL_SCANCODE_A]) - float(keys[SDL_SCANCODE_D]) : 0.0f;
  const float pad_throttle = -axis_with_deadzone(pad, SDL_GAMEPAD_AXIS_LEFTY, c.deadzone);
  const float pad_turn = -axis_with_deadzone(pad, SDL_GAMEPAD_AXIS_LEFTX, c.deadzone);
  if (pad_throttle != 0.0f) throttle = pad_throttle;
  if (pad_turn != 0.0f) turn = pad_turn;

  const double speed = c.wheel_rpm * 2.0 * pi / 60.0;
  const auto [left, right] = mix_drive(throttle, turn, speed);
  apply_wheel_drive_targets(model, data, c, left, right);

  const bool weapon_pressed = (keyboard_enabled && keys[SDL_SCANCODE_SPACE]) ||
    (pad && (SDL_GetGamepadButton(pad, SDL_GAMEPAD_BUTTON_LEFT_SHOULDER) ||
             SDL_GetGamepadButton(pad, SDL_GAMEPAD_BUTTON_SOUTH)));
  const double max_weapon = c.weapon_rpm * 2.0 * pi / 60.0;
  const double target = weapon_pressed ? max_weapon : 0.0;
  const double delta = max_weapon * model->opt.timestep / std::max(0.01f, c.spinup_seconds);
  c.weapon_command += std::clamp(target - c.weapon_command, -delta, delta);
  for (int id : c.weapon) data->ctrl[id] = c.weapon_sign * c.weapon_command;
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

SDL_Gamepad* first_gamepad() {
  int count = 0;
  SDL_JoystickID* ids = SDL_GetGamepads(&count);
  SDL_Gamepad* pad = count > 0 ? SDL_OpenGamepad(ids[0]) : nullptr;
  SDL_free(ids);
  return pad;
}
}

int main(int argc, char** argv) {
  FusionBundle bundle;
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
    const double ideal_stall = motor_output_torque(motor, voltage, free_speed, 0.0);
    const double ideal_free = motor_output_torque(motor, voltage, free_speed, free_speed);
    const double shared_wheel_torque = motor_group_torque_per_wheel(
      motor, voltage, free_speed, 0.0, 2);
    motor.phase_resistance_ohm = voltage / motor.peak_current_a;
    const double resistive_stall = motor_output_torque(motor, voltage, free_speed, 0.0);
    const double resistive_free = motor_output_torque(motor, voltage, free_speed, free_speed);
    if (std::abs(ideal_stall - expected_stall) > 1e-8 ||
        std::abs(resistive_stall - expected_stall) > 1e-8 ||
        std::abs(2.0 * shared_wheel_torque - expected_stall) > 1e-8 ||
        std::abs(ideal_free) > 1e-8 || std::abs(resistive_free) > 1e-8) {
      std::fprintf(stderr, "Motor torque model failed its stall/free-speed checks.\n");
      return 1;
    }
    std::printf("Motor torque model and shared-wheel split passed: %.4f Nm total stall torque; zero torque at %.0f output RPM.\n",
                expected_stall, free_speed * 60.0 / (2.0 * pi));
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
    double voltage = 0.0;
    bundle.BindControls(checked, left, right, weapons, motors, voltage);
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
    mj_deleteData(checked_data); mj_deleteModel(checked);
    if (!finite || final_rpm < 0.1 * free_rpm || final_rpm > 1.05 * free_rpm) {
      std::fprintf(stderr, "Motor actuator spin check was %.1f RPM versus %.1f RPM free speed.\n",
                   final_rpm, free_rpm);
      return 1;
    }
    std::printf("Shared motor spin test passed: %d wheel assemblies, %.0f RPM of %.0f RPM no-load.\n",
                static_cast<int>(shared_motor_actuators.size()), final_rpm, free_rpm);
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
    double bot_battery_voltage = 0.0;
    bundle.BindControls(checked, left_wheels, right_wheels, weapon_drives,
                        motor_models, bot_battery_voltage);
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
  if (!data || !SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMEPAD)) {
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
      drive(model, data, controls, pad, !ImGui::GetIO().WantCaptureKeyboard);
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
    bool load_requested = false, apply_requested = false;
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
    ImGui::TextColored(pad ? ImVec4(0.35f, 0.84f, 0.58f, 1.0f) :
                             ImVec4(0.68f, 0.72f, 0.78f, 1.0f),
                       "%s", pad ? "GAMEPAD CONNECTED" : "KEYBOARD READY");
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
      ImGui::SetNextWindowPos(ImVec2(16, 76), ImGuiCond_Always);
      ImGui::SetNextWindowSize(ImVec2(350, display.y - 92), ImGuiCond_Always);
      ImGui::Begin("##drive_dashboard", nullptr, fixed_window);
      ImGui::TextColored(ImVec4(0.32f, 0.88f, 0.89f, 1.0f), "PRACTICE DRIVE");
      ImGui::TextDisabled("MuJoCo simulation  |  %.0f Hz", 1.0 / model->opt.timestep);
      ImGui::Spacing();
      if (ImGui::Button(paused ? "Resume simulation" : "Pause simulation", ImVec2(166, 40)))
        paused = !paused;
      ImGui::SameLine();
      if (ImGui::Button("Reset robot", ImVec2(132, 40))) {
        mj_resetData(model, data); mj_forward(model, data); controls.weapon_command = 0.0;
      }
      ImGui::Text("Simulation time  %.2f s", data->time);
      ImGui::SeparatorText("Drive settings");
      ImGui::SetNextItemWidth(-1);
      ImGui::InputFloat("Wheel target RPM (0-5000)", &controls.wheel_rpm, 50.0f, 250.0f, "%.0f");
      if (!std::isfinite(controls.wheel_rpm)) controls.wheel_rpm = 0.0f;
      controls.wheel_rpm = std::clamp(controls.wheel_rpm, 0.0f, 5000.0f);
      ImGui::SetNextItemWidth(-1);
      ImGui::SliderFloat("Weapon target RPM", &controls.weapon_rpm, 0, 30000, "%.0f RPM");
      ImGui::SetNextItemWidth(-1);
      ImGui::SliderFloat("Weapon spin-up", &controls.spinup_seconds, 0.1f, 5.0f, "%.1f s");
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
      ImGui::ProgressBar(controls.weapon_rpm > 0 ?
        float(std::clamp(weapon_actual / controls.weapon_rpm, 0.0, 1.0)) : 0.0f,
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
        }
        if (valid) ImGui::TextColored(ImVec4(0.75f, 0.82f, 0.9f, 1.0f),
                                      "  Average: %+.0f RPM", sum / valid);
      };
      draw_wheel_telemetry("Left", controls.left);
      draw_wheel_telemetry("Right", controls.right);
      ImGui::SeparatorText("Controls");
      ImGui::TextWrapped("W / S drive forward and reverse. A / D steer; with no throttle, both sides counter-rotate for a spot turn. Hold Space to spin the weapon.");
      ImGui::Text("Drive joints: left %d  |  right %d",
                  int(controls.left.size()), int(controls.right.size()));
      if (controls.left.empty() || controls.right.empty())
        ImGui::TextWrapped("Assign at least one wheel assembly to each side in Bot workshop, then Apply and preview.");
      if (!runtime_error.empty()) ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.4f, 1.0f),
                                                       "Model message: %s", runtime_error.c_str());
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
      if (editor_width <= 0.0f)
        editor_width = std::min(480.0f, std::max(340.0f, display.x * 0.39f));
      editor_width = std::clamp(editor_width, min_editor_width, max_editor_width);
      const float splitter_height = ImGui::GetContentRegionAvail().y;
      ImGui::BeginChild("Bot configuration", ImVec2(editor_width, 0),
                        ImGuiChildFlags_Borders, ImGuiWindowFlags_AlwaysVerticalScrollbar);
      ImGui::TextColored(ImVec4(0.32f, 0.88f, 0.89f, 1.0f), "BOT CONFIGURATION");
      ImGui::TextDisabled("Import and assign your robot components");
      ImGui::Separator();
      bundle.Draw(window, load_requested, apply_requested);
      if (!runtime_error.empty()) ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.4f, 1.0f),
                                                       "Model message: %s", runtime_error.c_str());
      ImGui::EndChild();
      ImGui::SameLine();
      ImGui::InvisibleButton("##bot_config_splitter", ImVec2(splitter_width, splitter_height));
      const bool splitter_hovered = ImGui::IsItemHovered();
      const bool splitter_active = ImGui::IsItemActive();
      if (splitter_hovered || splitter_active)
        ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
      if (splitter_active)
        editor_width = std::clamp(editor_width + ImGui::GetIO().MouseDelta.x,
                                  min_editor_width, max_editor_width);
      const ImVec2 splitter_min = ImGui::GetItemRectMin();
      const ImVec2 splitter_max = ImGui::GetItemRectMax();
      const ImU32 splitter_color = ImGui::GetColorU32(splitter_active ? ImGuiCol_SeparatorActive :
          splitter_hovered ? ImGuiCol_SeparatorHovered : ImGuiCol_Separator);
      ImGui::GetWindowDrawList()->AddRectFilled(
          ImVec2(splitter_min.x + splitter_width * 0.5f - 1.0f, splitter_min.y + 4.0f),
          ImVec2(splitter_min.x + splitter_width * 0.5f + 1.0f, splitter_max.y - 4.0f),
          splitter_color, 1.0f);
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
                              controls.motor_models, controls.battery_voltage);
          controls.weapon_sign = 1.0; controls.weapon_command = 0.0;
          accumulator = 0.0; paused = true; runtime_error.clear();
        }
      } else runtime_error = bundle.Error();
    }
    ImGui::Render(); ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    SDL_GL_SwapWindow(window);
  }

  if (pad) SDL_CloseGamepad(pad);
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


