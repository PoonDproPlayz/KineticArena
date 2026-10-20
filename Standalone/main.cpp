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
  std::vector<int> left, right, weapon;
  double weapon_sign = -1.0;
};

int actuator(const mjModel* model, const char* name) {
  return mj_name2id(model, mjOBJ_ACTUATOR, name);
}

void bind_stock_actuators(const mjModel* model, Controls& c) {
  c.left.clear(); c.right.clear(); c.weapon.clear();
  for (const auto* name : {"front_left_drive", "rear_left_drive"}) {
    const int id = actuator(model, name); if (id >= 0) c.left.push_back(id);
  }
  for (const auto* name : {"front_right_drive", "rear_right_drive"}) {
    const int id = actuator(model, name); if (id >= 0) c.right.push_back(id);
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

void drive(const mjModel* model, mjData* data, Controls& c, SDL_Gamepad* pad, bool keyboard_enabled) {
  const bool* keys = SDL_GetKeyboardState(nullptr);
  float throttle = keyboard_enabled ? float(keys[SDL_SCANCODE_W]) - float(keys[SDL_SCANCODE_S]) : 0.0f;
  float turn = keyboard_enabled ? float(keys[SDL_SCANCODE_A]) - float(keys[SDL_SCANCODE_D]) : 0.0f;
  const float pad_throttle = -axis_with_deadzone(pad, SDL_GAMEPAD_AXIS_LEFTY, c.deadzone);
  const float pad_turn = -axis_with_deadzone(pad, SDL_GAMEPAD_AXIS_LEFTX, c.deadzone);
  if (pad_throttle != 0.0f) throttle = pad_throttle;
  if (pad_turn != 0.0f) turn = pad_turn;

  const double speed = c.wheel_rpm * 2.0 * pi / 60.0;
  double left = throttle * speed, right = throttle * speed;
  if (throttle == 0.0f) {
    left = -turn * speed;
    right = turn * speed;
  } else if (turn != 0.0f) {
    if (turn * throttle > 0.0f) left = 0.0;
    else right = 0.0;
  }

  const bool weapon_pressed = (keyboard_enabled && keys[SDL_SCANCODE_SPACE]) ||
    (pad && (SDL_GetGamepadButton(pad, SDL_GAMEPAD_BUTTON_LEFT_SHOULDER) ||
             SDL_GetGamepadButton(pad, SDL_GAMEPAD_BUTTON_SOUTH)));
  const double max_weapon = c.weapon_rpm * 2.0 * pi / 60.0;
  const double target = weapon_pressed ? max_weapon : 0.0;
  const double delta = max_weapon * model->opt.timestep / std::max(0.01f, c.spinup_seconds);
  c.weapon_command += std::clamp(target - c.weapon_command, -delta, delta);
  for (int id : c.left) data->ctrl[id] = left;
  for (int id : c.right) data->ctrl[id] = right;
  for (int id : c.weapon) data->ctrl[id] = c.weapon_sign * c.weapon_command;
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
    mjData* sample = mj_makeData(checked);
    if (!sample) { mj_deleteModel(checked); std::fprintf(stderr, "MuJoCo could not allocate model data.\n"); return 1; }
    for (int step = 0; step < 2500; ++step) mj_step(checked, sample);
    bool finite = std::isfinite(sample->time);
    for (int i = 0; i < checked->nq; ++i) finite = finite && std::isfinite(sample->qpos[i]);
    mj_deleteData(sample);
    if (!finite) { mj_deleteModel(checked); std::fprintf(stderr, "MuJoCo state became nonfinite during the one-second step test.\n"); return 1; }
    bundle.StartAxis(1);
    for (const auto& point : {std::array<double, 3>{0.12, 0.08, 0.09},
                              {0.10, 0.10, 0.09}, {0.08, 0.08, 0.09},
                              {0.10, 0.06, 0.09}})
      if (!bundle.AddAxisPoint(1, point, false)) {
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
      if (!bundle.AddAxisPoint(1, point, false)) {
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
    if (!bundle.AddAxisPoint(1, {0.10, 0.0, 0.0}) || bundle.AxisPointCount() != 1) {
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
  ImGui_ImplSDL3_InitForOpenGL(window, gl);
  ImGui_ImplOpenGL3_Init("#version 330");

  bool running = true, paused = false, show_cad_setup = true;
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
    if (paused) accumulator = 0.0;
    int steps = 0;
    while (!paused && accumulator >= model->opt.timestep && steps++ < 250) {
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
    if (show_cad_setup) {
    ImGui::SetNextWindowPos(ImVec2(320, 45), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(std::max(600.0f, std::min(1050.0f, ImGui::GetIO().DisplaySize.x-330.0f)),
                                    std::max(420.0f, std::min(680.0f, ImGui::GetIO().DisplaySize.y-60.0f))),
                             ImGuiCond_FirstUseEver);
    ImGui::Begin("CAD setup");
    ImGui::BeginChild("Parts and axis controls", ImVec2(380, 0), ImGuiChildFlags_Borders);
    bundle.Draw(window, load_requested, apply_requested);
    ImGui::EndChild();
    ImGui::SameLine();
    ImGui::BeginChild("3D CAD preview", ImVec2(0, 0), ImGuiChildFlags_Borders);
    bundle.DrawAxisSelectionControls();
    ImGui::TextWrapped("Click a part to select. In axis mode, click rim edges around a hole. Left drag: orbit and tilt. Right drag: pan. Scroll: zoom.");
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
      const int selected = bundle.SelectedPart();
      if (selected >= 0) {
        const std::string name = "cad_part_" + std::to_string(selected);
        const int geom_id = mj_name2id(setup_model, mjOBJ_GEOM, name.c_str());
        for (int i = 0; i < setup_scene.ngeom; ++i)
          if (setup_scene.geoms[i].objtype == mjOBJ_GEOM && setup_scene.geoms[i].objid == geom_id) {
            setup_scene.geoms[i].rgba[0] = 1.0f;
            setup_scene.geoms[i].rgba[1] = 0.8f;
            setup_scene.geoms[i].rgba[2] = 0.1f;
          }
      }
      bundle.AddSetupAxisMarkers(setup_scene);
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
          const ImVec2 mouse = ImGui::GetIO().MousePos;
          // MuJoCo 3.3.7's mjv_select writes all three IDs, including on misses.
          mjtNum hit[3]; int geom_id = -1, flex_id = -1, skin_id = -1;
          mjv_select(setup_model, setup_data, &options, double(preview_width)/preview_height,
            (mouse.x-image_min.x)/preview_width, 1.0-(mouse.y-image_min.y)/preview_height,
            &setup_scene, hit, &geom_id, &flex_id, &skin_id);
          const char* picked = geom_id >= 0 ? mj_id2name(setup_model, mjOBJ_GEOM, geom_id) : nullptr;
          if (picked && std::string(picked).rfind("cad_part_", 0) == 0) {
            const int part = std::stoi(std::string(picked).substr(9));
            if (bundle.AxisPicking()) bundle.AddAxisPoint(part, {hit[0], hit[1], hit[2]});
            else bundle.SelectPart(part);
          }
        }
      }
    } else ImGui::TextWrapped("Load a Fusion manifest to preview and select its parts.");
    ImGui::EndChild();
    ImGui::End();
    }
    ImGui::SetNextWindowPos(ImVec2(12, 12), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(300, 250), ImGuiCond_FirstUseEver);
    ImGui::Begin("CombatSim");
    ImGui::Text("Time: %.2f s", data->time);
    ImGui::Text("Gamepad: %s", pad ? SDL_GetGamepadName(pad) : "none");
    if (ImGui::Button(paused ? "Resume" : "Pause")) paused = !paused;
    ImGui::SameLine();
    if (ImGui::Button("Reset")) { mj_resetData(model, data); mj_forward(model, data); controls.weapon_command = 0.0; }
    if (ImGui::Button(show_cad_setup ? "Hide CAD setup" : "Open CAD setup")) show_cad_setup = !show_cad_setup;
    ImGui::SliderFloat("Wheel RPM", &controls.wheel_rpm, 0, 1000);
    ImGui::SliderFloat("Weapon RPM", &controls.weapon_rpm, 0, 30000);
    ImGui::SliderFloat("Spin-up seconds", &controls.spinup_seconds, 0.1f, 5.0f);
    ImGui::TextWrapped("W/S drive, A/D turn, Space weapon. Left stick and left shoulder/A on gamepad. Drag to orbit, scroll to zoom.");
    if (!runtime_error.empty()) ImGui::TextWrapped("Model error: %s", runtime_error.c_str());
    ImGui::End();
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
          bundle.BindControls(model, controls.left, controls.right, controls.weapon);
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


