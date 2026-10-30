cmake_minimum_required(VERSION 3.24)
project(CombatSimStandalone LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(SDL3 GIT_REPOSITORY https://github.com/libsdl-org/SDL.git GIT_TAG release-3.2.22 GIT_SHALLOW TRUE)
FetchContent_Declare(imgui GIT_REPOSITORY https://github.com/ocornut/imgui.git GIT_TAG v1.92.4-docking GIT_SHALLOW TRUE)
FetchContent_Declare(json GIT_REPOSITORY https://github.com/nlohmann/json.git GIT_TAG v3.11.3 GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(SDL3 imgui json)

option(COMBATSIM_USE_FILAMENT "Build CombatSim with MuJoCo's experimental Filament PBR renderer" OFF)
if(COMBATSIM_USE_FILAMENT)
  if(NOT TARGET mujoco OR NOT TARGET mujoco_filament)
    message(FATAL_ERROR
      "Filament mode must be configured through Standalone/build_filament.ps1 so MuJoCo 3.4 and its renderer are built together.")
  endif()
  set(COMBAT_SIM_MUJOCO_TARGET mujoco)
else()
  set(MUJOCO_ROOT "" CACHE PATH "Unpacked MuJoCo 3.3.7 Windows release (contains include/mujoco and bin/mujoco.dll)")
  if(NOT EXISTS "${MUJOCO_ROOT}/include/mujoco/mujoco.h" OR NOT EXISTS "${MUJOCO_ROOT}/bin/mujoco.dll")
    message(FATAL_ERROR "Set MUJOCO_ROOT to the unpacked MuJoCo 3.3.7 Windows release")
  endif()

  add_library(mujoco SHARED IMPORTED)
  set_target_properties(mujoco PROPERTIES
    IMPORTED_IMPLIB "${MUJOCO_ROOT}/lib/mujoco.lib"
    IMPORTED_LOCATION "${MUJOCO_ROOT}/bin/mujoco.dll"
    INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_ROOT}/include")
  set(COMBAT_SIM_MUJOCO_TARGET mujoco)
endif()

add_executable(CombatSimStandalone main.cpp AppSettings.cpp FusionBundle.cpp LanSession.cpp Replay.cpp
  ${imgui_SOURCE_DIR}/imgui.cpp ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl3.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp)
target_include_directories(CombatSimStandalone PRIVATE ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
target_link_libraries(CombatSimStandalone PRIVATE ${COMBAT_SIM_MUJOCO_TARGET}
  SDL3::SDL3-shared nlohmann_json::nlohmann_json opengl32 ws2_32 bcrypt shell32)
if(COMBATSIM_USE_FILAMENT)
  target_compile_definitions(CombatSimStandalone PRIVATE
    COMBATSIM_USE_FILAMENT=1 MJ_STATIC
    COMBATSIM_FILAMENT_ASSETS_DIR=\"${CMAKE_BINARY_DIR}/src/experimental/filament/assets\")
  target_link_libraries(CombatSimStandalone PRIVATE mujoco_filament)
  add_custom_command(TARGET CombatSimStandalone POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "$<TARGET_FILE:mujoco>" "$<TARGET_FILE_DIR:CombatSimStandalone>"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "$<TARGET_FILE:SDL3::SDL3-shared>" "$<TARGET_FILE_DIR:CombatSimStandalone>")
else()
  add_custom_command(TARGET CombatSimStandalone POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${MUJOCO_ROOT}/bin/mujoco.dll" "$<TARGET_FILE_DIR:CombatSimStandalone>"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "$<TARGET_FILE:SDL3::SDL3-shared>" "$<TARGET_FILE_DIR:CombatSimStandalone>")
endif()
