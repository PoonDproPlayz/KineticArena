cmake_minimum_required(VERSION 3.24)
project(CombatSimStandalone LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(SDL3 GIT_REPOSITORY https://github.com/libsdl-org/SDL.git GIT_TAG release-3.2.22 GIT_SHALLOW TRUE)
FetchContent_Declare(imgui GIT_REPOSITORY https://github.com/ocornut/imgui.git GIT_TAG v1.92.4-docking GIT_SHALLOW TRUE)
FetchContent_Declare(json GIT_REPOSITORY https://github.com/nlohmann/json.git GIT_TAG v3.11.3 GIT_SHALLOW TRUE)
FetchContent_MakeAvailable(SDL3 imgui json)

set(MUJOCO_ROOT "" CACHE PATH "Unpacked MuJoCo 3.3.7 Windows release (contains include/mujoco and bin/mujoco.dll)")
if(NOT EXISTS "${MUJOCO_ROOT}/include/mujoco/mujoco.h" OR NOT EXISTS "${MUJOCO_ROOT}/bin/mujoco.dll")
  message(FATAL_ERROR "Set MUJOCO_ROOT to the unpacked MuJoCo 3.3.7 Windows release")
endif()

add_library(mujoco SHARED IMPORTED)
set_target_properties(mujoco PROPERTIES
  IMPORTED_IMPLIB "${MUJOCO_ROOT}/lib/mujoco.lib"
  IMPORTED_LOCATION "${MUJOCO_ROOT}/bin/mujoco.dll"
  INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_ROOT}/include")

add_executable(CombatSimStandalone main.cpp FusionBundle.cpp
  ${imgui_SOURCE_DIR}/imgui.cpp ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl3.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp)
target_include_directories(CombatSimStandalone PRIVATE ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
target_link_libraries(CombatSimStandalone PRIVATE mujoco SDL3::SDL3-shared nlohmann_json::nlohmann_json opengl32)
add_custom_command(TARGET CombatSimStandalone POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${MUJOCO_ROOT}/bin/mujoco.dll" "$<TARGET_FILE_DIR:CombatSimStandalone>"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "$<TARGET_FILE:SDL3::SDL3-shared>" "$<TARGET_FILE_DIR:CombatSimStandalone>")
