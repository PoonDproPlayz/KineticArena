# Included at the end of MuJoCo's CMakeLists when building from source.
# Add CombatSim only after MuJoCo's core and Filament targets exist.
if(NOT DEFINED COMBATSIM_SOURCE_ROOT)
  message(FATAL_ERROR "COMBATSIM_SOURCE_ROOT must point to the CombatSim checkout.")
endif()

if(NOT TARGET mujoco_filament OR NOT TARGET mujoco)
  message(FATAL_ERROR "MuJoCo 3.4 Filament targets were not created.")
endif()

get_target_property(_filament_sources mujoco_filament SOURCES)
set(_filtered_sources)
foreach(_source IN LISTS _filament_sources)
  get_filename_component(_source_name "${_source}" NAME)
  if(NOT _source_name STREQUAL "render_context_filament.cc")
    list(APPEND _filtered_sources "${_source}")
  endif()
endforeach()
set_property(TARGET mujoco_filament PROPERTY SOURCES "${_filtered_sources}")
set_property(TARGET mujoco_filament PROPERTY INTERFACE_SOURCES "")
target_sources(mujoco_filament PRIVATE
  "${COMBATSIM_SOURCE_ROOT}/Standalone/FilamentMultiContext.cpp")
if(TARGET dear_imgui)
  set_property(TARGET dear_imgui PROPERTY INTERFACE_SOURCES "")
endif()

add_subdirectory("${COMBATSIM_SOURCE_ROOT}/Standalone"
                 "${CMAKE_BINARY_DIR}/combat-sim-standalone")
