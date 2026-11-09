#pragma once

// Registers the STL-backed signed-distance collider used by Fusion imports.
// Call once before loading any generated MJCF that references this plugin.
bool RegisterStlMeshSdfPlugin();
