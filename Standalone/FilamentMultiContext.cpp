// MuJoCo's experimental Filament renderer keeps one context per thread.
// CombatSim needs a drive render context and a CAD-preview context at once.
// This source preserves MuJoCo's API while looking up each context by its
// mjrContext address.

#include "experimental/filament/render_context_filament.h"

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <unordered_map>

#include <mujoco/mujoco.h>
#include "experimental/filament/filament/filament_context.h"

namespace {
thread_local std::unordered_map<const mjrContext*,
                                std::unique_ptr<mujoco::FilamentContext>>
    g_filament_contexts;

int DefaultLoadAsset(const char* filename, void*, unsigned char** contents,
                     uint64_t* out_size) {
  std::filesystem::path path = std::filesystem::path("filament/assets/data") /
                               filename;
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    mju_error("File does not exist: %s", path.string().c_str());
    return 1;
  }
  file.seekg(0, std::ios::end);
  const auto size = file.tellg();
  if (size <= 0) {
    mju_error("File is empty: %s", path.string().c_str());
    return 1;
  }
  *out_size = static_cast<uint64_t>(size);
  file.seekg(0, std::ios::beg);
  *contents = static_cast<unsigned char*>(std::malloc(*out_size));
  if (!*contents) return 1;
  file.read(reinterpret_cast<char*>(*contents), static_cast<std::streamsize>(size));
  return file ? 0 : 1;
}

mujoco::FilamentContext* FindContext(const mjrContext* context) {
  const auto found = g_filament_contexts.find(context);
  if (found == g_filament_contexts.end() || !found->second) {
    mju_error("Missing Filament context for this mjrContext.");
    return nullptr;
  }
  return found->second.get();
}
}  // namespace

extern "C" {

void mjr_defaultFilamentConfig(mjrFilamentConfig* config) {
  std::memset(config, 0, sizeof(*config));
}

void mjr_makeFilamentContext(const mjModel* model, mjrContext* context,
                             const mjrFilamentConfig* config) {
  if (g_filament_contexts.find(context) != g_filament_contexts.end()) {
    mju_error("Filament context already exists for this mjrContext.");
  }
  g_filament_contexts.emplace(
      context, std::make_unique<mujoco::FilamentContext>(config, model, context));
}

void mjr_defaultContext(mjrContext* context) {
  std::memset(context, 0, sizeof(*context));
}

void mjr_makeContext(const mjModel* model, mjrContext* context, int) {
  mjr_freeContext(context);
  mjrFilamentConfig config;
  mjr_defaultFilamentConfig(&config);
  config.load_asset = DefaultLoadAsset;
  mjr_makeFilamentContext(model, context, &config);
}

void mjr_freeContext(mjrContext* context) {
  g_filament_contexts.erase(context);
  mjr_defaultContext(context);
}

void mjr_render(mjrRect viewport, mjvScene* scene, const mjrContext* context) {
  FindContext(context)->Render(viewport, scene, context);
}

void mjr_uploadMesh(const mjModel* model, const mjrContext* context,
                    int meshid) {
  FindContext(context)->UploadMesh(model, meshid);
}

void mjr_uploadTexture(const mjModel* model, const mjrContext* context,
                       int texid) {
  FindContext(context)->UploadTexture(model, texid);
}

void mjr_uploadHField(const mjModel* model, const mjrContext* context,
                      int hfieldid) {
  FindContext(context)->UploadHeightField(model, hfieldid);
}

void mjr_setBuffer(int framebuffer, mjrContext* context) {
  FindContext(context)->SetFrameBuffer(framebuffer);
}

void mjr_readPixels(unsigned char* rgb, float* depth, mjrRect viewport,
                    const mjrContext* context) {
  FindContext(context)->ReadPixels(viewport, rgb, depth);
}

uintptr_t mjr_uploadGuiImage(uintptr_t texture, const unsigned char* pixels,
                             int width, int height, int bpp,
                             const mjrContext* context) {
  return FindContext(context)->UploadGuiImage(texture, pixels, width, height,
                                               bpp);
}

double mjr_getFrameRate(const mjrContext* context) {
  return FindContext(context)->GetFrameRate();
}

}  // extern "C"
