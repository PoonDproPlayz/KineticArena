#include "Replay.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>

namespace {
constexpr std::array<char, 8> replay_magic{'K', 'A', 'R', 'E', 'P', 'L', 'Y', '2'};
constexpr uint32_t replay_version = 2;
constexpr uint64_t max_replay_file_bytes = 512ull * 1024ull * 1024ull;
constexpr double max_recording_seconds = 600.0;

class Hasher {
 public:
  void Bytes(const void* data, size_t size) {
    const auto* bytes = static_cast<const unsigned char*>(data);
    for (size_t i = 0; i < size; ++i) {
      value_ ^= bytes[i];
      value_ *= 1099511628211ull;
    }
  }

  template <typename T>
  void Value(const T& value) { Bytes(&value, sizeof(value)); }

  void Text(const char* text) {
    if (text) Bytes(text, std::strlen(text));
    const unsigned char separator = 0xff;
    Bytes(&separator, sizeof(separator));
  }

  uint64_t Finish() const { return value_; }

 private:
  uint64_t value_ = 14695981039346656037ull;
};

uint64_t model_signature(const mjModel* model) {
  Hasher hash;
  hash.Value(model->nq); hash.Value(model->nv); hash.Value(model->nu);
  hash.Value(model->nbody); hash.Value(model->njnt); hash.Value(model->ngeom);
  hash.Value(model->nmesh); hash.Value(model->nmeshvert); hash.Value(model->nmeshface);
  hash.Value(model->opt.timestep);
  for (int i = 0; i < model->nbody; ++i) {
    hash.Text(mj_id2name(model, mjOBJ_BODY, i));
    hash.Value(model->body_parentid[i]);
    hash.Bytes(model->body_pos + 3*i, 3*sizeof(mjtNum));
    hash.Bytes(model->body_quat + 4*i, 4*sizeof(mjtNum));
    hash.Bytes(model->body_ipos + 3*i, 3*sizeof(mjtNum));
    hash.Bytes(model->body_mass + i, sizeof(mjtNum));
    hash.Bytes(model->body_inertia + 3*i, 3*sizeof(mjtNum));
  }
  for (int i = 0; i < model->njnt; ++i) {
    hash.Text(mj_id2name(model, mjOBJ_JOINT, i));
    hash.Value(model->jnt_type[i]); hash.Value(model->jnt_bodyid[i]);
    hash.Value(model->jnt_qposadr[i]);
    hash.Bytes(model->jnt_pos + 3*i, 3*sizeof(mjtNum));
    hash.Bytes(model->jnt_axis + 3*i, 3*sizeof(mjtNum));
  }
  for (int i = 0; i < model->ngeom; ++i) {
    hash.Text(mj_id2name(model, mjOBJ_GEOM, i));
    hash.Value(model->geom_type[i]); hash.Value(model->geom_bodyid[i]);
    hash.Bytes(model->geom_size + 3*i, 3*sizeof(mjtNum));
    hash.Bytes(model->geom_pos + 3*i, 3*sizeof(mjtNum));
    hash.Bytes(model->geom_quat + 4*i, 4*sizeof(mjtNum));
    hash.Value(model->geom_dataid[i]);
    hash.Value(model->geom_contype[i]);
    hash.Value(model->geom_conaffinity[i]);
    hash.Bytes(model->geom_friction + 3*i, 3*sizeof(mjtNum));
    hash.Bytes(model->geom_rgba + 4*i, 4*sizeof(float));
  }
  for (int i = 0; i < model->nu; ++i) {
    hash.Text(mj_id2name(model, mjOBJ_ACTUATOR, i));
    hash.Bytes(model->actuator_trnid + 2*i, 2*sizeof(int));
    hash.Bytes(model->actuator_ctrlrange + 2*i, 2*sizeof(mjtNum));
    hash.Bytes(model->actuator_gear + 6*i, 6*sizeof(mjtNum));
  }
  for (int i = 0; i < model->nmeshvert * 3; ++i) hash.Value(model->mesh_vert[i]);
  for (int i = 0; i < model->nmeshface * 3; ++i) hash.Value(model->mesh_face[i]);
  return hash.Finish();
}

template <typename T>
bool write_value(std::ofstream& output, const T& value) {
  output.write(reinterpret_cast<const char*>(&value), sizeof(T));
  return bool(output);
}

template <typename T>
bool read_value(std::ifstream& input, T& value) {
  input.read(reinterpret_cast<char*>(&value), sizeof(T));
  return bool(input);
}
}  // namespace

bool FightReplay::Begin(const mjModel* model, const mjData* data, std::string& error) {
  error.clear();
  recording_ = false;
  frames_.clear();
  model_blob_.clear();
  if (!model || !data || model->nq <= 0 || !std::isfinite(data->time)) {
    error = "The current simulation state cannot be recorded.";
    return false;
  }
  nq_ = model->nq;
  timestep_ = model->opt.timestep;
  model_signature_ = model_signature(model);
  const int model_size = mj_sizeModel(model);
  if (model_size <= 0 || uint64_t(model_size) > max_replay_file_bytes / 2) {
    error = "The active model is too large to include in a replay file.";
    return false;
  }
  model_blob_.resize(size_t(model_size));
  mj_saveModel(model, nullptr, model_blob_.data(), model_size);
  start_time_ = data->time;
  recording_ = true;
  error_.clear();
  Frame initial;
  initial.qpos.reserve(nq_);
  for (int i = 0; i < nq_; ++i)
    initial.qpos.push_back(static_cast<float>(data->qpos[i]));
  frames_.push_back(std::move(initial));
  return true;
}

bool FightReplay::Capture(const mjModel* model, const mjData* data, std::string& error) {
  error.clear();
  if (!recording_) return true;
  if (!model || !data || model->nq != nq_) {
    recording_ = false;
    error = "Recording stopped because the active model changed.";
    error_ = error;
    return false;
  }
  const double elapsed = data->time - start_time_;
  if (!std::isfinite(elapsed) || elapsed < 0.0) {
    recording_ = false;
    error = "Recording stopped because simulation time became invalid.";
    error_ = error;
    return false;
  }
  if (elapsed >= max_recording_seconds || frames_.size() >= max_frame_count_) {
    recording_ = false;
    error = "Recording reached the 10 minute limit and stopped.";
    error_ = error;
    return false;
  }
  const double minimum_interval = 1.0 / sample_rate_hz_;
  if (elapsed - frames_.back().time + 1e-10 < minimum_interval) return true;
  const uint64_t frame_bytes = 8 + uint64_t(nq_) * sizeof(float);
  if (48 + uint64_t(model_blob_.size()) +
      uint64_t(frames_.size() + 1) * frame_bytes > max_replay_file_bytes) {
    recording_ = false;
    error = "Recording reached the replay file size limit and stopped.";
    error_ = error;
    return false;
  }

  Frame frame;
  frame.time = elapsed;
  frame.qpos.reserve(nq_);
  for (int i = 0; i < nq_; ++i) frame.qpos.push_back(static_cast<float>(data->qpos[i]));
  frames_.push_back(std::move(frame));
  return true;
}

bool FightReplay::Save(const std::filesystem::path& path, std::string& error) const {
  error.clear();
  if (frames_.empty() || nq_ <= 0) {
    error = "There is no recorded fight to save.";
    return false;
  }
  std::error_code ec;
  std::filesystem::create_directories(path.parent_path(), ec);
  if (ec) {
    error = "Could not create replay folder: " + ec.message();
    return false;
  }
  const uint64_t estimated_size = 8 + 4 + 4 + 8 + 8 + 8 + 8 + model_blob_.size() +
      uint64_t(frames_.size()) * (8 + uint64_t(nq_) * sizeof(float));
  if (estimated_size > max_replay_file_bytes) {
    error = "Replay is too large to save safely.";
    return false;
  }
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  if (!output) {
    error = "Could not open the replay file for writing.";
    return false;
  }
  output.write(replay_magic.data(), replay_magic.size());
  const uint32_t nq = uint32_t(nq_);
  const uint64_t count = uint64_t(frames_.size());
  const uint64_t model_blob_size = model_blob_.size();
  if (!output || !write_value(output, replay_version) || !write_value(output, nq) ||
      !write_value(output, count) || !write_value(output, model_signature_) ||
      !write_value(output, timestep_) || !write_value(output, model_blob_size)) {
    error = "Could not write the replay header.";
    return false;
  }
  output.write(reinterpret_cast<const char*>(model_blob_.data()),
               std::streamsize(model_blob_.size()));
  if (!output) {
    error = "Could not write the embedded simulation model.";
    return false;
  }
  for (const auto& frame : frames_) {
    if (!write_value(output, frame.time)) {
      error = "Could not write replay frame time.";
      return false;
    }
    output.write(reinterpret_cast<const char*>(frame.qpos.data()),
                 std::streamsize(frame.qpos.size() * sizeof(float)));
    if (!output) {
      error = "Could not write replay pose data.";
      return false;
    }
  }
  return true;
}

bool FightReplay::Load(const std::filesystem::path& path, std::string& error) {
  error.clear();
  std::error_code ec;
  const auto file_size = std::filesystem::file_size(path, ec);
  if (ec || file_size < 48 || file_size > max_replay_file_bytes) {
    error = "Replay file is missing, truncated, or too large.";
    return false;
  }
  std::ifstream input(path, std::ios::binary);
  std::array<char, 8> magic{};
  input.read(magic.data(), magic.size());
  uint32_t version = 0, nq = 0;
  uint64_t count = 0, signature = 0, model_blob_size = 0;
  double timestep = 0.0;
  if (!input || magic != replay_magic || !read_value(input, version) ||
      !read_value(input, nq) || !read_value(input, count) ||
      !read_value(input, signature) || !read_value(input, timestep) ||
      !read_value(input, model_blob_size)) {
    error = "This is not a valid KineticArena replay file.";
    return false;
  }
  if (version != replay_version) {
    error = "This replay was made by a different, unsupported format version.";
    return false;
  }
  if (nq == 0 || model_blob_size == 0 || model_blob_size > max_replay_file_bytes / 2) {
    error = "Replay has invalid model data.";
    return false;
  }
  if (count == 0 || count > max_frame_count_ ||
      count > (max_replay_file_bytes - 48 - model_blob_size) /
          (8 + uint64_t(nq) * sizeof(float)) ||
      file_size != 48 + model_blob_size + count * (8 + uint64_t(nq) * sizeof(float))) {
    error = "Replay frame data is invalid or incomplete.";
    return false;
  }
  std::vector<unsigned char> loaded_model(static_cast<size_t>(model_blob_size), 0);
  input.read(reinterpret_cast<char*>(loaded_model.data()),
             std::streamsize(loaded_model.size()));
  if (!input) {
    error = "Replay model data is incomplete.";
    return false;
  }
  std::vector<Frame> loaded;
  loaded.reserve(size_t(count));
  for (uint64_t i = 0; i < count; ++i) {
    Frame frame;
    if (!read_value(input, frame.time) || !std::isfinite(frame.time) ||
        (i && frame.time <= loaded.back().time)) {
      error = "Replay contains invalid timestamps.";
      return false;
    }
    frame.qpos.resize(nq);
    input.read(reinterpret_cast<char*>(frame.qpos.data()),
               std::streamsize(frame.qpos.size() * sizeof(float)));
    if (!input || std::any_of(frame.qpos.begin(), frame.qpos.end(),
        [](float value) { return !std::isfinite(value); })) {
      error = "Replay contains invalid pose data.";
      return false;
    }
    loaded.push_back(std::move(frame));
  }
  if (input.peek() != std::ifstream::traits_type::eof()) {
    error = "Replay contains unexpected trailing data.";
    return false;
  }
  model_blob_ = std::move(loaded_model);
  frames_ = std::move(loaded);
  nq_ = int(nq);
  timestep_ = timestep;
  model_signature_ = signature;
  recording_ = false;
  error_.clear();
  return true;
}

mjModel* FightReplay::CreateModel(std::string& error) const {
  error.clear();
  if (model_blob_.empty() || model_blob_.size() > size_t(std::numeric_limits<int>::max())) {
    error = "Replay has no usable embedded model.";
    return nullptr;
  }
  mjVFS vfs;
  mj_defaultVFS(&vfs);
  if (mj_addBufferVFS(&vfs, "fight.mjb", model_blob_.data(), int(model_blob_.size())) != 0) {
    mj_deleteVFS(&vfs);
    error = "MuJoCo could not read the embedded replay model.";
    return nullptr;
  }
  mjModel* model = mj_loadModel("fight.mjb", &vfs);
  mj_deleteVFS(&vfs);
  if (!model || model->nq != nq_ || model_signature(model) != model_signature_) {
    if (model) mj_deleteModel(model);
    error = "Replay model failed its integrity check.";
    return nullptr;
  }
  return model;
}

bool FightReplay::Apply(const mjModel* model, mjData* data, double elapsed_seconds,
                        std::string& error) const {
  error.clear();
  if (!model || !data || frames_.empty() || model->nq != nq_) {
    error = "The loaded replay does not match the active model.";
    return false;
  }
  const double time = std::clamp(elapsed_seconds, 0.0, Duration());
  auto upper = std::upper_bound(frames_.begin(), frames_.end(), time,
      [](double value, const Frame& frame) { return value < frame.time; });
  if (upper == frames_.begin()) {
    std::copy(frames_.front().qpos.begin(), frames_.front().qpos.end(), data->qpos);
    data->time = 0.0;
  } else if (upper == frames_.end()) {
    std::copy(frames_.back().qpos.begin(), frames_.back().qpos.end(), data->qpos);
    data->time = frames_.back().time;
  } else {
    const Frame& next = *upper;
    const Frame& previous = *(upper - 1);
    const double span = next.time - previous.time;
    const double alpha = span > 0.0 ? std::clamp((time - previous.time) / span, 0.0, 1.0) : 0.0;
    for (int i = 0; i < nq_; ++i)
      data->qpos[i] = previous.qpos[i] + alpha * (next.qpos[i] - previous.qpos[i]);
    data->time = time;
  }
  mj_normalizeQuat(model, data->qpos);
  std::fill(data->qvel, data->qvel + model->nv, 0.0);
  mj_forward(model, data);
  return true;
}

void FightReplay::Clear() {
  recording_ = false;
  nq_ = 0;
  timestep_ = 0.0;
  model_signature_ = 0;
  start_time_ = 0.0;
  model_blob_.clear();
  frames_.clear();
  error_.clear();
}

double FightReplay::Duration() const {
  return frames_.empty() ? 0.0 : frames_.back().time;
}
