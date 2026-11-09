#include "Replay.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>

namespace {
constexpr std::array<char, 8> replay_magic_v2{'K', 'A', 'R', 'E', 'P', 'L', 'Y', '2'};
constexpr std::array<char, 8> replay_magic_v3{'K', 'A', 'R', 'E', 'P', 'L', 'Y', '3'};
constexpr uint32_t replay_version = 3;
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
  controls_.clear();
  initial_state_.clear();
  resimulation_frames_.clear();
  physics_steps_ = 0;
  resimulation_ready_ = false;
  resimulating_ = false;
  model_blob_.clear();
  if (!model || !data || model->nq <= 0 || !std::isfinite(data->time)) {
    error = "The current simulation state cannot be recorded.";
    return false;
  }
  nq_ = model->nq;
  nv_ = model->nv;
  nu_ = model->nu;
  state_size_ = mj_stateSize(model, mjSTATE_INTEGRATION);
  timestep_ = model->opt.timestep;
  model_signature_ = model_signature(model);
  const int model_size = mj_sizeModel(model);
  if (model_size <= 0 || uint64_t(model_size) > max_replay_file_bytes / 2) {
    error = "The active model is too large to include in a replay file.";
    return false;
  }
  model_blob_.resize(size_t(model_size));
  mj_saveModel(model, nullptr, model_blob_.data(), model_size);
  if (state_size_ <= 0) {
    error = "MuJoCo could not size the replay's initial integration state.";
    model_blob_.clear();
    return false;
  }
  std::vector<mjtNum> state;
  state.resize(static_cast<size_t>(state_size_));
  mj_getState(model, data, state.data(), mjSTATE_INTEGRATION);
  initial_state_.reserve(state.size());
  for (mjtNum value : state) initial_state_.push_back(static_cast<double>(value));
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
  if (!model || !data || model->nq != nq_ || model->nv != nv_ || model->nu != nu_) {
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
  if (elapsed >= max_recording_seconds ||
      physics_steps_ >= max_physics_step_count_ || frames_.size() >= max_frame_count_) {
    recording_ = false;
    error = "Recording reached the 10 minute limit and stopped.";
    error_ = error;
    return false;
  }
  const uint64_t projected_size = 68 + uint64_t(model_blob_.size()) +
      uint64_t(initial_state_.size()) * sizeof(double) +
      uint64_t(physics_steps_ + 1) * uint64_t(nu_) * sizeof(double) +
      uint64_t(frames_.size() + 1) * (8 + uint64_t(nq_) * sizeof(float));
  if (projected_size > max_replay_file_bytes) {
    recording_ = false;
    error = "Recording reached the replay file size limit and stopped.";
    error_ = error;
    return false;
  }

  for (int i = 0; i < nu_; ++i) {
    if (!std::isfinite(data->ctrl[i])) {
      recording_ = false;
      error = "Recording stopped because an actuator input became invalid.";
      error_ = error;
      return false;
    }
    controls_.push_back(static_cast<double>(data->ctrl[i]));
  }
  ++physics_steps_;
  const double minimum_interval = 1.0 / sample_rate_hz_;
  if (elapsed - frames_.back().time + 1e-10 >= minimum_interval) {
    Frame frame;
    frame.time = elapsed;
    frame.qpos.reserve(nq_);
    for (int i = 0; i < nq_; ++i) frame.qpos.push_back(static_cast<float>(data->qpos[i]));
    frames_.push_back(std::move(frame));
  }
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
  const uint64_t estimated_size = 68 + uint64_t(model_blob_.size()) +
      uint64_t(initial_state_.size()) * sizeof(double) +
      uint64_t(controls_.size()) * sizeof(double) +
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
  output.write(replay_magic_v3.data(), replay_magic_v3.size());
  const uint32_t nq = uint32_t(nq_);
  const uint32_t nv = uint32_t(nv_);
  const uint32_t nu = uint32_t(nu_);
  const uint32_t state_size = uint32_t(state_size_);
  const uint64_t count = uint64_t(frames_.size());
  const uint64_t physics_steps = uint64_t(physics_steps_);
  const uint64_t model_blob_size = model_blob_.size();
  if (!output || !write_value(output, replay_version) || !write_value(output, nq) ||
      !write_value(output, nv) || !write_value(output, nu) ||
      !write_value(output, state_size) || !write_value(output, count) ||
      !write_value(output, physics_steps) || !write_value(output, model_signature_) ||
      !write_value(output, timestep_) || !write_value(output, model_blob_size)) {
    error = "Could not write the replay header.";
    return false;
  }
  output.write(reinterpret_cast<const char*>(initial_state_.data()),
               std::streamsize(initial_state_.size() * sizeof(double)));
  if (!output) {
    error = "Could not write the replay's initial physics state.";
    return false;
  }
  output.write(reinterpret_cast<const char*>(model_blob_.data()),
               std::streamsize(model_blob_.size()));
  if (!output) {
    error = "Could not write the embedded simulation model.";
    return false;
  }
  output.write(reinterpret_cast<const char*>(controls_.data()),
               std::streamsize(controls_.size() * sizeof(double)));
  if (!output) {
    error = "Could not write replay actuator inputs.";
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
  uint32_t version = 0, nq = 0, nv = 0, nu = 0, state_size = 0;
  uint64_t count = 0, physics_steps = 0, signature = 0, model_blob_size = 0;
  double timestep = 0.0;
  if (!input || !read_value(input, version) || !read_value(input, nq)) {
    error = "This is not a valid KineticArena replay file.";
    return false;
  }
  const bool legacy_v2 = version == 2 && magic == replay_magic_v2;
  const bool current_v3 = version == replay_version && magic == replay_magic_v3;
  if (!legacy_v2 && !current_v3) {
    error = "This replay was made by a different, unsupported format version.";
    return false;
  }
  if (legacy_v2) {
    if (!read_value(input, count) || !read_value(input, signature) ||
        !read_value(input, timestep) || !read_value(input, model_blob_size)) {
      error = "Replay header is incomplete.";
      return false;
    }
  } else if (!read_value(input, nv) || !read_value(input, nu) ||
             !read_value(input, state_size) || !read_value(input, count) ||
             !read_value(input, physics_steps) || !read_value(input, signature) ||
             !read_value(input, timestep) || !read_value(input, model_blob_size)) {
    error = "Replay header is incomplete.";
    return false;
  }
  if (nq == 0 || model_blob_size == 0 || model_blob_size > max_replay_file_bytes / 2 ||
      !std::isfinite(timestep) || timestep <= 0.0) {
    error = "Replay has invalid model data.";
    return false;
  }
  if (count == 0 || count > max_frame_count_) {
    error = "Replay frame count is outside the supported range.";
    return false;
  }
  const uint64_t frame_bytes = 8 + uint64_t(nq) * sizeof(float);
  uint64_t expected_size = 0;
  if (legacy_v2) {
    expected_size = 48 + model_blob_size + count * frame_bytes;
  } else {
    if (nv == 0 || state_size == 0 || physics_steps > max_physics_step_count_ ||
        uint64_t(state_size) > max_replay_file_bytes / sizeof(double)) {
      error = "Replay has invalid initial state or actuator input data.";
      return false;
    }
    const uint64_t control_values = physics_steps * uint64_t(nu);
    if (control_values > max_replay_file_bytes / sizeof(double)) {
      error = "Replay actuator input trace is too large.";
      return false;
    }
    expected_size = 68 + uint64_t(state_size) * sizeof(double) + model_blob_size +
        control_values * sizeof(double) + count * frame_bytes;
  }
  if (expected_size > max_replay_file_bytes ||
      file_size != expected_size) {
    error = "Replay frame data is invalid or incomplete.";
    return false;
  }
  std::vector<double> loaded_initial_state;
  std::vector<double> loaded_controls;
  if (current_v3) {
    loaded_initial_state.resize(state_size);
    input.read(reinterpret_cast<char*>(loaded_initial_state.data()),
               std::streamsize(loaded_initial_state.size() * sizeof(double)));
    if (!input || std::any_of(loaded_initial_state.begin(), loaded_initial_state.end(),
        [](double value) { return !std::isfinite(value); })) {
      error = "Replay's initial physics state is invalid.";
      return false;
    }
  }
  std::vector<unsigned char> loaded_model(static_cast<size_t>(model_blob_size), 0);
  input.read(reinterpret_cast<char*>(loaded_model.data()),
             std::streamsize(loaded_model.size()));
  if (!input) {
    error = "Replay model data is incomplete.";
    return false;
  }
  if (current_v3) {
    const uint64_t control_values = physics_steps * uint64_t(nu);
    loaded_controls.resize(static_cast<size_t>(control_values));
    input.read(reinterpret_cast<char*>(loaded_controls.data()),
               std::streamsize(loaded_controls.size() * sizeof(double)));
    if (!input || std::any_of(loaded_controls.begin(), loaded_controls.end(),
        [](double value) { return !std::isfinite(value); })) {
      error = "Replay actuator inputs are invalid or incomplete.";
      return false;
    }
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
  initial_state_ = std::move(loaded_initial_state);
  controls_ = std::move(loaded_controls);
  physics_steps_ = static_cast<size_t>(physics_steps);
  frames_ = std::move(loaded);
  resimulation_frames_.clear();
  resimulation_step_ = 0;
  resimulating_ = false;
  resimulation_ready_ = false;
  nq_ = int(nq);
  nv_ = int(nv);
  nu_ = int(nu);
  state_size_ = int(state_size);
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
  const std::vector<Frame>& playback_frames = resimulation_ready_
    ? resimulation_frames_ : frames_;
  if (!model || !data || playback_frames.empty() || model->nq != nq_) {
    error = "The loaded replay does not match the active model.";
    return false;
  }
  const double time = std::clamp(elapsed_seconds, 0.0, Duration());
  auto upper = std::upper_bound(playback_frames.begin(), playback_frames.end(), time,
      [](double value, const Frame& frame) { return value < frame.time; });
  if (upper == playback_frames.begin()) {
    std::copy(playback_frames.front().qpos.begin(), playback_frames.front().qpos.end(), data->qpos);
    data->time = 0.0;
  } else if (upper == playback_frames.end()) {
    std::copy(playback_frames.back().qpos.begin(), playback_frames.back().qpos.end(), data->qpos);
    data->time = playback_frames.back().time;
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

bool FightReplay::BeginResimulation(const mjModel* model, mjData* data,
                                   std::string& error) {
  error.clear();
  if (!CanResimulate() || !model || !data || model->nq != nq_ ||
      model->nv != nv_ || model->nu != nu_ || model_signature(model) != model_signature_ ||
      mj_stateSize(model, mjSTATE_INTEGRATION) != state_size_ ||
      controls_.size() != physics_steps_ * size_t(nu_)) {
    error = "This replay does not contain compatible physics inputs for re-simulation.";
    return false;
  }
  std::vector<mjtNum> state(initial_state_.size());
  for (size_t i = 0; i < initial_state_.size(); ++i)
    state[i] = static_cast<mjtNum>(initial_state_[i]);
  mj_setState(model, data, state.data(), mjSTATE_INTEGRATION);
  resimulation_frames_.clear();
  Frame initial;
  initial.time = 0.0;
  initial.qpos.reserve(nq_);
  for (int i = 0; i < nq_; ++i) initial.qpos.push_back(float(data->qpos[i]));
  resimulation_frames_.push_back(std::move(initial));
  resimulation_step_ = 0;
  resimulation_next_sample_ = 1.0 / sample_rate_hz_;
  resimulation_ready_ = false;
  resimulating_ = true;
  return true;
}

bool FightReplay::AdvanceResimulation(const mjModel* model, mjData* data,
                                      size_t max_steps, std::string& error) {
  error.clear();
  if (!resimulating_) return true;
  if (!model || !data || model->nq != nq_ || model->nv != nv_ || model->nu != nu_) {
    resimulating_ = false;
    error = "Re-simulation stopped because its model changed.";
    return false;
  }
  const double initial_time = initial_state_.empty() ? 0.0 : initial_state_.front();
  const size_t stop = std::min(physics_steps_, resimulation_step_ + std::max<size_t>(1, max_steps));
  while (resimulation_step_ < stop) {
    const size_t control_offset = resimulation_step_ * size_t(nu_);
    for (int i = 0; i < nu_; ++i)
      data->ctrl[i] = static_cast<mjtNum>(controls_[control_offset + size_t(i)]);
    mj_step(model, data);
    ++resimulation_step_;
    const double elapsed = data->time - initial_time;
    if (elapsed + timestep_ * 1e-6 >= resimulation_next_sample_) {
      Frame frame;
      frame.time = elapsed;
      frame.qpos.reserve(nq_);
      for (int i = 0; i < nq_; ++i) frame.qpos.push_back(float(data->qpos[i]));
      resimulation_frames_.push_back(std::move(frame));
      resimulation_next_sample_ += 1.0 / sample_rate_hz_;
    }
  }
  if (resimulation_step_ >= physics_steps_) {
    const double elapsed = data->time - initial_time;
    if (resimulation_frames_.empty() ||
        elapsed - resimulation_frames_.back().time > timestep_ * 0.25) {
      Frame frame;
      frame.time = elapsed;
      frame.qpos.reserve(nq_);
      for (int i = 0; i < nq_; ++i) frame.qpos.push_back(float(data->qpos[i]));
      resimulation_frames_.push_back(std::move(frame));
    }
    resimulating_ = false;
    resimulation_ready_ = true;
  }
  return true;
}

double FightReplay::ResimulationProgress() const {
  if (!CanResimulate()) return 0.0;
  if (resimulation_ready_) return 1.0;
  return std::clamp(double(resimulation_step_) / double(physics_steps_), 0.0, 1.0);
}

void FightReplay::Clear() {
  recording_ = false;
  nq_ = 0;
  timestep_ = 0.0;
  model_signature_ = 0;
  nv_ = 0;
  nu_ = 0;
  state_size_ = 0;
  start_time_ = 0.0;
  model_blob_.clear();
  initial_state_.clear();
  controls_.clear();
  physics_steps_ = 0;
  frames_.clear();
  resimulation_frames_.clear();
  resimulation_step_ = 0;
  resimulation_next_sample_ = 0.0;
  resimulating_ = false;
  resimulation_ready_ = false;
  error_.clear();
}

double FightReplay::Duration() const {
  const auto& active_frames = resimulation_ready_ ? resimulation_frames_ : frames_;
  return active_frames.empty() ? 0.0 : active_frames.back().time;
}
