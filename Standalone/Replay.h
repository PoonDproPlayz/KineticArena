#pragma once

#include <mujoco/mujoco.h>

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

// Stores sampled MuJoCo poses for lightweight, portable fight review.
class FightReplay {
 public:
  bool Begin(const mjModel* model, const mjData* data, std::string& error);
  void Stop() { recording_ = false; }
  bool Capture(const mjModel* model, const mjData* data, std::string& error);
  bool Save(const std::filesystem::path& path, std::string& error) const;
  bool Load(const std::filesystem::path& path, std::string& error);
  mjModel* CreateModel(std::string& error) const;
  bool Apply(const mjModel* model, mjData* data, double elapsed_seconds,
             std::string& error) const;
  bool BeginResimulation(const mjModel* model, mjData* data, std::string& error);
  bool AdvanceResimulation(const mjModel* model, mjData* data, size_t max_steps,
                           std::string& error);

  void Clear();
  bool IsRecording() const { return recording_; }
  bool HasReplay() const { return !frames_.empty(); }
  size_t FrameCount() const { return frames_.size(); }
  double Duration() const;
  double SampleRate() const { return sample_rate_hz_; }
  bool CanResimulate() const { return physics_steps_ > 0 && !initial_state_.empty(); }
  bool IsResimulating() const { return resimulating_; }
  double ResimulationProgress() const;
  bool IsResimulated() const { return resimulation_ready_; }
  const std::string& Error() const { return error_; }

 private:
  struct Frame {
    double time = 0.0;
    std::vector<float> qpos;
  };

  static constexpr double sample_rate_hz_ = 120.0;
  static constexpr size_t max_frame_count_ = 72000;
  static constexpr size_t max_physics_step_count_ = 1500000;

  bool recording_ = false;
  int nq_ = 0;
  int nv_ = 0;
  int nu_ = 0;
  int state_size_ = 0;
  double timestep_ = 0.0;
  uint64_t model_signature_ = 0;
  double start_time_ = 0.0;
  std::vector<unsigned char> model_blob_;
  std::vector<double> initial_state_;
  std::vector<double> controls_;
  size_t physics_steps_ = 0;
  std::vector<Frame> frames_;
  std::vector<Frame> resimulation_frames_;
  size_t resimulation_step_ = 0;
  double resimulation_next_sample_ = 0.0;
  bool resimulating_ = false;
  bool resimulation_ready_ = false;
  mutable std::string error_;
};
