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

  void Clear();
  bool IsRecording() const { return recording_; }
  bool HasReplay() const { return !frames_.empty(); }
  size_t FrameCount() const { return frames_.size(); }
  double Duration() const;
  double SampleRate() const { return sample_rate_hz_; }
  const std::string& Error() const { return error_; }

 private:
  struct Frame {
    double time = 0.0;
    std::vector<float> qpos;
  };

  static constexpr double sample_rate_hz_ = 120.0;
  static constexpr size_t max_frame_count_ = 72000;

  bool recording_ = false;
  int nq_ = 0;
  double timestep_ = 0.0;
  uint64_t model_signature_ = 0;
  double start_time_ = 0.0;
  std::vector<unsigned char> model_blob_;
  std::vector<Frame> frames_;
  mutable std::string error_;
};
