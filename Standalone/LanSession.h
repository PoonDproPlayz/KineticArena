#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class LanSession {
 public:
  enum class Role { None, Host, Client };
  enum class State { Idle, Starting, Waiting, PackageReady, Ready, Connected, Failed, Stopped };

  struct Input {
    float throttle = 0.0f;
    float turn = 0.0f;
    float weapon = 0.0f;
  };

  struct Snapshot {
    double time = 0.0;
    std::vector<double> qpos;
    std::vector<double> qvel;
  };

  LanSession();
  ~LanSession();
  LanSession(const LanSession&) = delete;
  LanSession& operator=(const LanSession&) = delete;

  bool StartHost(uint16_t port, std::vector<unsigned char> local_package,
                 const std::string& passphrase, std::string& error);
  bool StartClient(const std::string& address, uint16_t port,
                   std::vector<unsigned char> local_package,
                   const std::string& passphrase, std::string& error);
  static bool GeneratePassphrase(std::string& passphrase);
  void Stop();

  State GetState() const;
  Role GetRole() const { return role_; }
  uint16_t BoundPort() const { return bound_port_.load(); }
  std::string Error() const;
  bool TakeRemotePackage(std::vector<unsigned char>& package);
  void MarkReady();
  bool SendInput(const Input& input);
  Input LatestRemoteInput() const;
  bool SendSnapshot(double time, const double* qpos, int nq,
                    const double* qvel, int nv);
  bool TakeLatestSnapshot(Snapshot& snapshot);

 private:
  struct Crypto;
  void Worker(std::string address, uint16_t port,
              std::vector<unsigned char> local_package,
              std::string passphrase);
  bool SendFrame(const std::vector<unsigned char>& plaintext);
  bool ReceiveFrame(std::vector<unsigned char>& plaintext);
  void SetState(State state, const std::string& error = {});
  void CloseSockets();

  mutable std::mutex state_mutex_;
  std::condition_variable ready_cv_;
  State state_ = State::Idle;
  std::string error_;
  Role role_ = Role::None;
  std::atomic<bool> stopping_{false};
  std::atomic<uint16_t> bound_port_{0};
  bool ready_requested_ = false;
  std::vector<unsigned char> remote_package_;
  Input remote_input_;
  std::chrono::steady_clock::time_point remote_input_time_{};
  Snapshot snapshot_;
  bool snapshot_available_ = false;
  std::thread worker_;
  mutable std::mutex socket_mutex_;
  uintptr_t listen_socket_ = ~uintptr_t(0);
  uintptr_t peer_socket_ = ~uintptr_t(0);
  std::mutex send_mutex_;
  std::unique_ptr<Crypto> crypto_;
};
