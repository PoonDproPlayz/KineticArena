#include "LanSession.h"

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <bcrypt.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace {
constexpr uintptr_t invalid_socket_value = ~uintptr_t(0);
constexpr uint64_t max_package_bytes = 512ull * 1024ull * 1024ull;
constexpr uint32_t max_wire_frame = uint32_t(max_package_bytes + 64);
constexpr unsigned char message_bundle = 1;
constexpr unsigned char message_ready = 2;
constexpr unsigned char message_input = 3;
constexpr unsigned char message_snapshot = 4;

void Wipe(void* memory, size_t size) {
  volatile unsigned char* bytes = static_cast<volatile unsigned char*>(memory);
  for (size_t i = 0; i < size; ++i) bytes[i] = 0;
}

bool SendAll(SOCKET socket, const unsigned char* data, size_t size) {
  while (size) {
    const int chunk = int(std::min<size_t>(size, size_t(std::numeric_limits<int>::max())));
    const int sent = send(socket, reinterpret_cast<const char*>(data), chunk, 0);
    if (sent <= 0) return false;
    data += sent;
    size -= size_t(sent);
  }
  return true;
}

bool ReceiveAll(SOCKET socket, unsigned char* data, size_t size) {
  while (size) {
    const int chunk = int(std::min<size_t>(size, size_t(std::numeric_limits<int>::max())));
    const int received = recv(socket, reinterpret_cast<char*>(data), chunk, 0);
    if (received <= 0) return false;
    data += received;
    size -= size_t(received);
  }
  return true;
}

void AppendU32(std::vector<unsigned char>& bytes, uint32_t value) {
  for (int i = 0; i < 4; ++i) bytes.push_back(static_cast<unsigned char>(value >> (8 * i)));
}

void AppendU64(std::vector<unsigned char>& bytes, uint64_t value) {
  for (int i = 0; i < 8; ++i) bytes.push_back(static_cast<unsigned char>(value >> (8 * i)));
}

uint32_t ReadU32(const unsigned char* bytes) {
  return uint32_t(bytes[0]) | (uint32_t(bytes[1]) << 8) |
    (uint32_t(bytes[2]) << 16) | (uint32_t(bytes[3]) << 24);
}

uint64_t ReadU64(const unsigned char* bytes) {
  uint64_t value = 0;
  for (int i = 0; i < 8; ++i) value |= uint64_t(bytes[i]) << (8 * i);
  return value;
}

void AppendDouble(std::vector<unsigned char>& bytes, double value) {
  unsigned char raw[sizeof(value)];
  std::memcpy(raw, &value, sizeof(value));
  bytes.insert(bytes.end(), raw, raw + sizeof(value));
}

double ReadDouble(const unsigned char* bytes) {
  double value = 0.0;
  std::memcpy(&value, bytes, sizeof(value));
  return value;
}

std::vector<unsigned char> Message(unsigned char type,
                                  const std::vector<unsigned char>& payload = {}) {
  std::vector<unsigned char> message;
  message.reserve(payload.size() + 1);
  message.push_back(type);
  message.insert(message.end(), payload.begin(), payload.end());
  return message;
}
}  // namespace

struct LanSession::Crypto {
  BCRYPT_ALG_HANDLE aes = nullptr;
  BCRYPT_ALG_HANDLE sha = nullptr;
  BCRYPT_KEY_HANDLE key = nullptr;
  std::vector<unsigned char> key_object;
  std::array<unsigned char, 8> send_prefix{};
  std::array<unsigned char, 8> receive_prefix{};
  uint32_t send_sequence = 0;
  uint32_t receive_sequence = 0;
  bool have_receive_prefix = false;

  ~Crypto() {
    if (key) BCryptDestroyKey(key);
    Wipe(key_object.data(), key_object.size());
    key_object.clear();
    if (aes) BCryptCloseAlgorithmProvider(aes, 0);
    if (sha) BCryptCloseAlgorithmProvider(sha, 0);
    Wipe(send_prefix.data(), send_prefix.size());
    Wipe(receive_prefix.data(), receive_prefix.size());
  }

  bool Initialize(const std::string& passphrase, std::string& error) {
    if (passphrase.size() < 16 || passphrase.size() > 256) {
      error = "Use a session key between 16 and 256 characters.";
      return false;
    }
    NTSTATUS status = BCryptOpenAlgorithmProvider(&sha, BCRYPT_SHA256_ALGORITHM,
                                                   nullptr, 0);
    if (status < 0) { error = "Windows could not initialize SHA-256."; return false; }
    ULONG object_size = 0, returned = 0;
    status = BCryptGetProperty(sha, BCRYPT_OBJECT_LENGTH,
      reinterpret_cast<PUCHAR>(&object_size), sizeof(object_size), &returned, 0);
    if (status < 0 || object_size == 0) {
      error = "Windows could not query SHA-256."; return false;
    }
    std::vector<unsigned char> hash_object(object_size);
    BCRYPT_HASH_HANDLE hash = nullptr;
    status = BCryptCreateHash(sha, &hash, hash_object.data(), object_size,
                              nullptr, 0, 0);
    std::array<unsigned char, 32> digest{};
    if (status >= 0)
      status = BCryptHashData(hash,
        reinterpret_cast<PUCHAR>(const_cast<char*>(passphrase.data())),
        ULONG(passphrase.size()), 0);
    if (status >= 0)
      status = BCryptFinishHash(hash, digest.data(), ULONG(digest.size()), 0);
    if (hash) BCryptDestroyHash(hash);
    Wipe(hash_object.data(), hash_object.size());
    if (status < 0) {
      Wipe(digest.data(), digest.size());
      error = "Windows could not derive the session encryption key.";
      return false;
    }

    status = BCryptOpenAlgorithmProvider(&aes, BCRYPT_AES_ALGORITHM, nullptr, 0);
    if (status >= 0)
      status = BCryptSetProperty(aes, BCRYPT_CHAINING_MODE,
        reinterpret_cast<PUCHAR>(const_cast<wchar_t*>(BCRYPT_CHAIN_MODE_GCM)),
        ULONG(sizeof(BCRYPT_CHAIN_MODE_GCM)), 0);
    if (status >= 0) {
      status = BCryptGetProperty(aes, BCRYPT_OBJECT_LENGTH,
        reinterpret_cast<PUCHAR>(&object_size), sizeof(object_size), &returned, 0);
    }
    key_object.resize(object_size);
    if (status >= 0)
      status = BCryptGenerateSymmetricKey(aes, &key, key_object.data(), object_size,
                                          digest.data(), ULONG(digest.size()), 0);
    Wipe(digest.data(), digest.size());
    if (status < 0) { error = "Windows could not initialize AES-GCM."; return false; }
    status = BCryptGenRandom(nullptr, send_prefix.data(), ULONG(send_prefix.size()),
                             BCRYPT_USE_SYSTEM_PREFERRED_RNG);
    if (status < 0) { error = "Windows could not create a unique session nonce."; return false; }
    return true;
  }

  bool Encrypt(const std::vector<unsigned char>& plaintext,
               std::vector<unsigned char>& sealed) {
    if (!key || plaintext.size() > ULONG_MAX || send_sequence == UINT32_MAX) return false;
    std::array<unsigned char, 12> nonce{};
    std::copy(send_prefix.begin(), send_prefix.end(), nonce.begin());
    const uint32_t sequence = ++send_sequence;
    for (int i = 0; i < 4; ++i) nonce[8 + i] = static_cast<unsigned char>(sequence >> (8 * i));
    std::vector<unsigned char> ciphertext(plaintext.size());
    std::array<unsigned char, 16> tag{};
    BCRYPT_AUTHENTICATED_CIPHER_MODE_INFO auth;
    BCRYPT_INIT_AUTH_MODE_INFO(auth);
    auth.pbNonce = nonce.data();
    auth.cbNonce = ULONG(nonce.size());
    auth.pbTag = tag.data();
    auth.cbTag = ULONG(tag.size());
    ULONG written = 0;
    const NTSTATUS status = BCryptEncrypt(key,
      const_cast<PUCHAR>(plaintext.data()), ULONG(plaintext.size()), &auth, nullptr, 0,
      ciphertext.data(), ULONG(ciphertext.size()), &written, 0);
    if (status < 0 || written != ciphertext.size()) return false;
    sealed.clear();
    sealed.reserve(nonce.size() + tag.size() + ciphertext.size());
    sealed.insert(sealed.end(), nonce.begin(), nonce.end());
    sealed.insert(sealed.end(), tag.begin(), tag.end());
    sealed.insert(sealed.end(), ciphertext.begin(), ciphertext.end());
    Wipe(ciphertext.data(), ciphertext.size());
    Wipe(tag.data(), tag.size());
    return true;
  }

  bool Decrypt(const std::vector<unsigned char>& sealed,
               std::vector<unsigned char>& plaintext) {
    constexpr size_t nonce_size = 12, tag_size = 16;
    if (!key || sealed.size() < nonce_size + tag_size ||
        sealed.size() - nonce_size - tag_size > ULONG_MAX) return false;
    const auto* nonce = sealed.data();
    const uint32_t sequence = ReadU32(nonce + 8);
    if (sequence != receive_sequence + 1 || sequence == 0) return false;
    if (have_receive_prefix && !std::equal(receive_prefix.begin(), receive_prefix.end(), nonce))
      return false;
    std::array<unsigned char, 16> tag{};
    std::copy(sealed.begin() + nonce_size, sealed.begin() + nonce_size + tag_size, tag.begin());
    const size_t cipher_size = sealed.size() - nonce_size - tag_size;
    plaintext.resize(cipher_size);
    BCRYPT_AUTHENTICATED_CIPHER_MODE_INFO auth;
    BCRYPT_INIT_AUTH_MODE_INFO(auth);
    auth.pbNonce = const_cast<PUCHAR>(nonce);
    auth.cbNonce = ULONG(nonce_size);
    auth.pbTag = tag.data();
    auth.cbTag = ULONG(tag.size());
    ULONG written = 0;
    const NTSTATUS status = BCryptDecrypt(key,
      const_cast<PUCHAR>(sealed.data() + nonce_size + tag_size), ULONG(cipher_size),
      &auth, nullptr, 0, plaintext.data(), ULONG(plaintext.size()), &written, 0);
    Wipe(tag.data(), tag.size());
    if (status < 0 || written != cipher_size) {
      Wipe(plaintext.data(), plaintext.size());
      plaintext.clear();
      return false;
    }
    if (!have_receive_prefix) {
      std::copy(nonce, nonce + receive_prefix.size(), receive_prefix.begin());
      have_receive_prefix = true;
    }
    receive_sequence = sequence;
    return true;
  }
};

LanSession::LanSession() = default;
LanSession::~LanSession() { Stop(); }

bool LanSession::GeneratePassphrase(std::string& passphrase) {
  std::array<unsigned char, 16> random{};
  if (BCryptGenRandom(nullptr, random.data(), ULONG(random.size()),
                      BCRYPT_USE_SYSTEM_PREFERRED_RNG) < 0) return false;
  constexpr char hex[] = "0123456789abcdef";
  passphrase.clear();
  passphrase.reserve(random.size() * 2);
  for (unsigned char value : random) {
    passphrase.push_back(hex[value >> 4]);
    passphrase.push_back(hex[value & 15]);
  }
  Wipe(random.data(), random.size());
  return true;
}

bool LanSession::StartHost(uint16_t port, std::vector<unsigned char> local_package,
                           const std::string& passphrase, std::string& error) {
  Stop();
  if (local_package.empty() || local_package.size() > max_package_bytes) {
    error = "Load a bot bundle smaller than 512 MiB before hosting.";
    return false;
  }
  WSADATA wsa{};
  if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
    error = "Windows networking could not start.";
    return false;
  }
  stopping_ = false;
  bound_port_ = 0;
  ready_requested_ = false;
  remote_package_.clear();
  snapshot_available_ = false;
  role_ = Role::Host;
  SetState(State::Starting);
  try {
    worker_ = std::thread(&LanSession::Worker, this, std::string(), port,
                          std::move(local_package), passphrase);
    return true;
  } catch (const std::exception& exception) {
    WSACleanup();
    role_ = Role::None;
    error = exception.what();
    SetState(State::Failed, error);
    return false;
  }
}

bool LanSession::StartClient(const std::string& address, uint16_t port,
                             std::vector<unsigned char> local_package,
                             const std::string& passphrase, std::string& error) {
  Stop();
  if (address.empty() || local_package.empty() || local_package.size() > max_package_bytes) {
    error = "Enter the host address and load a bot bundle smaller than 512 MiB.";
    return false;
  }
  WSADATA wsa{};
  if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
    error = "Windows networking could not start.";
    return false;
  }
  stopping_ = false;
  bound_port_ = port;
  ready_requested_ = false;
  remote_package_.clear();
  snapshot_available_ = false;
  role_ = Role::Client;
  SetState(State::Starting);
  try {
    worker_ = std::thread(&LanSession::Worker, this, address, port,
                          std::move(local_package), passphrase);
    return true;
  } catch (const std::exception& exception) {
    WSACleanup();
    role_ = Role::None;
    error = exception.what();
    SetState(State::Failed, error);
    return false;
  }
}

void LanSession::SetState(State state, const std::string& error) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  state_ = state;
  if (!error.empty()) error_ = error;
  ready_cv_.notify_all();
}

LanSession::State LanSession::GetState() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_;
}

std::string LanSession::Error() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return error_;
}

bool LanSession::TakeRemotePackage(std::vector<unsigned char>& package) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (state_ != State::PackageReady || remote_package_.empty()) return false;
  package = std::move(remote_package_);
  return true;
}

void LanSession::MarkReady() {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (state_ != State::PackageReady) return;
  ready_requested_ = true;
  state_ = State::Ready;
  ready_cv_.notify_all();
}

LanSession::Input LanSession::LatestRemoteInput() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (remote_input_time_ == std::chrono::steady_clock::time_point{} ||
      std::chrono::steady_clock::now() - remote_input_time_ >
        std::chrono::milliseconds(250)) return {};
  return remote_input_;
}

bool LanSession::TakeLatestSnapshot(Snapshot& snapshot) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!snapshot_available_) return false;
  snapshot = std::move(snapshot_);
  snapshot_ = {};
  snapshot_available_ = false;
  return true;
}

void LanSession::CloseSockets() {
  SOCKET listen_socket = INVALID_SOCKET;
  SOCKET peer_socket = INVALID_SOCKET;
  {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (listen_socket_ != invalid_socket_value) {
      listen_socket = SOCKET(listen_socket_);
      listen_socket_ = invalid_socket_value;
    }
    if (peer_socket_ != invalid_socket_value) {
      peer_socket = SOCKET(peer_socket_);
      peer_socket_ = invalid_socket_value;
    }
  }
  if (listen_socket != INVALID_SOCKET) {
    shutdown(listen_socket, SD_BOTH);
    closesocket(listen_socket);
  }
  if (peer_socket != INVALID_SOCKET) {
    shutdown(peer_socket, SD_BOTH);
    closesocket(peer_socket);
  }
}

void LanSession::Stop() {
  stopping_ = true;
  ready_cv_.notify_all();
  CloseSockets();
  if (worker_.joinable()) worker_.join();
  {
    std::lock_guard<std::mutex> lock(send_mutex_);
    crypto_.reset();
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    Wipe(remote_package_.data(), remote_package_.size());
    remote_package_.clear();
    remote_input_ = {};
    remote_input_time_ = {};
    snapshot_ = {};
    snapshot_available_ = false;
    ready_requested_ = false;
    error_.clear();
    state_ = State::Stopped;
  }
  bound_port_ = 0;
  role_ = Role::None;
}

bool LanSession::SendFrame(const std::vector<unsigned char>& plaintext) {
  if (plaintext.empty() || plaintext.size() > max_package_bytes + 1) return false;
  std::lock_guard<std::mutex> send_lock(send_mutex_);
  if (!crypto_) return false;
  std::vector<unsigned char> encrypted;
  if (!crypto_->Encrypt(plaintext, encrypted) || encrypted.size() > max_wire_frame) return false;
  SOCKET socket = INVALID_SOCKET;
  {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (peer_socket_ != invalid_socket_value) socket = SOCKET(peer_socket_);
  }
  if (socket == INVALID_SOCKET) return false;
  const uint32_t length = uint32_t(encrypted.size());
  const unsigned char header[4] = {
    static_cast<unsigned char>(length >> 24), static_cast<unsigned char>(length >> 16),
    static_cast<unsigned char>(length >> 8), static_cast<unsigned char>(length)};
  return SendAll(socket, header, sizeof(header)) &&
         SendAll(socket, encrypted.data(), encrypted.size());
}

bool LanSession::ReceiveFrame(std::vector<unsigned char>& plaintext) {
  SOCKET socket = INVALID_SOCKET;
  {
    std::lock_guard<std::mutex> lock(socket_mutex_);
    if (peer_socket_ != invalid_socket_value) socket = SOCKET(peer_socket_);
  }
  if (socket == INVALID_SOCKET || !crypto_) return false;
  unsigned char header[4]{};
  if (!ReceiveAll(socket, header, sizeof(header))) return false;
  const uint32_t length = (uint32_t(header[0]) << 24) | (uint32_t(header[1]) << 16) |
    (uint32_t(header[2]) << 8) | uint32_t(header[3]);
  if (length < 28 || length > max_wire_frame) return false;
  std::vector<unsigned char> encrypted(length);
  if (!ReceiveAll(socket, encrypted.data(), encrypted.size())) return false;
  return crypto_->Decrypt(encrypted, plaintext);
}

bool LanSession::SendInput(const Input& input) {
  if (GetState() != State::Connected) return false;
  std::vector<unsigned char> payload;
  payload.reserve(1 + sizeof(float) * 3);
  payload.push_back(message_input);
  const float throttle = std::clamp(input.throttle, -1.0f, 1.0f);
  const float turn = std::clamp(input.turn, -1.0f, 1.0f);
  const float weapon = std::clamp(input.weapon, 0.0f, 1.0f);
  for (float value : {throttle, turn, weapon}) {
    unsigned char bytes[sizeof(value)];
    std::memcpy(bytes, &value, sizeof(value));
    payload.insert(payload.end(), bytes, bytes + sizeof(value));
  }
  return SendFrame(payload);
}

bool LanSession::SendSnapshot(double time, const double* qpos, int nq,
                              const double* qvel, int nv) {
  if (GetRole() != Role::Host || GetState() != State::Connected || !qpos || !qvel ||
      nq < 0 || nv < 0 || nq > 1000000 || nv > 1000000 || !std::isfinite(time))
    return false;
  const uint64_t size = 1 + sizeof(double) + 8ull +
    uint64_t(nq + nv) * sizeof(double);
  if (size > max_package_bytes) return false;
  std::vector<unsigned char> payload;
  payload.reserve(size_t(size));
  payload.push_back(message_snapshot);
  AppendDouble(payload, time);
  AppendU32(payload, uint32_t(nq));
  AppendU32(payload, uint32_t(nv));
  for (int i = 0; i < nq; ++i) AppendDouble(payload, qpos[i]);
  for (int i = 0; i < nv; ++i) AppendDouble(payload, qvel[i]);
  return SendFrame(payload);
}

void LanSession::Worker(std::string address, uint16_t port,
                        std::vector<unsigned char> local_package,
                        std::string passphrase) {
  std::string failure;
  try {
    crypto_ = std::make_unique<Crypto>();
    if (!crypto_->Initialize(passphrase, failure)) throw std::runtime_error(failure);
    Wipe(passphrase.data(), passphrase.size());

    SOCKET peer = INVALID_SOCKET;
    if (role_ == Role::Host) {
      SOCKET listener = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
      if (listener == INVALID_SOCKET) throw std::runtime_error("Could not create LAN host socket.");
      {
        std::lock_guard<std::mutex> lock(socket_mutex_);
        listen_socket_ = uintptr_t(listener);
      }
      BOOL reuse = TRUE;
      setsockopt(listener, SOL_SOCKET, SO_REUSEADDR,
                 reinterpret_cast<const char*>(&reuse), sizeof(reuse));
      sockaddr_in endpoint{};
      endpoint.sin_family = AF_INET;
      endpoint.sin_addr.s_addr = htonl(INADDR_ANY);
      endpoint.sin_port = htons(port);
      if (bind(listener, reinterpret_cast<sockaddr*>(&endpoint), sizeof(endpoint)) == SOCKET_ERROR)
        throw std::runtime_error("Could not bind the LAN port. It may already be in use or blocked.");
      if (listen(listener, 1) == SOCKET_ERROR)
        throw std::runtime_error("Could not listen for the LAN opponent.");
      int endpoint_size = sizeof(endpoint);
      if (getsockname(listener, reinterpret_cast<sockaddr*>(&endpoint), &endpoint_size) == 0)
        bound_port_ = ntohs(endpoint.sin_port);
      SetState(State::Waiting);
      peer = accept(listener, nullptr, nullptr);
      if (peer == INVALID_SOCKET) {
        if (stopping_) throw std::runtime_error("Session closed while waiting for a player.");
        throw std::runtime_error("LAN player could not connect to the host.");
      }
      {
        std::lock_guard<std::mutex> lock(socket_mutex_);
        if (listen_socket_ == uintptr_t(listener)) listen_socket_ = invalid_socket_value;
        peer_socket_ = uintptr_t(peer);
      }
      closesocket(listener);
      const BOOL no_delay = TRUE;
      setsockopt(peer, IPPROTO_TCP, TCP_NODELAY,
                 reinterpret_cast<const char*>(&no_delay), sizeof(no_delay));
      if (!SendFrame(Message(message_bundle, local_package)))
        throw std::runtime_error("Could not send the host bot bundle.");
    } else {
      addrinfo hints{};
      hints.ai_family = AF_UNSPEC;
      hints.ai_socktype = SOCK_STREAM;
      hints.ai_protocol = IPPROTO_TCP;
      addrinfo* results = nullptr;
      const std::string service = std::to_string(port);
      const int lookup = getaddrinfo(address.c_str(), service.c_str(), &hints, &results);
      if (lookup != 0) throw std::runtime_error("Host address could not be resolved.");
      for (addrinfo* result = results; result && !stopping_; result = result->ai_next) {
        peer = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
        if (peer == INVALID_SOCKET) continue;
        if (connect(peer, result->ai_addr, int(result->ai_addrlen)) == 0) break;
        closesocket(peer);
        peer = INVALID_SOCKET;
      }
      freeaddrinfo(results);
      if (peer == INVALID_SOCKET) {
        if (stopping_) throw std::runtime_error("Session closed while connecting.");
        throw std::runtime_error("Could not connect to host. Check its LAN address, port, and firewall.");
      }
      {
        std::lock_guard<std::mutex> lock(socket_mutex_);
        peer_socket_ = uintptr_t(peer);
      }
      const BOOL no_delay = TRUE;
      setsockopt(peer, IPPROTO_TCP, TCP_NODELAY,
                 reinterpret_cast<const char*>(&no_delay), sizeof(no_delay));
    }

    std::vector<unsigned char> message;
    if (role_ == Role::Client) {
      if (!ReceiveFrame(message) || message.empty() || message[0] != message_bundle ||
          message.size() > max_package_bytes + 1)
        throw std::runtime_error("Host bundle transfer failed authentication or validation.");
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        remote_package_.assign(message.begin() + 1, message.end());
      }
      if (remote_package_.empty()) throw std::runtime_error("Host sent an empty bot bundle.");
      if (!SendFrame(Message(message_bundle, local_package)))
        throw std::runtime_error("Could not send the joining player's bot bundle.");
    } else {
      if (!ReceiveFrame(message) || message.empty() || message[0] != message_bundle ||
          message.size() > max_package_bytes + 1)
        throw std::runtime_error("Joining player bundle transfer failed authentication or validation.");
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        remote_package_.assign(message.begin() + 1, message.end());
      }
      if (remote_package_.empty()) throw std::runtime_error("Joining player sent an empty bot bundle.");
    }
    Wipe(message.data(), message.size());
    Wipe(local_package.data(), local_package.size());
    local_package.clear();
    SetState(State::PackageReady);

    {
      std::unique_lock<std::mutex> lock(state_mutex_);
      ready_cv_.wait(lock, [&] { return ready_requested_ || stopping_.load(); });
    }
    if (stopping_) throw std::runtime_error("Session closed before both models were ready.");
    if (!SendFrame(Message(message_ready)))
      throw std::runtime_error("Could not signal LAN model readiness.");
    if (!ReceiveFrame(message) || message.size() != 1 || message[0] != message_ready)
      throw std::runtime_error("Opponent did not confirm the same MuJoCo model is ready.");
    SetState(State::Connected);
    Wipe(message.data(), message.size());

    while (!stopping_) {
      if (!ReceiveFrame(message)) {
        if (!stopping_) throw std::runtime_error("LAN connection lost.");
        break;
      }
      if (message.empty()) throw std::runtime_error("Received an empty LAN message.");
      if (role_ == Role::Host && message[0] == message_input) {
        if (message.size() != 1 + 3 * sizeof(float))
          throw std::runtime_error("Received an invalid controller input packet.");
        float values[3]{};
        std::memcpy(values, message.data() + 1, sizeof(values));
        if (!std::isfinite(values[0]) || !std::isfinite(values[1]) ||
            !std::isfinite(values[2]))
          throw std::runtime_error("Received non-finite controller values.");
        std::lock_guard<std::mutex> lock(state_mutex_);
        remote_input_ = {std::clamp(values[0], -1.0f, 1.0f),
                         std::clamp(values[1], -1.0f, 1.0f),
                         std::clamp(values[2], 0.0f, 1.0f)};
        remote_input_time_ = std::chrono::steady_clock::now();
      } else if (role_ == Role::Client && message[0] == message_snapshot) {
        if (message.size() < 1 + sizeof(double) + 8)
          throw std::runtime_error("Received a truncated physics snapshot.");
        const double time = ReadDouble(message.data() + 1);
        const uint32_t nq = ReadU32(message.data() + 1 + sizeof(double));
        const uint32_t nv = ReadU32(message.data() + 1 + sizeof(double) + 4);
        const uint64_t expected = 1 + sizeof(double) + 8ull +
          (uint64_t(nq) + nv) * sizeof(double);
        if (!std::isfinite(time) || nq > 1000000 || nv > 1000000 ||
            expected != message.size())
          throw std::runtime_error("Received an invalid physics snapshot.");
        Snapshot next;
        next.time = time;
        next.qpos.resize(nq);
        next.qvel.resize(nv);
        size_t offset = 1 + sizeof(double) + 8;
        for (uint32_t i = 0; i < nq; ++i, offset += sizeof(double))
          next.qpos[i] = ReadDouble(message.data() + offset);
        for (uint32_t i = 0; i < nv; ++i, offset += sizeof(double))
          next.qvel[i] = ReadDouble(message.data() + offset);
        if (!std::all_of(next.qpos.begin(), next.qpos.end(),
                         [](double value) { return std::isfinite(value); }) ||
            !std::all_of(next.qvel.begin(), next.qvel.end(),
                         [](double value) { return std::isfinite(value); }))
          throw std::runtime_error("Received non-finite MuJoCo state.");
        std::lock_guard<std::mutex> lock(state_mutex_);
        snapshot_ = std::move(next);
        snapshot_available_ = true;
      } else {
        throw std::runtime_error("Received an unexpected LAN message for this peer role.");
      }
      Wipe(message.data(), message.size());
      message.clear();
    }
  } catch (const std::exception& exception) {
    if (!stopping_) {
      failure = exception.what();
      SetState(State::Failed, failure);
    }
  }
  Wipe(local_package.data(), local_package.size());
  Wipe(passphrase.data(), passphrase.size());
  CloseSockets();
  WSACleanup();
  if (GetState() != State::Failed) SetState(State::Stopped);
}
