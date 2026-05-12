#pragma once
#include <chrono>
#include <optional>
#include <string>
#include <mutex>

namespace robot_control {

struct LeaseInfo {
  std::string session_id;
  std::string client_name;
  std::chrono::steady_clock::time_point expiry;
};

class LeaseManager {
public:
  explicit LeaseManager(std::chrono::seconds default_duration);

  std::optional<std::string> acquire(const std::string& client_name, double duration_sec);
  bool release(const std::string& session_id);
  bool renew(const std::string& session_id, double extension_sec);
  void check_expiry();

  bool is_valid_session(const std::string& session_id) const;
  bool has_active_lease() const;
  std::string active_session_id() const;
  std::string active_client_name() const;

  bool request_teaching_mode(const std::string& session_id);
  bool teaching_mode_active() const;

private:
  std::chrono::seconds default_duration_;
  mutable std::mutex mutex_;
  std::optional<LeaseInfo> active_lease_;
  bool teaching_mode_{false};
  uint64_t next_id_{1};
  std::string generate_session_id();
};

}  // namespace robot_control
