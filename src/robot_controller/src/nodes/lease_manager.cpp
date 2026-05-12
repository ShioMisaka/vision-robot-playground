#include "robot_controller/nodes/lease_manager.hpp"
#include "robot_logger/logger.hpp"

namespace robot_control {

LeaseManager::LeaseManager(std::chrono::seconds default_duration)
    : default_duration_(default_duration) {}

std::string LeaseManager::generate_session_id() {
  return std::to_string(next_id_++);
}

std::optional<std::string> LeaseManager::acquire(
    const std::string& client_name, double duration_sec) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (active_lease_.has_value()) {
    LOG_WARN("Lease acquire rejected: held by '{}'", active_lease_->client_name);
    return std::nullopt;
  }
  auto duration = duration_sec > 0
      ? std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::duration<double>(duration_sec))
      : default_duration_;
  LeaseInfo info{
      generate_session_id(),
      client_name,
      std::chrono::steady_clock::now() + duration,
  };
  active_lease_ = info;
  LOG_INFO("Lease acquired: session={} client={} duration={}s",
           info.session_id, info.client_name, duration.count());
  return info.session_id;
}

bool LeaseManager::release(const std::string& session_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_lease_.has_value() || active_lease_->session_id != session_id) {
    return false;
  }
  LOG_INFO("Lease released: session={} client={}",
           active_lease_->session_id, active_lease_->client_name);
  teaching_mode_ = false;
  active_lease_.reset();
  return true;
}

bool LeaseManager::renew(const std::string& session_id, double extension_sec) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_lease_.has_value() || active_lease_->session_id != session_id) {
    return false;
  }
  auto extension = extension_sec > 0
      ? std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::duration<double>(extension_sec))
      : default_duration_;
  active_lease_->expiry = std::chrono::steady_clock::now() + extension;
  return true;
}

void LeaseManager::check_expiry() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (active_lease_.has_value() &&
      std::chrono::steady_clock::now() > active_lease_->expiry) {
    LOG_WARN("Lease expired: session={} client={}",
             active_lease_->session_id, active_lease_->client_name);
    teaching_mode_ = false;
    active_lease_.reset();
  }
}

bool LeaseManager::is_valid_session(const std::string& session_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return active_lease_.has_value() && active_lease_->session_id == session_id;
}

bool LeaseManager::has_active_lease() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return active_lease_.has_value();
}

std::string LeaseManager::active_session_id() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return active_lease_.has_value() ? active_lease_->session_id : "";
}

std::string LeaseManager::active_client_name() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return active_lease_.has_value() ? active_lease_->client_name : "";
}

bool LeaseManager::request_teaching_mode(const std::string& session_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_lease_.has_value() || active_lease_->session_id != session_id) {
    return false;
  }
  // Idempotent: same session re-requesting is a no-op success
  teaching_mode_ = true;
  return true;
}

bool LeaseManager::teaching_mode_active() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return teaching_mode_;
}

}  // namespace robot_control
