#include <gtest/gtest.h>
#include <thread>
#include <chrono>
#include "robot_controller/nodes/lease_manager.hpp"

using robot_control::LeaseManager;

TEST(LeaseManagerTest, AcquireAndRelease) {
  LeaseManager lm(std::chrono::seconds(10));
  auto result = lm.acquire("pendant", 0);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(lm.active_session_id(), result.value());
  EXPECT_EQ(lm.active_client_name(), "pendant");
  EXPECT_TRUE(lm.release(result.value()));
  EXPECT_EQ(lm.active_session_id(), "");
}

TEST(LeaseManagerTest, DoubleAcquireFails) {
  LeaseManager lm(std::chrono::seconds(10));
  auto s1 = lm.acquire("pendant", 0);
  ASSERT_TRUE(s1.has_value());
  auto s2 = lm.acquire("python", 0);
  EXPECT_FALSE(s2.has_value());
}

TEST(LeaseManagerTest, ReleaseAllowsNewAcquire) {
  LeaseManager lm(std::chrono::seconds(10));
  auto s1 = lm.acquire("pendant", 0);
  lm.release(s1.value());
  auto s2 = lm.acquire("python", 0);
  ASSERT_TRUE(s2.has_value());
}

TEST(LeaseManagerTest, RenewExtendsLease) {
  LeaseManager lm(std::chrono::seconds(1));
  auto s = lm.acquire("pendant", 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(800));
  EXPECT_TRUE(lm.renew(s.value(), 0));
}

TEST(LeaseManagerTest, ExpiredLeaseReleased) {
  LeaseManager lm(std::chrono::seconds(1));
  auto s = lm.acquire("pendant", 0.1);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  lm.check_expiry();
  EXPECT_EQ(lm.active_session_id(), "");
}

TEST(LeaseManagerTest, ValidateSession) {
  LeaseManager lm(std::chrono::seconds(10));
  EXPECT_FALSE(lm.is_valid_session("nonexistent"));
  auto s = lm.acquire("pendant", 0);
  EXPECT_TRUE(lm.is_valid_session(s.value()));
  EXPECT_FALSE(lm.is_valid_session("wrong_id"));
}

TEST(LeaseManagerTest, TeachingModeRequiresSession) {
  LeaseManager lm(std::chrono::seconds(10));
  EXPECT_FALSE(lm.request_teaching_mode("nonexistent"));
  auto s = lm.acquire("pendant", 0);
  EXPECT_TRUE(lm.request_teaching_mode(s.value()));
  EXPECT_TRUE(lm.teaching_mode_active());
}

TEST(LeaseManagerTest, TeachingModeExclusive) {
  LeaseManager lm(std::chrono::seconds(10));
  auto s1 = lm.acquire("pendant", 0);
  lm.request_teaching_mode(s1.value());
  lm.release(s1.value());
  EXPECT_FALSE(lm.teaching_mode_active());
  auto s2 = lm.acquire("python", 0);
  EXPECT_TRUE(lm.request_teaching_mode(s2.value()));
}
