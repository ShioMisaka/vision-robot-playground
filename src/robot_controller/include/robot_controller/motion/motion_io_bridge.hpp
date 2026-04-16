#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace robot_control {

/// IO 桥接接口 — 将运动控制逻辑与底层通信实现解耦
/// ROS 2 节点实现此接口以提供实际的发布/订阅能力
class MotionIOBridge {
public:
  virtual ~MotionIOBridge() = default;

  /// 发布关节指令（arm + gripper）
  virtual void publish_command(const std::vector<double>& arm,
                               double finger) = 0;

  /// 获取当前关节角度反馈
  virtual std::vector<double> get_current_arm() const = 0;

  /// 获取当前夹爪宽度反馈
  virtual double get_current_finger() const = 0;

  /// 阻塞等待运动到位
  virtual bool wait_for_motion(const std::vector<double>& target_arm,
                               double finger,
                               double joint_tol, double finger_tol,
                               double timeout, double poll_interval,
                               double settle_time,
                               bool check_finger) = 0;

  /// 阻塞等待夹爪稳定（用于抓取物体时）
  virtual bool wait_for_finger_settle(int stable_count, double tol,
                                      double poll_interval,
                                      double timeout) = 0;

  /// 查询 TF 变换
  virtual std::optional<std::array<double, 6>> lookup_transform(
      const std::string& target_frame,
      const std::string& source_frame,
      double timeout) = 0;

  /// 设置当前 TCP 名称
  virtual void set_tcp_name(const std::string& name) = 0;
};

}  // namespace robot_control
