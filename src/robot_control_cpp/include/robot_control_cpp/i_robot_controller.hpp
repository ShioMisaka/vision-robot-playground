#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

namespace robot_control {

/// 运动模式，用于 set_speed/get_speed
enum class MotionMode { kMoveJ, kMoveL };

/// 机器人运动控制抽象接口，不依赖任何 ROS 2 组件
class IRobotController {
public:
  virtual ~IRobotController() = default;

  /// 设置手臂关节角度
  virtual void set_arm(const std::vector<double>& angles,
                       bool block = true) = 0;

  /// 设置夹爪宽度
  virtual void set_gripper(double width, bool block = true) = 0;

  /// 张开夹爪
  virtual void open_gripper(bool block = true) = 0;

  /// 闭合夹爪（抓取物体）
  virtual void close_gripper(bool block = true) = 0;

  /// IK 求解并移动到指定位姿（当前 TCP 坐标系）
  virtual void move_to_pose(const std::array<double, 3>& xyz,
                            const std::optional<std::array<double, 3>>& rpy,
                            double finger = -1.0,
                            int steps = 0,
                            double step_time = 0.08,
                            bool block = true) = 0;

  /// 沿指定坐标系进行相对平移
  virtual void move_linear(const std::array<double, 3>& delta,
                           const std::string& frame = "base",
                           double finger = -1.0,
                           bool block = true) = 0;

  /// 旋转指定关节
  virtual void rotate_joint(int index, double delta_angle,
                            bool block = true) = 0;

  /// 回到安全 home 位
  virtual void go_home(bool block = true) = 0;

  /// 获取当前关节角度
  virtual std::vector<double> get_joint_angles() const = 0;

  /// 获取当前 TCP 位姿
  virtual std::array<double, 6> get_end_effector_pose() const = 0;

  /// 获取夹爪宽度
  virtual double get_finger_width() const = 0;

  /// 切换 TCP 工具坐标系
  virtual void set_tcp(const std::string& name) = 0;

  /// 获取当前 TCP 名称
  virtual std::string get_current_tcp() const = 0;

  /// 查询 TF 变换（供 TaskManager 等上层模块使用）
  virtual std::optional<std::array<double, 6>> lookup_transform(
      const std::string& target_frame,
      const std::string& source_frame,
      double timeout = 1.0) = 0;

  /// 关节空间运动到目标角度（S 曲线）
  virtual void moveJ(const std::vector<double>& target_angles,
                     bool block = true) = 0;

  /// 关节空间运动到目标位姿（IK + S 曲线，适合大范围移动）
  virtual void moveJ(const std::array<double, 3>& xyz,
                     const std::optional<std::array<double, 3>>& rpy = std::nullopt,
                     double finger = -1.0, bool block = true) = 0;

  /// 笛卡尔空间直线运动到目标位姿
  virtual void moveL(const std::array<double, 3>& xyz,
                     const std::optional<std::array<double, 3>>& rpy = std::nullopt,
                     double finger = -1.0, bool block = true) = 0;

  /// 设置运动速度百分比（0-100）
  virtual void set_speed(MotionMode mode, double percent) = 0;

  /// 获取当前运动速度百分比
  virtual double get_speed(MotionMode mode) const = 0;
};

}  // namespace robot_control
