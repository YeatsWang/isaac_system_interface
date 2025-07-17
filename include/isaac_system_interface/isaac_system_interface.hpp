#pragma once

#include <vector>
#include <string>
#include <mutex>
#include <memory>
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include "hardware_interface/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <atomic>
#include <thread>
#include "sensor_msgs/msg/joint_state.hpp"

namespace isaac_system_interface
{

class IsaacSystemInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(IsaacSystemInterface)

  IsaacSystemInterface();

  // 系统接口标准函数
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // 关节名称、接口类型
  std::vector<std::string> joint_names_;
  std::vector<std::string> command_types_;  // velocity/position
  std::vector<std::string> state_types_;    // position/velocity/effort

  // 状态与指令数据
  std::vector<double> positions_, velocities_, efforts_, commands_;

  // ROS2通信
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::atomic<bool> spinning_{false};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;

  // 线程安全
  std::mutex state_mutex_;
  sensor_msgs::msg::JointState latest_joint_state_;

  // 话题名
  std::string joint_state_topic_ = "/joint_states";
  std::string joint_command_topic_ = "/joint_command";

  // 回调
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg);
};

}  // namespace isaac_system_interface
