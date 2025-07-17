#include "isaac_system_interface/isaac_system_interface.hpp"
#include <algorithm>

namespace isaac_system_interface
{

IsaacSystemInterface::IsaacSystemInterface()
{
}

hardware_interface::CallbackReturn IsaacSystemInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  // 读取 joint/command/state 定义
  joint_names_.clear();
  command_types_.clear();
  state_types_.clear();
  for (const auto & joint : info.joints) {
    joint_names_.push_back(joint.name);

    // command_interface 只取第一个（多接口可扩展）
    if (!joint.command_interfaces.empty())
      command_types_.push_back(joint.command_interfaces[0].name);

    // state_interface 只取前2个
    for (const auto & si : joint.state_interfaces)
      state_types_.push_back(si.name);
  }

  size_t n = joint_names_.size();
  positions_.resize(n, 0.0);
  velocities_.resize(n, 0.0);
  efforts_.resize(n, 0.0);
  commands_.resize(n, 0.0);

  // 参数可选
  if (info.hardware_parameters.count("joint_state_topic"))
    joint_state_topic_ = info.hardware_parameters.at("joint_state_topic");
  if (info.hardware_parameters.count("joint_command_topic"))
    joint_command_topic_ = info.hardware_parameters.at("joint_command_topic");

  // 初始化节点（HardwareInterface推荐做法）
  node_ = rclcpp::Node::make_shared("isaac_system_interface_node");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IsaacSystemInterface::on_activate(const rclcpp_lifecycle::State &)
{
  // 订阅 joint_states
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, 10,
      std::bind(&IsaacSystemInterface::on_joint_state, this, std::placeholders::_1));

  // 发布 joint_command
  joint_command_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      joint_command_topic_, 10);

  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor_->add_node(node_);
  spinning_ = true;
  spin_thread_ = std::thread([this]() {
    while (spinning_ && rclcpp::ok()) {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  });

  RCLCPP_INFO(node_->get_logger(), "IsaacSystemInterface activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn IsaacSystemInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  spinning_ = false;
  if (spin_thread_.joinable()) spin_thread_.join();
  executor_->remove_node(node_);

  joint_state_sub_.reset();
  joint_command_pub_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> IsaacSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    out.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &positions_[i]);
    out.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &velocities_[i]);
    out.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT,   &efforts_[i]);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> IsaacSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    // 只支持 velocity/position（可拓展支持effort等）
    if (command_types_[i] == "velocity")
      out.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &commands_[i]);
    else if (command_types_[i] == "position")
      out.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &commands_[i]);
  }
  return out;
}

hardware_interface::return_type IsaacSystemInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  // 用 latest_joint_state 更新内部状态
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    auto it = std::find(latest_joint_state_.name.begin(), latest_joint_state_.name.end(), joint_names_[i]);
    if (it != latest_joint_state_.name.end()) {
      size_t idx = std::distance(latest_joint_state_.name.begin(), it);
      if (idx < latest_joint_state_.position.size())
        positions_[i] = latest_joint_state_.position[idx];
      if (idx < latest_joint_state_.velocity.size())
        velocities_[i] = latest_joint_state_.velocity[idx];
      if (idx < latest_joint_state_.effort.size())
        efforts_[i] = latest_joint_state_.effort[idx];
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type IsaacSystemInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  // 把当前命令转成 JointState 发布
  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = node_->now();
  msg.name = joint_names_;

  // 保证所有数组都补齐
  msg.position.resize(joint_names_.size(), 0.0);
  msg.velocity.resize(joint_names_.size(), 0.0);
  msg.effort.resize(joint_names_.size(), 0.0);

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    if (command_types_[i] == "velocity")
      msg.velocity[i] = commands_[i];
    else if (command_types_[i] == "position")
      msg.position[i] = commands_[i];
  }
  joint_command_pub_->publish(msg);
  return hardware_interface::return_type::OK;
}

void IsaacSystemInterface::on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_joint_state_ = *msg;
}

}  // namespace isaac_system_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(isaac_system_interface::IsaacSystemInterface, hardware_interface::SystemInterface)
