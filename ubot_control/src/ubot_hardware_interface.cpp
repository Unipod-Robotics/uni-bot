// Copyright 2024 ubot Development Team

#include "ubot_control/ubot_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ubot_control
{

hardware_interface::CallbackReturn UbotHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.left_wheel_name  = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];

  cfg_.enc_counts_per_rev_left  = std::stoi(info_.hardware_parameters["enc_counts_per_rev_left"]);
  cfg_.enc_counts_per_rev_right = std::stoi(info_.hardware_parameters["enc_counts_per_rev_right"]);
  cfg_.wheel_separation = hardware_interface::stod(info_.hardware_parameters["wheel_separation"]);
  cfg_.wheel_radius     = hardware_interface::stod(info_.hardware_parameters["wheel_radius"]);
  cfg_.loop_rate        = hardware_interface::stod(info_.hardware_parameters["loop_rate"]);

  wheel_l_.setup(cfg_.left_wheel_name,  cfg_.enc_counts_per_rev_left);
  wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev_right);

  // Expect all 4 wheel joints: front and rear, where rear mirrors front state
  if (info_.joints.size() != 4)
  {
    RCLCPP_FATAL(get_logger(), "Expected 4 joints, found %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Joints with command interfaces must use velocity
    if (!joint.command_interfaces.empty())
    {
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has %zu command interfaces. 0 or 1 expected.",
          joint.name.c_str(), joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' has '%s' command interface. '%s' expected.",
          joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interfaces. 2 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
UbotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(
    "front_left_wheel_joint", hardware_interface::HW_IF_POSITION, &wheel_l_.pos);
  state_interfaces.emplace_back(
    "front_left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel);

  state_interfaces.emplace_back(
    "front_right_wheel_joint", hardware_interface::HW_IF_POSITION, &wheel_r_.pos);
  state_interfaces.emplace_back(
    "front_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel);

  // Rear wheels are physically driven by the same motors as front — mirror their state
  state_interfaces.emplace_back(
    "rear_left_wheel_joint", hardware_interface::HW_IF_POSITION, &wheel_l_.pos);
  state_interfaces.emplace_back(
    "rear_left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel);

  state_interfaces.emplace_back(
    "rear_right_wheel_joint", hardware_interface::HW_IF_POSITION, &wheel_r_.pos);
  state_interfaces.emplace_back(
    "rear_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
UbotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
    "front_left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd);
  command_interfaces.emplace_back(
    "front_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd);

  return command_interfaces;
}

hardware_interface::CallbackReturn UbotHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring...");

  auto node = get_node();
  cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions opts;
  opts.callback_group = cb_group_;

    // BestEffort QoS: micro-ROS with CycloneDDS does not negotiate QoS normally
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                .reliability(rclcpp::ReliabilityPolicy::BestEffort)
                .durability(rclcpp::DurabilityPolicy::Volatile);

  left_enc_sub_ = node->create_subscription<std_msgs::msg::Int32>(
    "/horizon/left_encoder", qos,
    [this](const std_msgs::msg::Int32::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(enc_mutex_);
      left_enc_staged_ = msg->data;
    }, opts);

  right_enc_sub_ = node->create_subscription<std_msgs::msg::Int32>(
    "/horizon/right_encoder", qos,
    [this](const std_msgs::msg::Int32::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(enc_mutex_);
      right_enc_staged_ = msg->data;
    }, opts);

  cmd_vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  RCLCPP_INFO(get_logger(), "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UbotHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up...");
  left_enc_sub_  = nullptr;
  right_enc_sub_ = nullptr;
  cmd_vel_pub_   = nullptr;
  RCLCPP_INFO(get_logger(), "Successfully cleaned up!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UbotHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating...");

  // Reset wheel kinematics
  wheel_l_.enc = 0;  wheel_r_.enc = 0;
  wheel_l_.pos = 0.0; wheel_r_.pos = 0.0;
  wheel_l_.vel = 0.0; wheel_r_.vel = 0.0;
  wheel_l_.cmd = 0.0; wheel_r_.cmd = 0.0;

  // Seed safe counts from whatever the ESP32 has already published so the
  // first read() delta is zero instead of the full accumulated tick count
  {
    std::lock_guard<std::mutex> lock(enc_mutex_);
    left_enc_safe_  = left_enc_staged_;
    right_enc_safe_ = right_enc_staged_;
  }
  wheel_l_.enc      = static_cast<int>(left_enc_safe_);
  wheel_r_.enc      = static_cast<int>(right_enc_safe_);
  wheel_l_.last_enc = wheel_l_.enc;
  wheel_r_.last_enc = wheel_r_.enc;

  // Safety: ensure robot is stopped on activation
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});

  RCLCPP_INFO(get_logger(), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UbotHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating...");
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type UbotHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Atomically copy staged encoder counts (written by subscription callbacks) to safe buffer
  {
    std::lock_guard<std::mutex> lock(enc_mutex_);
    left_enc_safe_  = left_enc_staged_;
    right_enc_safe_ = right_enc_staged_;
  }
  wheel_l_.enc = static_cast<int>(left_enc_safe_);
  wheel_r_.enc = static_cast<int>(right_enc_safe_);

  double dt = period.seconds();

  double prev_pos_l = wheel_l_.pos;
  wheel_l_.update_position();
  wheel_l_.vel = (wheel_l_.pos - prev_pos_l) / dt;

  double prev_pos_r = wheel_r_.pos;
  wheel_r_.update_position();
  wheel_r_.vel = (wheel_r_.pos - prev_pos_r) / dt;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type UbotHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Convert per-wheel rad/s commands to m/s, then build Twist for ESP32
  double v_left_ms  = wheel_l_.cmd * cfg_.wheel_radius;
  double v_right_ms = wheel_r_.cmd * cfg_.wheel_radius;

  geometry_msgs::msg::Twist twist;
  twist.linear.x  = (v_left_ms + v_right_ms) / 2.0;
  twist.angular.z = (v_right_ms - v_left_ms) / cfg_.wheel_separation;
  cmd_vel_pub_->publish(twist);

  return hardware_interface::return_type::OK;
}

}  // namespace ubot_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ubot_control::UbotHardware, hardware_interface::SystemInterface)
