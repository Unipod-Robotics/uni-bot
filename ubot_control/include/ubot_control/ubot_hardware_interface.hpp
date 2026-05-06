#ifndef UBOT_HARDWARE_INTERFACE_HPP_
#define UBOT_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"

#include "ubot_control/wheel.hpp"

namespace ubot_control
{

class UbotHardware : public hardware_interface::SystemInterface
{
  struct Config
  {
    std::string left_wheel_name  = "";
    std::string right_wheel_name = "";
    int    enc_counts_per_rev_left  = 0;
    int    enc_counts_per_rev_right = 0;
    double wheel_separation = 0.0;
    double wheel_radius     = 0.0;
    float  loop_rate        = 0.0;
  };

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(UbotHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  Config cfg_;
  Wheel  wheel_l_;
  Wheel  wheel_r_;

  // ROS 2 topic transport
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr  left_enc_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr  right_enc_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  // Double-buffer for thread-safe encoder access between callback and read() threads
  std::mutex enc_mutex_;
  int32_t left_enc_staged_  = 0;
  int32_t right_enc_staged_ = 0;
  int32_t left_enc_safe_    = 0;
  int32_t right_enc_safe_   = 0;
};

}  // namespace ubot_control

#endif  // UBOT_HARDWARE_INTERFACE_HPP_
