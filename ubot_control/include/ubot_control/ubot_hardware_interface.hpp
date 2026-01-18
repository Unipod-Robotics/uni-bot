// #ifndef UBOT_HARDWARE_INTERFACE_HPP_
// #define UBOT_HARDWARE_INTERFACE_HPP_

// #include <memory>
// #include <string>
// #include <vector>

// #include "hardware_interface/handle.hpp"
// #include "hardware_interface/hardware_info.hpp"
// #include "hardware_interface/system_interface.hpp"
// #include "hardware_interface/types/hardware_interface_return_values.hpp"
// #include "rclcpp/clock.hpp"
// #include "rclcpp/duration.hpp"
// #include "rclcpp/macros.hpp"
// #include "rclcpp/time.hpp"
// #include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
// #include "rclcpp_lifecycle/state.hpp"

// #include "ubot_control/arduino_comms.hpp"
// #include "ubot_control/wheel.hpp"

// namespace ubot_control
// {

// class UbotHardware : public hardware_interface::SystemInterface
// {
//   struct Config
//   {
//     std::string front_left_wheel_name = "";
//     std::string rear_left_wheel_name = "";
//     std::string front_right_wheel_name = "";
//     std::string rear_right_wheel_name = "";
//     float loop_rate = 0.0;
//     std::string device = "";
//     int baud_rate = 0;
//     int timeout_ms = 0;
//     int enc_counts_per_rev = 0;
//     int pid_p = 0;
//     int pid_d = 0;
//     int pid_i = 0;
//     int pid_o = 0;
//   };

// public:
//   RCLCPP_SHARED_PTR_DEFINITIONS(UbotHardware);

//   hardware_interface::CallbackReturn on_init(
//     const hardware_interface::HardwareComponentInterfaceParams & params) override;

//   std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

//   std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

//   hardware_interface::CallbackReturn on_configure(
//     const rclcpp_lifecycle::State & previous_state) override;

//   hardware_interface::CallbackReturn on_cleanup(
//     const rclcpp_lifecycle::State & previous_state) override;

//   hardware_interface::CallbackReturn on_activate(
//     const rclcpp_lifecycle::State & previous_state) override;

//   hardware_interface::CallbackReturn on_deactivate(
//     const rclcpp_lifecycle::State & previous_state) override;

//   hardware_interface::return_type read(
//     const rclcpp::Time & time, const rclcpp::Duration & period) override;

//   hardware_interface::return_type write(
//     const rclcpp::Time & time, const rclcpp::Duration & period) override;

// private:
//   ArduinoComms comms_;
//   Config cfg_;
//   Wheel wheel_l_;  // Left side (both wheels)
//   Wheel wheel_r_;  // Right side (both wheels)
// };

// }  // namespace ubot_control

// #endif  // UBOT_HARDWARE_INTERFACE_HPP_

#ifndef UBOT_HARDWARE_INTERFACE_HPP_
#define UBOT_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "ubot_control/arduino_comms.hpp"
#include "ubot_control/wheel.hpp"

namespace ubot_control
{

class UbotHardware : public hardware_interface::SystemInterface
{
  struct Config
  {
    std::string left_wheel_name = "";
    // std::string rear_left_wheel_name = "";
    std::string right_wheel_name = "";
    // std::string rear_right_wheel_name = "";
    float loop_rate = 0.0;
    std::string device = "";
    int baud_rate = 0;
    int timeout_ms = 0;
    int enc_counts_per_rev = 0;
    int pid_p = 0;
    int pid_d = 0;
    int pid_i = 0;
    int pid_o = 0;
  };

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(UbotHardware);

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
  ArduinoComms comms_;
  Config cfg_;
  Wheel wheel_l_;  // Left side (both wheels)
  Wheel wheel_r_;  // Right side (both wheels)

  // Encoder zeroing
  double left_enc_offset_{0.0};
  double right_enc_offset_{0.0};
  bool encoders_zeroed_{false};
};

}  // namespace ubot_control

#endif  // UBOT_HARDWARE_INTERFACE_HPP_