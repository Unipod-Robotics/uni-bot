// // Copyright 2024 ubot Development Team
// // Updated for 4-wheel differential drive

// #include "ubot_control/ubot_hardware_interface.hpp"

// #include <chrono>
// #include <cmath>
// #include <limits>
// #include <memory>
// #include <vector>

// #include "hardware_interface/lexical_casts.hpp"
// #include "hardware_interface/types/hardware_interface_type_values.hpp"
// #include "rclcpp/rclcpp.hpp"

// namespace ubot_control
// {

// hardware_interface::CallbackReturn UbotHardware::on_init(
//   const hardware_interface::HardwareComponentInterfaceParams & params)
// {
//   if (
//     hardware_interface::SystemInterface::on_init(params) !=
//     hardware_interface::CallbackReturn::SUCCESS)
//   {
//     return hardware_interface::CallbackReturn::ERROR;
//   }

//   // Extract parameters from URDF
//   cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
// //   cfg_.rear_left_wheel_name = info_.hardware_parameters["rear_left_wheel_name"];
//   cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
// //   cfg_.rear_right_wheel_name = info_.hardware_parameters["rear_right_wheel_name"];
  
//   cfg_.loop_rate = hardware_interface::stod(info_.hardware_parameters["loop_rate"]);
//   cfg_.device = info_.hardware_parameters["device"];
//   cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
//   cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
//   cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  
//   if (info_.hardware_parameters.count("pid_p") > 0)
//   {
//     cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
//     cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
//     cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
//     cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
//   }
//   else
//   {
//     RCLCPP_INFO(get_logger(), "PID values not supplied, using defaults.");
//   }
  
//   // Setup 2 wheels
//   wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
// //   wheel_rl_.setup(cfg_.rear_left_wheel_name, cfg_.enc_counts_per_rev);
//   wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);
// //   wheel_rr_.setup(cfg_.rear_right_wheel_name, cfg_.enc_counts_per_rev);

//   // Verify all 4 joints have correct interfaces
//   if (info_.joints.size() != 4)
//   {
//     RCLCPP_FATAL(
//       get_logger(),
//       "Expected 4 joints, found %zu", info_.joints.size());
//     return hardware_interface::CallbackReturn::ERROR;
//   }

//   for (const hardware_interface::ComponentInfo & joint : info_.joints)
//   {
//     if (joint.command_interfaces.size() != 1)
//     {
//       RCLCPP_FATAL(
//         get_logger(),
//         "Joint '%s' has %zu command interfaces. 1 expected.", 
//         joint.name.c_str(), joint.command_interfaces.size());
//       return hardware_interface::CallbackReturn::ERROR;
//     }

//     if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
//     {
//       RCLCPP_FATAL(
//         get_logger(),
//         "Joint '%s' has '%s' command interface. '%s' expected.", 
//         joint.name.c_str(),
//         joint.command_interfaces[0].name.c_str(), 
//         hardware_interface::HW_IF_VELOCITY);
//       return hardware_interface::CallbackReturn::ERROR;
//     }

//     if (joint.state_interfaces.size() != 2)
//     {
//       RCLCPP_FATAL(
//         get_logger(),
//         "Joint '%s' has %zu state interfaces. 2 expected.", 
//         joint.name.c_str(), joint.state_interfaces.size());
//       return hardware_interface::CallbackReturn::ERROR;
//     }
//   }

//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// std::vector<hardware_interface::StateInterface> 
// UbotHardware::export_state_interfaces()
// {
//   std::vector<hardware_interface::StateInterface> state_interfaces;

//   // Front and rear left share the same state
//   state_interfaces.emplace_back(
//     "front_left_wheel_joint", HW_IF_POSITION, &wheel_l_.pos);
//   state_interfaces.emplace_back(
//     "front_left_wheel_joint", HW_IF_VELOCITY, &wheel_l_.vel);
    
//   state_interfaces.emplace_back(
//     "rear_left_wheel_joint", HW_IF_POSITION, &wheel_l_.pos);  // Same!
//   state_interfaces.emplace_back(
//     "rear_left_wheel_joint", HW_IF_VELOCITY, &wheel_l_.vel);  // Same!

//   // Front and rear right share the same state
//   state_interfaces.emplace_back(
//     "front_right_wheel_joint", HW_IF_POSITION, &wheel_r_.pos);
//   state_interfaces.emplace_back(
//     "front_right_wheel_joint", HW_IF_VELOCITY, &wheel_r_.vel);
    
//   state_interfaces.emplace_back(
//     "rear_right_wheel_joint", HW_IF_POSITION, &wheel_r_.pos);  // Same!
//   state_interfaces.emplace_back(
//     "rear_right_wheel_joint", HW_IF_VELOCITY, &wheel_r_.vel);  // Same!

//   return state_interfaces;
// }

// std::vector<hardware_interface::CommandInterface> UbotHardware::export_command_interfaces()
// {
//   std::vector<hardware_interface::CommandInterface> command_interfaces;

// //   command_interfaces.emplace_back(hardware_interface::CommandInterface(
// //     wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

// // //   command_interfaces.emplace_back(hardware_interface::CommandInterface(
// // //     wheel_rl_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rl_.cmd));

// //   command_interfaces.emplace_back(hardware_interface::CommandInterface(
// //     wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

// // //   command_interfaces.emplace_back(hardware_interface::CommandInterface(
// // //     wheel_rr_.name, hardware_interface::HW_IF_VELOCITY, &wheel_rr_.cmd));

//     command_interfaces.emplace_back(
//     "front_left_wheel_joint", HW_IF_VELOCITY, &wheel_l_.cmd);
//     command_interfaces.emplace_back(
//     "rear_left_wheel_joint", HW_IF_VELOCITY, &wheel_l_.cmd);  // Same as FL!

//     command_interfaces.emplace_back(
//     "front_right_wheel_joint", HW_IF_VELOCITY, &wheel_r_.cmd);
//     command_interfaces.emplace_back(
//     "rear_right_wheel_joint", HW_IF_VELOCITY, &wheel_r_.cmd);  // Same as FR!

//   return command_interfaces;
// }

// hardware_interface::CallbackReturn UbotHardware::on_configure(
//   const rclcpp_lifecycle::State & /*previous_state*/)
// {
//   RCLCPP_INFO(get_logger(), "Configuring ...please wait...");
  
//   if (comms_.connected())
//   {
//     comms_.disconnect();
//   }
  
//   comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);
  
//   RCLCPP_INFO(get_logger(), "Successfully configured!");
//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::CallbackReturn UbotHardware::on_cleanup(
//   const rclcpp_lifecycle::State & /*previous_state*/)
// {
//   RCLCPP_INFO(get_logger(), "Cleaning up ...please wait...");
  
//   if (comms_.connected())
//   {
//     comms_.disconnect();
//   }
  
//   RCLCPP_INFO(get_logger(), "Successfully cleaned up!");
//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::CallbackReturn UbotHardware::on_activate(
//   const rclcpp_lifecycle::State & /*previous_state*/)
// {
//   RCLCPP_INFO(get_logger(), "Activating ...please wait...");
  
//   if (!comms_.connected())
//   {
//     return hardware_interface::CallbackReturn::ERROR;
//   }
  
//   if (cfg_.pid_p > 0)
//   {
//     comms_.set_pid_values(cfg_.pid_p, cfg_.pid_d, cfg_.pid_i, cfg_.pid_o);
//   }
  
//   RCLCPP_INFO(get_logger(), "Successfully activated!");
//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::CallbackReturn UbotHardware::on_deactivate(
//   const rclcpp_lifecycle::State & /*previous_state*/)
// {
//   RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");
//   RCLCPP_INFO(get_logger(), "Successfully deactivated!");
//   return hardware_interface::CallbackReturn::SUCCESS;
// }

// hardware_interface::return_type UbotHardware::read(
//   const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
// {
//   if (!comms_.connected())
//   {
//     return hardware_interface::return_type::ERROR;
//   }

//   // Read encoder values for all 4 wheels
//   // NOTE: Your Arduino must send 4 encoder values now!
//   comms_.read_encoder_values(
//     wheel_l_.enc, wheel_r_.enc);

//   double delta_seconds = period.seconds();

//   // Update front left wheel
//   double pos_prev = wheel_l_.pos;
//   wheel_l_.pos = wheel_l_.calc_enc_angle();
//   wheel_l_.vel = (wheel_l_.pos - pos_prev) / delta_seconds;

// //   // Update rear left wheel
// //   pos_prev = wheel_rl_.pos;
// //   wheel_rl_.pos = wheel_rl_.calc_enc_angle();
// //   wheel_rl_.vel = (wheel_rl_.pos - pos_prev) / delta_seconds;

//   // Update front right wheel
//   pos_prev = wheel_r_.pos;
//   wheel_r_.pos = wheel_r_.calc_enc_angle();
//   wheel_r_.vel = (wheel_r_.pos - pos_prev) / delta_seconds;

// //   // Update rear right wheel
// //   pos_prev = wheel_rr_.pos;
// //   wheel_rr_.pos = wheel_rr_.calc_enc_angle();
// //   wheel_rr_.vel = (wheel_rr_.pos - pos_prev) / delta_seconds;

//   return hardware_interface::return_type::OK;
// }

// hardware_interface::return_type UbotHardware::write(
//   const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
// {
//   if (!comms_.connected())
//   {
//     return hardware_interface::return_type::ERROR;
//   }

//   // Convert velocity commands to encoder counts per loop for each wheel
//   int motor_l_counts = wheel_l_.cmd / wheel_l_.rads_per_count / cfg_.loop_rate;
// //   int motor_rl_counts = wheel_rl_.cmd / wheel_rl_.rads_per_count / cfg_.loop_rate;
//   int motor_r_counts = wheel_r_.cmd / wheel_r_.rads_per_count / cfg_.loop_rate;
// //   int motor_rr_counts = wheel_rr_.cmd / wheel_rr_.rads_per_count / cfg_.loop_rate;
  
//   // Send all 4 motor commands
//   comms_.set_motor_values(
//     motor_l_counts, motor_r_counts);
  
//   return hardware_interface::return_type::OK;
// }

// }  // namespace ubot_control

// #include "pluginlib/class_list_macros.hpp"
// PLUGINLIB_EXPORT_CLASS(
//   ubot_control::UbotHardware, hardware_interface::SystemInterface)

// Copyright 2024 ubot Development Team
// Updated for 2-wheel differential drive

#include "ubot_control/ubot_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
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

  wheel_l_.pos = 0.0;
  wheel_l_.vel = 0.0;
  wheel_l_.cmd = 0.0;

  wheel_r_.pos = 0.0;
  wheel_r_.vel = 0.0;
  wheel_r_.cmd = 0.0;

  // Extract parameters from URDF
  cfg_.left_wheel_name  = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];

  cfg_.loop_rate = hardware_interface::stod(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  if (info_.hardware_parameters.count("pid_p") > 0)
  {
    cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
    cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
    cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
    cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
  }
  else
  {
    RCLCPP_INFO(get_logger(), "PID values not supplied, using defaults.");
  }

  // Setup ONLY 2 wheels
  wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);

  // Expect EXACTLY 2 joints
  if (info_.joints.size() != 2)
  {
    RCLCPP_FATAL(
      get_logger(),
      "Expected 2 joints, found %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' has %zu command interfaces. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' has '%s' command interface. '%s' expected.",
        joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        get_logger(),
        "Joint '%s' has %zu state interfaces. 2 expected.",
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
  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

  if (comms_.connected())
  {
    comms_.disconnect();
  }

  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);

  RCLCPP_INFO(get_logger(), "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UbotHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up ...please wait...");

  if (comms_.connected())
  {
    comms_.disconnect();
  }

  RCLCPP_INFO(get_logger(), "Successfully cleaned up!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UbotHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (cfg_.pid_p > 0)
  {
    comms_.set_pid_values(cfg_.pid_p, cfg_.pid_d, cfg_.pid_i, cfg_.pid_o);
  }

  // RESET ENCODERS TO ZERO
  RCLCPP_INFO(get_logger(), "Resetting encoders...");
  comms_.send_msg("r\r");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  
  // Flush any "OK" responses
  comms_.send_msg("e\r");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  // Initialize wheel state to zero
  wheel_l_.enc = 0;
  wheel_r_.enc = 0;
  wheel_l_.pos = 0.0;
  wheel_r_.pos = 0.0;
  wheel_l_.vel = 0.0;
  wheel_r_.vel = 0.0;

  RCLCPP_INFO(get_logger(), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UbotHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");
  RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type UbotHardware::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (!comms_.connected()) return hardware_interface::return_type::ERROR;

  // 1. Comms layer updates .enc ONLY if data is valid
  comms_.read_encoder_values(wheel_l_.enc, wheel_r_.enc);

  double dt = period.seconds();

  // 2. Left Wheel: Calculate movement since last successful read
  double prev_pos_l = wheel_l_.pos;
  wheel_l_.update_position(); 
  wheel_l_.vel = (wheel_l_.pos - prev_pos_l) / dt;

  // 3. Right Wheel: Calculate movement since last successful read
  double prev_pos_r = wheel_r_.pos;
  wheel_r_.update_position(); 
  wheel_r_.vel = (wheel_r_.pos - prev_pos_r) / dt;

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type UbotHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!comms_.connected()) return hardware_interface::return_type::ERROR;

  // Convert rad/s to Ticks Per Second
  // Formula: (rad/s) / (rad/tick) = ticks/s
  int motor_l = static_cast<int>(wheel_l_.cmd / wheel_l_.rads_per_count);
  int motor_r = static_cast<int>(wheel_r_.cmd / wheel_r_.rads_per_count);

  comms_.set_motor_values(motor_l, motor_r);

  return hardware_interface::return_type::OK;
}

}  // namespace ubot_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ubot_control::UbotHardware, hardware_interface::SystemInterface)