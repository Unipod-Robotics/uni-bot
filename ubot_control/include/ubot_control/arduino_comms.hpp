#ifndef UBOT_CONTROL_ARDUINO_COMMS_HPP_
#define UBOT_CONTROL_ARDUINO_COMMS_HPP_

#include <string>
#include <sstream>
#include <cstdlib>
#include <iostream>

#include <libserial/SerialPort.h>

namespace ubot_control
{

inline LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cerr << "Unsupported baud rate " << baud_rate
                << ", defaulting to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms
{
public:
  ArduinoComms() = default;

  void connect(const std::string & serial_device,
               int baud_rate,
               int timeout_ms)
  {
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    if (serial_conn_.IsOpen())
    {
      serial_conn_.Close();
    }
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }

  std::string send_msg(const std::string & msg, bool print = false)
  {
    serial_conn_.FlushIOBuffers();
    serial_conn_.Write(msg);

    std::string response;
    try
    {
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout &)
    {
      std::cerr << "Serial read timeout" << std::endl;
    }

    if (print)
    {
      std::cout << "TX: " << msg << " RX: " << response << std::endl;
    }

    return response;
  }

  void read_encoder_values(int & left, int & right)
  {
    std::string response = send_msg("e\r");
    if (response.empty()) return;

    try {
        const auto pos = response.find(' ');
        if (pos == std::string::npos) return;

        // We store in temporary variables first
        int new_left  = std::stoi(response.substr(0, pos));
        int new_right = std::stoi(response.substr(pos + 1));

        // Only update the actual references if conversion succeeded
        left = new_left;
        right = new_right;
    }
    catch (...) {
        // A simple, standard C++ error message that doesn't need ROS2 handles
        std::cerr << "[UbotComms] Serial data corruption detected, skipping frame." << std::endl;
    }
  }

  void set_motor_values(int left, int right)
  {
    std::stringstream ss;
    ss << "m " << left << " " << right << "\r";
    send_msg(ss.str());
  }

  void set_pid_values(int p, int d, int i, int o)
  {
    std::stringstream ss;
    ss << "u " << p << ":" << d << ":" << i << ":" << o << "\r";
    send_msg(ss.str());
  }

private:
  LibSerial::SerialPort serial_conn_;
  int timeout_ms_{0};
};

}  // namespace ubot_control

#endif  // UBOT_CONTROL_ARDUINO_COMMS_HPP_
