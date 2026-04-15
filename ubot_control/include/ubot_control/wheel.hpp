#ifndef UBOT_HARDWARE_WHEEL_HPP
#define UBOT_HARDWARE_WHEEL_HPP

#include <string>
#include <cmath>

class Wheel {
public:
  std::string name = "";
  int enc = 0;
  int last_enc = 0; // Tracks the previous tick count
  double cmd = 0;
  double pos = 0;
  double vel = 0;
  double rads_per_count = 0;

  void setup(const std::string &wheel_name, int counts_per_rev) {
    name = wheel_name;
    rads_per_count = (2.0 * M_PI) / counts_per_rev;
  }

  void update_position() {
    // Calculate the difference between current and last ticks
    int diff = enc - last_enc;
    pos += diff * rads_per_count;
    last_enc = enc; 
  }
};

#endif