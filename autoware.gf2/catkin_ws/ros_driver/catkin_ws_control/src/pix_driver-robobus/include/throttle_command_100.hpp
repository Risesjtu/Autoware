#pragma once

#include "Byte.hpp"

class Throttlecommand100{
 public:
  static int32_t ID;

  Throttlecommand100();

  void UpdateData(double vel_target, double throttle_acc, int checksum_100, double throttle_pedal_target, int throttle_en_ctrl);

  void Reset();

  uint8_t * get_data();

 private:

  // config detail: {'bit': 47, 'is_signed_var': False, 'len': 10, 'name': 'Vel_Target', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|10.23]', 'physical_unit': 'm/s', 'precision': 0.01, 'type': 'double'}
  void set_p_vel_target(double vel_target);

  // config detail: {'bit': 15, 'is_signed_var': False, 'len': 10, 'name': 'Throttle_Acc', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|10]', 'physical_unit': 'm/s^2', 'precision': 0.01, 'type': 'double'}
  void set_p_throttle_acc(double throttle_acc);

  // config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'CheckSum_100', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_checksum_100(int checksum_100);

  // config detail: {'bit': 31, 'is_signed_var': False, 'len': 16, 'name': 'Throttle_Pedal_Target', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
  void set_p_throttle_pedal_target(double throttle_pedal_target);

  // config detail: {'bit': 0, 'enum': {0: 'THROTTLE_EN_CTRL_DISABLE', 1: 'THROTTLE_EN_CTRL_ENABLE'}, 'is_signed_var': False, 'len': 1, 'name': 'Throttle_EN_CTRL', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_throttle_en_ctrl(int throttle_en_ctrl);

 private:
  uint8_t data[8];
};
