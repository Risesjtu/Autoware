#pragma once

#include "Byte.hpp"


class Steeringcommand102{
 public:
  static int32_t ID;

  Steeringcommand102();

  void UpdateData(int steer_en_ctrl, int steer_angle_target, int steer_angle_spd, int checksum_102);

  void Reset();
  uint8_t * get_data();

 private:

  // config detail: {'bit': 0, 'enum': {0: 'STEER_EN_CTRL_DISABLE', 1: 'STEER_EN_CTRL_ENABLE'}, 'is_signed_var': False, 'len': 1, 'name': 'Steer_EN_CTRL', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_steer_en_ctrl(int steer_en_ctrl);

  // config detail: {'bit': 31, 'is_signed_var': False, 'len': 16, 'name': 'Steer_ANGLE_Target', 'offset': -500.0, 'order': 'motorola', 'physical_range': '[-360|360]', 'physical_unit': 'deg', 'precision': 1.0, 'type': 'int'}
  void set_p_steer_angle_target(int steer_angle_target);

  // config detail: {'bit': 15, 'is_signed_var': False, 'len': 8, 'name': 'Steer_ANGLE_SPD', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|250]', 'physical_unit': 'deg/s', 'precision': 1.0, 'type': 'int'}
  void set_p_steer_angle_spd(int steer_angle_spd);

  // config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'CheckSum_102', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_checksum_102(int checksum_102);

 private:
  uint8_t data[8];
};


