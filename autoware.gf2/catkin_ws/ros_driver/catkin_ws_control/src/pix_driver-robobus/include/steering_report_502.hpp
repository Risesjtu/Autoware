#pragma once

#include "Byte.hpp"


class Steeringreport502
{
 public:
  static int32_t ID;
  Steeringreport502();
  void Parse();
  void update_bytes(uint8_t bytes_data[8]);
  int get_steer_angle_spd_actual();
  int get_steer_flt2();
  int get_steer_flt1();
  int get_steer_en_state();
  int get_steer_angle_actual();
  int steer_angle_spd_actual;
  int steer_flt2;
  int steer_flt1;
  int steer_en_state;
  int steer_angle_actual;

 private:
  uint8_t bytes[8];
;
  // config detail: {'bit': 55, 'is_signed_var': False, 'len': 8, 'name': 'Steer_ANGLE_SPD_Actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': 'deg/s', 'precision': 1.0, 'type': 'int'}
  int decode_steer_angle_spd_actual();

  // config detail: {'bit': 23, 'description': 'Steer system communication fault', 'enum': {0: 'STEER_FLT2_NO_FAULT', 1: 'STEER_FLT2_STEER_SYSTEM_COMUNICATION_FAULT'}, 'is_signed_var': False, 'len': 8, 'name': 'Steer_FLT2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int decode_steer_flt2();

  // config detail: {'bit': 15, 'description': 'Steer system hardware fault', 'enum': {0: 'STEER_FLT1_NO_FAULT', 1: 'STEER_FLT1_STEER_SYSTEM_HARDWARE_FAULT'}, 'is_signed_var': False, 'len': 8, 'name': 'Steer_FLT1', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int decode_steer_flt1();

  // config detail: {'bit': 1, 'enum': {0: 'STEER_EN_STATE_MANUAL', 1: 'STEER_EN_STATE_AUTO', 2: 'STEER_EN_STATE_TAKEOVER', 3: 'STEER_EN_STATE_STANDBY'}, 'is_signed_var': False, 'len': 2, 'name': 'Steer_EN_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|2]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int decode_steer_en_state();

  // config detail: {'bit': 31, 'is_signed_var': False, 'len': 16, 'name': 'Steer_ANGLE_Actual', 'offset': -500.0, 'order': 'motorola', 'physical_range': '[-360|360]', 'physical_unit': 'deg', 'precision': 1.0, 'type': 'int'}
  int decode_steer_angle_actual();
};


