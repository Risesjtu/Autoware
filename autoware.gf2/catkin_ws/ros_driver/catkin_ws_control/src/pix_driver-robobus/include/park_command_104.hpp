#pragma once

#include "Byte.hpp"

class Parkcommand104{
 public:
  static int32_t ID;

  Parkcommand104();

  void UpdateData(int checksum_104, int park_target, int park_en_ctrl);

  void Reset();
  uint8_t * get_data();

 private:

  // config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'CheckSum_104', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_checksum_104(int checksum_104);

  // config detail: {'bit': 8, 'enum': {0: 'PARK_TARGET_RELEASE', 1: 'PARK_TARGET_PARKING_TRIGGER'}, 'is_signed_var': False, 'len': 1, 'name': 'Park_Target', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_park_target(int park_target);

  // config detail: {'bit': 0, 'enum': {0: 'PARK_EN_CTRL_DISABLE', 1: 'PARK_EN_CTRL_ENABLE'}, 'is_signed_var': False, 'len': 1, 'name': 'Park_EN_CTRL', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_park_en_ctrl(int park_en_ctrl);

 private:
 uint8_t data[8];
};
