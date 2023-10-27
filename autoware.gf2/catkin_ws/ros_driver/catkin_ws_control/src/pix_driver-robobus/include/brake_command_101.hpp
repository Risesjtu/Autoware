#pragma once
#include "Byte.hpp"


class Brakecommand101 {
 public:
  static int32_t ID;

  Brakecommand101();

  void UpdateData(int aeb_en_ctrl, double brake_dec, int checksum_101, double brake_pedal_traget, int brake_en_ctrl);

  void Reset();

  uint8_t *get_data();

 private:

  // config detail: {'bit': 1, 'enum': {0: 'AEB_EN_CTRL_DISABLE_AEB', 1: 'AEB_EN_CTRL_ENABLE_AEB'}, 'is_signed_var': False, 'len': 1, 'name': 'AEB_EN_CTRL', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_aeb_en_ctrl(int aeb_en_ctrl);

  // config detail: {'bit': 15, 'is_signed_var': False, 'len': 10, 'name': 'Brake_Dec', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|10]', 'physical_unit': 'm/s^2', 'precision': 0.01, 'type': 'double'}
  void set_p_brake_dec(double brake_dec);

  // config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'CheckSum_101', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_checksum_101(int checksum_101);

  // config detail: {'bit': 31, 'is_signed_var': False, 'len': 16, 'name': 'Brake_Pedal_Target', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
  void set_p_brake_pedal_target(double brake_pedal_target);

  // config detail: {'bit': 0, 'enum': {0: 'BRAKE_EN_CTRL_DISABLE', 1: 'BRAKE_EN_CTRL_ENABLE'}, 'is_signed_var': False, 'len': 1, 'name': 'Brake_EN_CTRL', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_brake_en_ctrl(int brake_en_ctrl);

 private:
  uint8_t data[8];
};


