#pragma once

#include "Byte.hpp"
#include <iostream>

class Brakereport501
{
 public:
  static int32_t ID;
  Brakereport501();
  void Parse();
  void update_bytes(uint8_t bytes_data[8]);
  double brake_pedal_actual;
  int brake_flt2;
  int brake_flt1;
  int brake_en_state;

 private:
  uint8_t bytes[8];
  

  // config detail: {'bit': 31, 'is_signed_var': False, 'len': 16, 'name': 'Brake_Pedal_Actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
  double decode_brake_pedal_actual();

  // config detail: {'bit': 23, 'description': 'Brake system communication fault', 'enum': {0: 'BRAKE_FLT2_NO_FAULT', 1: 'BRAKE_FLT2_BRAKE_SYSTEM_COMUNICATION_FAULT'}, 'is_signed_var': False, 'len': 8, 'name': 'Brake_FLT2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int decode_brake_flt2();

  // config detail: {'bit': 15, 'description': 'Brake system hardware fault', 'enum': {0: 'BRAKE_FLT1_NO_FAULT', 1: 'BRAKE_FLT1_BRAKE_SYSTEM_HARDWARE_FAULT'}, 'is_signed_var': False, 'len': 8, 'name': 'Brake_FLT1', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int decode_brake_flt1();

  // config detail: {'bit': 1, 'enum': {0: 'BRAKE_EN_STATE_MANUAL', 1: 'BRAKE_EN_STATE_AUTO', 2: 'BRAKE_EN_STATE_TAKEOVER', 3: 'BRAKE_EN_STATE_STANDBY'}, 'is_signed_var': False, 'len': 2, 'name': 'Brake_EN_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|2]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int decode_brake_en_state();
};


