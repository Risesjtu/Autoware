#pragma once
#include <iostream>
#include "Byte.hpp"

class Throttlereport500{
 public:
  static int32_t ID;
  Throttlereport500();
  void Parse();
  void update_bytes(uint8_t bytes_data[8]);
  double throttle_pedal_actual;
  int throttle_flt2;
  int throttle_flt1;
  int throttle_en_state; 

 private:
  uint8_t bytes[8];

  // config detail: {'bit': 31, 'is_signed_var': False, 'len': 16, 'name': 'Throttle_Pedal_Actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
  double decode_throttle_pedal_actual();

  // config detail: {'bit': 23, 'description': 'Drive system communication fault', 'enum': {0: 'THROTTLE_FLT2_NO_FAULT', 1: 'THROTTLE_FLT2_DRIVE_SYSTEM_COMUNICATION_FAULT'}, 'is_signed_var': False, 'len': 8, 'name': 'Throttle_FLT2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int decode_throttle_flt2();

  // config detail: {'bit': 15, 'description': 'Drive system hardware fault', 'enum': {0: 'THROTTLE_FLT1_NO_FAULT', 1: 'THROTTLE_FLT1_DRIVE_SYSTEM_HARDWARE_FAULT'}, 'is_signed_var': False, 'len': 8, 'name': 'Throttle_FLT1', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int decode_throttle_flt1();

  // config detail: {'bit': 1, 'enum': {0: 'THROTTLE_EN_STATE_MANUAL', 1: 'THROTTLE_EN_STATE_AUTO', 2: 'THROTTLE_EN_STATE_TAKEOVER', 3: 'THROTTLE_EN_STATE_STANDBY'}, 'is_signed_var': False, 'len': 2, 'name': 'Throttle_EN_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|2]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int decode_throttle_en_state();
};

