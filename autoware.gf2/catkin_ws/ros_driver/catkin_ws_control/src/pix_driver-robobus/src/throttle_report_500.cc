#include "throttle_report_500.hpp"

Throttlereport500::Throttlereport500() {}
int32_t Throttlereport500::ID = 0x500;

void Throttlereport500::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void Throttlereport500::Parse(){
  throttle_pedal_actual = decode_throttle_pedal_actual();
  throttle_flt2 = decode_throttle_flt2();
  throttle_flt1 = decode_throttle_flt1();
  throttle_en_state = decode_throttle_en_state();
}

// config detail: {'bit': 31, 'is_signed_var': False, 'len': 16, 'name': 'throttle_pedal_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
double Throttlereport500::decode_throttle_pedal_actual() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 4));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 23, 'description': 'Drive system communication fault', 'enum': {0: 'THROTTLE_FLT2_NO_FAULT', 1: 'THROTTLE_FLT2_DRIVE_SYSTEM_COMUNICATION_FAULT'}, 'is_signed_var': False, 'len': 8, 'name': 'throttle_flt2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Throttlereport500::decode_throttle_flt2() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(0, 8);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 15, 'description': 'Drive system hardware fault', 'enum': {0: 'THROTTLE_FLT1_NO_FAULT', 1: 'THROTTLE_FLT1_DRIVE_SYSTEM_HARDWARE_FAULT'}, 'is_signed_var': False, 'len': 8, 'name': 'throttle_flt1', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Throttlereport500::decode_throttle_flt1() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 8);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 1, 'enum': {0: 'THROTTLE_EN_STATE_MANUAL', 1: 'THROTTLE_EN_STATE_AUTO', 2: 'THROTTLE_EN_STATE_TAKEOVER', 3: 'THROTTLE_EN_STATE_STANDBY'}, 'is_signed_var': False, 'len': 2, 'name': 'throttle_en_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|2]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Throttlereport500::decode_throttle_en_state() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 2);

  int ret =  static_cast<int>(x);
  return ret;
}
