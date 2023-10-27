#include "steering_report_502.hpp"

Steeringreport502::Steeringreport502() {}
int32_t Steeringreport502::ID = 0x502;

void Steeringreport502::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void Steeringreport502::Parse(){
  steer_angle_spd_actual = decode_steer_angle_spd_actual();
  steer_flt2 = decode_steer_flt2();
  steer_flt1 = decode_steer_flt1();
  steer_en_state = decode_steer_en_state();
  steer_angle_actual = decode_steer_angle_actual();
}

// config detail: {'bit': 55, 'is_signed_var': False, 'len': 8, 'name': 'steer_angle_spd_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': 'deg/s', 'precision': 1.0, 'type': 'int'}
int Steeringreport502::decode_steer_angle_spd_actual() {
  Byte t0(*(bytes + 6));
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 23, 'description': 'Steer system communication fault', 'enum': {0: 'STEER_FLT2_NO_FAULT', 1: 'STEER_FLT2_STEER_SYSTEM_COMUNICATION_FAULT'}, 'is_signed_var': False, 'len': 8, 'name': 'steer_flt2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Steeringreport502::decode_steer_flt2() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(0, 8);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 15, 'description': 'Steer system hardware fault', 'enum': {0: 'STEER_FLT1_NO_FAULT', 1: 'STEER_FLT1_STEER_SYSTEM_HARDWARE_FAULT'}, 'is_signed_var': False, 'len': 8, 'name': 'steer_flt1', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Steeringreport502::decode_steer_flt1() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 8);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 1, 'enum': {0: 'STEER_EN_STATE_MANUAL', 1: 'STEER_EN_STATE_AUTO', 2: 'STEER_EN_STATE_TAKEOVER', 3: 'STEER_EN_STATE_STANDBY'}, 'is_signed_var': False, 'len': 2, 'name': 'steer_en_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|2]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Steeringreport502::decode_steer_en_state() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 2);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 31, 'is_signed_var': False, 'len': 16, 'name': 'steer_angle_actual', 'offset': -500.0, 'order': 'motorola', 'physical_range': '[-360|360]', 'physical_unit': 'deg', 'precision': 1.0, 'type': 'int'}
int Steeringreport502::decode_steer_angle_actual() {
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 4));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  int ret = x + -500.000000;
  return ret;
}

int Steeringreport502::get_steer_angle_spd_actual()
{
  return steer_angle_spd_actual;
}

int Steeringreport502::get_steer_angle_actual()
{
  return steer_angle_actual;
}

int Steeringreport502::get_steer_en_state()
{
  return steer_en_state;
}

int Steeringreport502::get_steer_flt1()
{
  return steer_flt1;
}

int Steeringreport502::get_steer_flt2()
{
  return steer_flt2;
}