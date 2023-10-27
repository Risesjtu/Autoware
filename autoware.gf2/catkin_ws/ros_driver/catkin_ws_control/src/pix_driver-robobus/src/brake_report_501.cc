#include "brake_report_501.hpp"

Brakereport501::Brakereport501() {}
int32_t Brakereport501::ID = 0x501;

void Brakereport501::Parse()
{
  brake_pedal_actual = decode_brake_pedal_actual();
  brake_flt2 = decode_brake_flt2();
  brake_flt1 = decode_brake_flt1();
  brake_en_state = decode_brake_en_state();
}

void Brakereport501::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}


// config detail: {'bit': 31, 'is_signed_var': False, 'len': 16, 'name': 'brake_pedal_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
double Brakereport501::decode_brake_pedal_actual(){
  Byte t0(*(bytes + 3));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 4));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000;
  return ret;
}

// config detail: {'bit': 23, 'description': 'Brake system communication fault', 'enum': {0: 'BRAKE_FLT2_NO_FAULT', 1: 'BRAKE_FLT2_BRAKE_SYSTEM_COMUNICATION_FAULT'}, 'is_signed_var': False, 'len': 8, 'name': 'brake_flt2', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Brakereport501::decode_brake_flt2(){
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(0, 8);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 15, 'description': 'Brake system hardware fault', 'enum': {0: 'BRAKE_FLT1_NO_FAULT', 1: 'BRAKE_FLT1_BRAKE_SYSTEM_HARDWARE_FAULT'}, 'is_signed_var': False, 'len': 8, 'name': 'brake_flt1', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Brakereport501::decode_brake_flt1(){
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 8);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 1, 'enum': {0: 'BRAKE_EN_STATE_MANUAL', 1: 'BRAKE_EN_STATE_AUTO', 2: 'BRAKE_EN_STATE_TAKEOVER', 3: 'BRAKE_EN_STATE_STANDBY'}, 'is_signed_var': False, 'len': 2, 'name': 'brake_en_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|2]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Brakereport501::decode_brake_en_state(){
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 2);

  int ret =  static_cast<int>(x);
  return ret;
}
