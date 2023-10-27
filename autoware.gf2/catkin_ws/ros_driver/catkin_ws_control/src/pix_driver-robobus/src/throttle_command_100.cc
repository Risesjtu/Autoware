#include "throttle_command_100.hpp"

int32_t Throttlecommand100::ID = 0x100;

// public
Throttlecommand100::Throttlecommand100() { Reset(); }

void Throttlecommand100::UpdateData(double vel_target, double throttle_acc, int checksum_100, double throttle_pedal_target, int throttle_en_ctrl) {
  set_p_vel_target(vel_target);
  set_p_throttle_acc(throttle_acc);
  set_p_checksum_100(checksum_100);
  set_p_throttle_pedal_target(throttle_pedal_target);
  set_p_throttle_en_ctrl(throttle_en_ctrl);
}

void Throttlecommand100::Reset() {
  for(uint i=0;i<8;i++)
  {
    data[i] = 0;
  }
}

uint8_t * Throttlecommand100::get_data()
{
  return data;
}


// config detail: {'bit': 47, 'is_signed_var': False, 'len': 10, 'name': 'Vel_Target', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|10.23]', 'physical_unit': 'm/s', 'precision': 0.01, 'type': 'double'}
void Throttlecommand100::set_p_vel_target(double vel_target) {
  //vel_target = ProtocolData::BoundedValue(0.0, 10.23, vel_target);
  int x = vel_target / 0.010000;
  uint8_t t = 0;
  uint8_t a = 0;

  t = x & 0x3;
  Byte to_set0(a);
  to_set0.set_value(t, 6, 2);
  data[6] += to_set0.return_byte_t();
  x >>= 2;

  t = x & 0xFF;
  Byte to_set1(a);
  to_set1.set_value(t, 0, 8);
  data[5] += to_set1.return_byte_t();
}


// config detail: {'bit': 15, 'is_signed_var': False, 'len': 10, 'name': 'Throttle_Acc', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|10]', 'physical_unit': 'm/s^2', 'precision': 0.01, 'type': 'double'}
void Throttlecommand100::set_p_throttle_acc(double throttle_acc) {
  //throttle_acc = ProtocolData::BoundedValue(0.0, 10.0, throttle_acc);
  int x = throttle_acc / 0.010000;
  uint8_t t = 0;
  uint8_t a = 0;

  t = x & 0x3;
  Byte to_set0(a);
  to_set0.set_value(t, 6, 2);
  data[2] += to_set0.return_byte_t();
  x >>= 2;

  t = x & 0xFF;
  Byte to_set1(a);
  to_set1.set_value(t, 0, 8);
  data[1] += to_set1.return_byte_t();
}


// config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'CheckSum_100', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Throttlecommand100::set_p_checksum_100(int checksum_100) {
  //checksum_100 = ProtocolData::BoundedValue(0, 255, checksum_100);
  int x = checksum_100;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 8);
  data[7] += to_set.return_byte_t();
}


// config detail: {'bit': 31, 'is_signed_var': False, 'len': 16, 'name': 'Throttle_Pedal_Target', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
void Throttlecommand100::set_p_throttle_pedal_target(double throttle_pedal_target) {
  //throttle_pedal_target = ProtocolData::BoundedValue(0.0, 100.0, throttle_pedal_target);
  int x = throttle_pedal_target / 0.100000;
  uint8_t t = 0;
  uint8_t a = 0;

  t = x & 0xFF;
  Byte to_set0(a);
  to_set0.set_value(t, 0, 8);
  data[4] += to_set0.return_byte_t();
  x >>= 8;

  t = x & 0xFF;
  Byte to_set1(a);
  to_set1.set_value(t, 0, 8);
  data[3] += to_set1.return_byte_t();
}


// config detail: {'bit': 0, 'enum': {0: 'THROTTLE_EN_CTRL_DISABLE', 1: 'THROTTLE_EN_CTRL_ENABLE'}, 'is_signed_var': False, 'len': 1, 'name': 'Throttle_EN_CTRL', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Throttlecommand100::set_p_throttle_en_ctrl(int throttle_en_ctrl) {
  int x = throttle_en_ctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 1);
  data[0] += to_set.return_byte_t();
}
