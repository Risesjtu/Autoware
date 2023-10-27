#include "steering_command_102.hpp"

int32_t Steeringcommand102::ID = 0x102;

// public
Steeringcommand102::Steeringcommand102() { Reset(); }

void Steeringcommand102::UpdateData(int steer_en_ctrl, int steer_angle_target, int steer_angle_spd, int checksum_102) {
  set_p_steer_en_ctrl(steer_en_ctrl);
  set_p_steer_angle_target(steer_angle_target);
  set_p_steer_angle_spd(steer_angle_spd);
  set_p_checksum_102(checksum_102);
}

void Steeringcommand102::Reset() {
  // TODO(All) :  you should check this manually
  for(uint8_t i=0;i<8;i++)
  {
    data[i] = 0;
  }
}

uint8_t * Steeringcommand102::get_data()
{
  return data;
}


// config detail: {'bit': 0, 'enum': {0: 'STEER_EN_CTRL_DISABLE', 1: 'STEER_EN_CTRL_ENABLE'}, 'is_signed_var': False, 'len': 1, 'name': 'Steer_EN_CTRL', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Steeringcommand102::set_p_steer_en_ctrl(int steer_en_ctrl) {
  int x = steer_en_ctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 1);
  data[0] += to_set.return_byte_t();
}


// config detail: {'bit': 31, 'is_signed_var': False, 'len': 16, 'name': 'Steer_ANGLE_Target', 'offset': -500.0, 'order': 'motorola', 'physical_range': '[-360|360]', 'physical_unit': 'deg', 'precision': 1.0, 'type': 'int'}
void Steeringcommand102::set_p_steer_angle_target(int steer_angle_target) {
  //steer_angle_target = ProtocolData::BoundedValue(-360, 360, steer_angle_target);
  int x = (steer_angle_target - -500.000000);
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


// config detail: {'bit': 15, 'is_signed_var': False, 'len': 8, 'name': 'Steer_ANGLE_SPD', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|250]', 'physical_unit': 'deg/s', 'precision': 1.0, 'type': 'int'}
void Steeringcommand102::set_p_steer_angle_spd(int steer_angle_spd) {
  //steer_angle_spd = ProtocolData::BoundedValue(0, 250, steer_angle_spd);
  int x = steer_angle_spd;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 8);
  data[1] += to_set.return_byte_t();
}


// config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'CheckSum_102', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Steeringcommand102::set_p_checksum_102(int checksum_102) {
  //checksum_102 = ProtocolData::BoundedValue(0, 255, checksum_102);
  int x = checksum_102;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 8);
  data[7] += to_set.return_byte_t();
}

