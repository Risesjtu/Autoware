#include "vcu_report_505.hpp"

Vcureport505::Vcureport505() {}
int32_t Vcureport505::ID = 0x505;

void Vcureport505::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void Vcureport505::Parse(){
  brake_light_actual = decode_brake_light_actual();
  turn_light_actual = decode_turn_light_actual();
  chassis_errcode = decode_chassis_errcode();
  drive_mode_sts = decode_drive_mode_sts();
  steer_mode_sts = decode_steer_mode_sts();
  vehicle_mode_state = decode_vehicle_mode_state();
  frontcrash_state = decode_frontcrash_state();
  backcrash_state = decode_backcrash_state();
  aeb_state = decode_aeb_state();
  acc = decode_acc();
  speed = decode_speed();
}

// config detail: {'bit': 11, 'enum': {0: 'BRAKE_LIGHT_ACTUAL_BRAKELIGHT_OFF', 1: 'BRAKE_LIGHT_ACTUAL_BRAKELIGHT_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'brake_light_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Vcureport505::decode_brake_light_actual() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(3, 1);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 57, 'enum': {0: 'TURN_LIGHT_ACTUAL_TURNLAMPSTS_OFF', 1: 'TURN_LIGHT_ACTUAL_LEFT_TURNLAMPSTS_ON', 2: 'TURN_LIGHT_ACTUAL_RIGHT_TURNLAMPSTS_ON', 3: 'TURN_LIGHT_ACTUAL_HAZARD_WARNING_LAMPSTS_ON'}, 'is_signed_var': False, 'len': 2, 'name': 'turn_light_actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Vcureport505::decode_turn_light_actual() {
  Byte t0(*(bytes + 7));
  int32_t x = t0.get_byte(0, 2);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 47, 'is_signed_var': False, 'len': 8, 'name': 'chassis_errcode', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
int Vcureport505::decode_chassis_errcode() {
  Byte t0(*(bytes + 5));
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

// config detail: {'bit': 39, 'enum': {0: 'DRIVE_MODE_STS_THROTTLE_PADDLE_DRIVE_MODE', 1: 'DRIVE_MODE_STS_SPEED_DRIVE_MODE'}, 'is_signed_var': False, 'len': 3, 'name': 'drive_mode_sts', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Vcureport505::decode_drive_mode_sts() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(5, 3);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 10, 'enum': {0: 'STEER_MODE_STS_STANDARD_STEER_MODE', 1: 'STEER_MODE_STS_NON_DIRECTION_STEER_MODE', 2: 'STEER_MODE_STS_SYNC_DIRECTION_STEER_MODE'}, 'is_signed_var': False, 'len': 3, 'name': 'steer_mode_sts', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Vcureport505::decode_steer_mode_sts() {
  Byte t0(*(bytes + 1));
  int32_t x = t0.get_byte(0, 3);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 36, 'enum': {0: 'VEHICLE_MODE_STATE_MANUAL_REMOTE_MODE', 1: 'VEHICLE_MODE_STATE_AUTO_MODE', 2: 'VEHICLE_MODE_STATE_EMERGENCY_MODE', 3: 'VEHICLE_MODE_STATE_STANDBY_MODE'}, 'is_signed_var': False, 'len': 2, 'name': 'vehicle_mode_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Vcureport505::decode_vehicle_mode_state() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(3, 2);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 33, 'enum': {0: 'FRONTCRASH_STATE_NO_EVENT', 1: 'FRONTCRASH_STATE_CRASH_EVENT'}, 'is_signed_var': False, 'len': 1, 'name': 'frontcrash_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Vcureport505::decode_frontcrash_state() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(1, 1);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 34, 'enum': {0: 'BACKCRASH_STATE_NO_EVENT', 1: 'BACKCRASH_STATE_CRASH_EVENT'}, 'is_signed_var': False, 'len': 1, 'name': 'backcrash_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Vcureport505::decode_backcrash_state() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(2, 1);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 32, 'enum': {0: 'AEB_STATE_INACTIVE', 1: 'AEB_STATE_ACTIVE'}, 'is_signed_var': False, 'len': 1, 'name': 'aeb_state', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
int Vcureport505::decode_aeb_state() {
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(0, 1);

  int ret =  static_cast<int>(x);
  return ret;
}

// config detail: {'bit': 7, 'is_signed_var': True, 'len': 12, 'name': 'acc', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-10|10]', 'physical_unit': 'm/s^2', 'precision': 0.01, 'type': 'double'}
double Vcureport505::decode_acc() {
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 1));
  int32_t t = t1.get_byte(4, 4);
  x <<= 4;
  x |= t;

  x <<= 20;
  x >>= 20;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'bit': 23, 'is_signed_var': True, 'len': 16, 'name': 'speed', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-32.768|32.767]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
double Vcureport505::decode_speed() {
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 3));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.001000;
  return ret;
}
