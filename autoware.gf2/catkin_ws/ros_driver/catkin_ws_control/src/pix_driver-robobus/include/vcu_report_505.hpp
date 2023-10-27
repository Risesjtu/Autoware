#pragma once

#include "Byte.hpp"



class Vcureport505{
 public:
  static int32_t ID;
  Vcureport505();
  void Parse();
  void update_bytes(uint8_t bytes_data[8]);
  int brake_light_actual;
  int turn_light_actual;
  int chassis_errcode;
  int drive_mode_sts;
  int steer_mode_sts;
  int vehicle_mode_state;
  int frontcrash_state;
  int backcrash_state;
  int aeb_state;
  double acc;
  double speed;

 private:
  uint8_t bytes[8];


  // config detail: {'bit': 11, 'enum': {0: 'BRAKE_LIGHT_ACTUAL_BRAKELIGHT_OFF', 1: 'BRAKE_LIGHT_ACTUAL_BRAKELIGHT_ON'}, 'is_signed_var': False, 'len': 1, 'name': 'Brake_Light_Actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int decode_brake_light_actual();

  // config detail: {'bit': 57, 'enum': {0: 'TURN_LIGHT_ACTUAL_TURNLAMPSTS_OFF', 1: 'TURN_LIGHT_ACTUAL_LEFT_TURNLAMPSTS_ON', 2: 'TURN_LIGHT_ACTUAL_RIGHT_TURNLAMPSTS_ON', 3: 'TURN_LIGHT_ACTUAL_HAZARD_WARNING_LAMPSTS_ON'}, 'is_signed_var': False, 'len': 2, 'name': 'Turn_Light_Actual', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int decode_turn_light_actual();

  // config detail: {'bit': 47, 'is_signed_var': False, 'len': 8, 'name': 'Chassis_errcode', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  int decode_chassis_errcode();

  // config detail: {'bit': 39, 'enum': {0: 'DRIVE_MODE_STS_THROTTLE_PADDLE_DRIVE_MODE', 1: 'DRIVE_MODE_STS_SPEED_DRIVE_MODE'}, 'is_signed_var': False, 'len': 3, 'name': 'Drive_Mode_STS', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int decode_drive_mode_sts();

  // config detail: {'bit': 10, 'enum': {0: 'STEER_MODE_STS_STANDARD_STEER_MODE', 1: 'STEER_MODE_STS_NON_DIRECTION_STEER_MODE', 2: 'STEER_MODE_STS_SYNC_DIRECTION_STEER_MODE'}, 'is_signed_var': False, 'len': 3, 'name': 'Steer_Mode_STS', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int decode_steer_mode_sts();

  // config detail: {'bit': 36, 'enum': {0: 'VEHICLE_MODE_STATE_MANUAL_REMOTE_MODE', 1: 'VEHICLE_MODE_STATE_AUTO_MODE', 2: 'VEHICLE_MODE_STATE_EMERGENCY_MODE', 3: 'VEHICLE_MODE_STATE_STANDBY_MODE'}, 'is_signed_var': False, 'len': 2, 'name': 'Vehicle_Mode_State', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int decode_vehicle_mode_state();

  // config detail: {'bit': 33, 'enum': {0: 'FRONTCRASH_STATE_NO_EVENT', 1: 'FRONTCRASH_STATE_CRASH_EVENT'}, 'is_signed_var': False, 'len': 1, 'name': 'FrontCrash_State', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int decode_frontcrash_state();

  // config detail: {'bit': 34, 'enum': {0: 'BACKCRASH_STATE_NO_EVENT', 1: 'BACKCRASH_STATE_CRASH_EVENT'}, 'is_signed_var': False, 'len': 1, 'name': 'BackCrash_State', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int decode_backcrash_state();

  // config detail: {'bit': 32, 'enum': {0: 'AEB_STATE_INACTIVE', 1: 'AEB_STATE_ACTIVE'}, 'is_signed_var': False, 'len': 1, 'name': 'AEB_State', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  int decode_aeb_state();

  // config detail: {'bit': 7, 'is_signed_var': True, 'len': 12, 'name': 'ACC', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-10|10]', 'physical_unit': 'm/s^2', 'precision': 0.01, 'type': 'double'}
  double decode_acc();

  // config detail: {'bit': 23, 'is_signed_var': True, 'len': 16, 'name': 'SPEED', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[-32.768|32.767]', 'physical_unit': 'm/s', 'precision': 0.001, 'type': 'double'}
  double decode_speed();
};