#pragma once

#include "Byte.hpp"

class Vehiclemodecommand105{
 public:
  static int32_t ID;

  Vehiclemodecommand105();

  void UpdateData(int checksum_105, int turn_light_ctrl, int vin_req, int drive_mode_ctrl, int steer_mode_ctrl);

  void Reset();
  uint8_t * get_data();
 private:

  // config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'CheckSum_105', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_checksum_105(int checksum_105);

  // config detail: {'bit': 17, 'enum': {0: 'TURN_LIGHT_CTRL_TURNLAMP_OFF', 1: 'TURN_LIGHT_CTRL_LEFT_TURNLAMP_ON', 2: 'TURN_LIGHT_CTRL_RIGHT_TURNLAMP_ON', 3: 'TURN_LIGHT_CTRL_HAZARD_WARNING_LAMPSTS_ON'}, 'is_signed_var': False, 'len': 2, 'name': 'Turn_Light_CTRL', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_turn_light_ctrl(int turn_light_ctrl);

  // config detail: {'bit': 24, 'enum': {0: 'VIN_REQ_VIN_REQ_DISABLE', 1: 'VIN_REQ_VIN_REQ_ENABLE'}, 'is_signed_var': False, 'len': 1, 'name': 'VIN_Req', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_vin_req(int vin_req);

  // config detail: {'bit': 10, 'enum': {0: 'DRIVE_MODE_CTRL_THROTTLE_PADDLE_DRIVE', 1: 'DRIVE_MODE_CTRL_SPEED_DRIVE'}, 'is_signed_var': False, 'len': 3, 'name': 'Drive_Mode_CTRL', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_drive_mode_ctrl(int drive_mode_ctrl);

  // config detail: {'bit': 2, 'enum': {0: 'STEER_MODE_CTRL_STANDARD_STEER', 1: 'STEER_MODE_CTRL_NON_DIRECTION_STEER', 2: 'STEER_MODE_CTRL_SYNC_DIRECTION_STEER'}, 'is_signed_var': False, 'len': 3, 'name': 'Steer_Mode_CTRL', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
  void set_p_steer_mode_ctrl(int steer_mode_ctrl);

 private:
  uint8_t data[8];
};


