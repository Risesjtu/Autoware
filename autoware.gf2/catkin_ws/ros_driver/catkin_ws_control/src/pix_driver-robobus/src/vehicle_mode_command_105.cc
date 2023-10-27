#include "vehicle_mode_command_105.hpp"

int32_t Vehiclemodecommand105::ID = 0x105;

// public
Vehiclemodecommand105::Vehiclemodecommand105() { Reset(); }


void Vehiclemodecommand105::UpdateData(int checksum_105, int turn_light_ctrl, int vin_req, int drive_mode_ctrl, int steer_mode_ctrl) {
  set_p_checksum_105(checksum_105);
  set_p_turn_light_ctrl(turn_light_ctrl);
  set_p_vin_req(vin_req);
  set_p_drive_mode_ctrl(drive_mode_ctrl);
  set_p_steer_mode_ctrl(steer_mode_ctrl);
}

void Vehiclemodecommand105::Reset() {
  for(uint i=0;i<8;i++)
  {
    data[i] = 0;
  }
}

uint8_t * Vehiclemodecommand105::get_data()
{
  return data;
}


// config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'CheckSum_105', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Vehiclemodecommand105::set_p_checksum_105(int checksum_105) {
  //checksum_105 = ProtocolData::BoundedValue(0, 255, checksum_105);
  int x = checksum_105;
  uint8_t a =0;

  Byte to_set(a);
  to_set.set_value(x, 0, 8);
  data[7] += to_set.return_byte_t();
}


// config detail: {'bit': 17, 'enum': {0: 'TURN_LIGHT_CTRL_TURNLAMP_OFF', 1: 'TURN_LIGHT_CTRL_LEFT_TURNLAMP_ON', 2: 'TURN_LIGHT_CTRL_RIGHT_TURNLAMP_ON', 3: 'TURN_LIGHT_CTRL_HAZARD_WARNING_LAMPSTS_ON'}, 'is_signed_var': False, 'len': 2, 'name': 'Turn_Light_CTRL', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Vehiclemodecommand105::set_p_turn_light_ctrl(int turn_light_ctrl) {
  int x = turn_light_ctrl;
  uint8_t a =0;

  Byte to_set(a);
  to_set.set_value(x, 0, 2);
  data[2] += to_set.return_byte_t();
}


// config detail: {'bit': 24, 'enum': {0: 'VIN_REQ_VIN_REQ_DISABLE', 1: 'VIN_REQ_VIN_REQ_ENABLE'}, 'is_signed_var': False, 'len': 1, 'name': 'VIN_Req', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Vehiclemodecommand105::set_p_vin_req(int vin_req) {
  int x = vin_req;
  uint8_t a =0;

  Byte to_set(a);
  to_set.set_value(x, 0, 1);
  data[3] += to_set.return_byte_t();
}


// config detail: {'bit': 10, 'enum': {0: 'DRIVE_MODE_CTRL_THROTTLE_PADDLE_DRIVE', 1: 'DRIVE_MODE_CTRL_SPEED_DRIVE'}, 'is_signed_var': False, 'len': 3, 'name': 'Drive_Mode_CTRL', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Vehiclemodecommand105::set_p_drive_mode_ctrl(int drive_mode_ctrl) {
  int x = drive_mode_ctrl;
  uint8_t a =0;

  Byte to_set(a);
  to_set.set_value(x, 0, 3);
  data[1] += to_set.return_byte_t();
}


// config detail: {'bit': 2, 'enum': {0: 'STEER_MODE_CTRL_STANDARD_STEER', 1: 'STEER_MODE_CTRL_NON_DIRECTION_STEER', 2: 'STEER_MODE_CTRL_SYNC_DIRECTION_STEER'}, 'is_signed_var': False, 'len': 3, 'name': 'Steer_Mode_CTRL', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|7]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Vehiclemodecommand105::set_p_steer_mode_ctrl(int steer_mode_ctrl) {
  int x = steer_mode_ctrl;
  uint8_t a =0;

  Byte to_set(a);
  to_set.set_value(x, 0, 3);
  data[0] += to_set.return_byte_t();
}
