#include "park_command_104.hpp"

int32_t Parkcommand104::ID = 0x104;

// public
Parkcommand104::Parkcommand104() { Reset(); }

void Parkcommand104::UpdateData(int checksum_104, int park_target, int park_en_ctrl) {
  set_p_checksum_104(checksum_104);
  set_p_park_target(park_target);
  set_p_park_en_ctrl(park_en_ctrl);
}

void Parkcommand104::Reset() {
  for(uint8_t i=0;i<8;i++)
  {
    data[i] = 0;
  }
}
uint8_t * Parkcommand104::get_data()
{
  return data;
}


// config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'CheckSum_104', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Parkcommand104::set_p_checksum_104(int checksum_104) {
  // checksum_104 = ProtocolData::BoundedValue(0, 255, checksum_104);
  int x = checksum_104;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 8);
  data[7] += to_set.return_byte_t();
}


// config detail: {'bit': 8, 'enum': {0: 'PARK_TARGET_RELEASE', 1: 'PARK_TARGET_PARKING_TRIGGER'}, 'is_signed_var': False, 'len': 1, 'name': 'Park_Target', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Parkcommand104::set_p_park_target(int park_target) {
  int x = park_target;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 1);
  data[1] += to_set.return_byte_t();
}


// config detail: {'bit': 0, 'enum': {0: 'PARK_EN_CTRL_DISABLE', 1: 'PARK_EN_CTRL_ENABLE'}, 'is_signed_var': False, 'len': 1, 'name': 'Park_EN_CTRL', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Parkcommand104::set_p_park_en_ctrl(int park_en_ctrl) {
  int x = park_en_ctrl;
  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 1);
  data[0] += to_set.return_byte_t();
}
