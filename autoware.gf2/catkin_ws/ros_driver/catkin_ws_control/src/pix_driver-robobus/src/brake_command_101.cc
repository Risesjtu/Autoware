#include "brake_command_101.hpp"
#include "ros/ros.h"

int32_t Brakecommand101::ID = 0x101;

// public
Brakecommand101::Brakecommand101() { Reset(); }

void Brakecommand101::Reset()
{
  for(uint i=0;i<8;i++)
  {
    data[i] = 0;
  }
}

uint8_t * Brakecommand101::get_data()
{
  return data;
}

void Brakecommand101::UpdateData(int aeb_en_ctrl, double brake_dec, int checksum_101, double brake_pedal_target, int brake_en_ctrl) {
  set_p_aeb_en_ctrl(aeb_en_ctrl);
  set_p_brake_dec(brake_dec);
  set_p_checksum_101(checksum_101);
  set_p_brake_pedal_target(brake_pedal_target);
  set_p_brake_en_ctrl(brake_en_ctrl);
}


// config detail: {'bit': 1, 'enum': {0: 'AEB_EN_CTRL_DISABLE_AEB', 1: 'AEB_EN_CTRL_ENABLE_AEB'}, 'is_signed_var': False, 'len': 1, 'name': 'AEB_EN_CTRL', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|0]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Brakecommand101::set_p_aeb_en_ctrl(int aeb_en_ctrl) {
  // int x = aeb_en_ctrl;
  // uint8_t a = 2;
  // // 类实例化一个参数 相当于有参数的构造函数
  // Byte to_set(a);
  // // 打印中文字符，不乱码
  // // setlocale(LC_ALL, "");
  // // ROS_INFO("私有属性=%d", to_set.return_byte_t());
  // // 开始计算，计算结果赋予给私有属性
  // to_set.set_value(x, 1, 1);
  // // 获取私有属性
  // data[0] += to_set.return_byte_t();
  if (aeb_en_ctrl == 1)
  {
    data[0] += 2;
  }
  else
  {
    data[0] += 0;
  }
}

// config detail: {'bit': 15, 'is_signed_var': False, 'len': 10, 'name': 'Brake_Dec', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|10]', 'physical_unit': 'm/s^2', 'precision': 0.01, 'type': 'double'}
void Brakecommand101::set_p_brake_dec(double brake_dec) {
  // brake_dec = ProtocolData::BoundedValue(0.0, 10.0, brake_dec);
  int x = brake_dec / 0.010000;
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


// config detail: {'bit': 63, 'is_signed_var': False, 'len': 8, 'name': 'CheckSum_101', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Brakecommand101::set_p_checksum_101(int checksum_101) {
  // checksum_101 = ProtocolData::BoundedValue(0, 255, checksum_101);
  int x = checksum_101;
  uint8_t a = 0;
  Byte to_set(a);
  to_set.set_value(x, 0, 8);
  data[7] += to_set.return_byte_t();
}


// config detail: {'bit': 31, 'is_signed_var': False, 'len': 16, 'name': 'Brake_Pedal_Target', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 0.1, 'type': 'double'}
void Brakecommand101::set_p_brake_pedal_target(double brake_pedal_target) {
  // brake_pedal_target = ProtocolData::BoundedValue(0.0, 100.0, brake_pedal_target);
  int x = brake_pedal_target / 0.100000;
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


// config detail: {'bit': 0, 'enum': {0: 'BRAKE_EN_CTRL_DISABLE', 1: 'BRAKE_EN_CTRL_ENABLE'}, 'is_signed_var': False, 'len': 1, 'name': 'Brake_EN_CTRL', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|1]', 'physical_unit': '', 'precision': 1.0, 'type': 'enum'}
void Brakecommand101::set_p_brake_en_ctrl(int brake_en_ctrl) {
  int x = brake_en_ctrl;

  uint8_t a = 0;

  Byte to_set(a);
  to_set.set_value(x, 0, 1);
  data[0] += to_set.return_byte_t();
  // ROS_INFO("data[0]=%d", data[0]);
}
