#include "../include/bms_report_512.hpp"
#include "../include/Byte.hpp"

Bmsreport512::Bmsreport512()
{
}
int Bmsreport512::ID = 0x512;

void Bmsreport512::update_bytes(uint8_t bytes_data[8])
{
  for(uint i=0;i<8;i++)
  {
    bytes[i] = bytes_data[i];
  }
}

void Bmsreport512::Parse()
{
  battery_soc = decode_battery_soc();
  battery_current = decode_battery_current();
  battery_votage =  decode_battery_voltage();
}

// config detail: {'bit': 23, 'description': 'Battery Total Current', 'is_signed_var': False, 'len': 16, 'name': 'battery_current', 'offset': -3200.0, 'order': 'motorola', 'physical_range': '[-3200|3353.5]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
double Bmsreport512::decode_battery_current()
{
  Byte t0(*(bytes + 2));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 3));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.100000 + -3200.000000;
  return ret;
}

// config detail: {'bit': 7, 'description': 'Battery Total Voltage', 'is_signed_var': False, 'len': 16, 'name': 'battery_voltage', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|300]', 'physical_unit': 'V', 'precision': 0.01, 'type': 'double'}
double Bmsreport512::decode_battery_voltage()
{
  Byte t0(*(bytes + 0));
  int32_t x = t0.get_byte(0, 8);

  Byte t1(*(bytes + 1));
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  double ret = x * 0.010000;
  return ret;
}

// config detail: {'bit': 39, 'description': 'Battery Soc percentage', 'is_signed_var': False, 'len': 8, 'name': 'battery_soc', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 1.0, 'type': 'int'}
int Bmsreport512::decode_battery_soc()
{
  Byte t0(*(bytes + 4));
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
