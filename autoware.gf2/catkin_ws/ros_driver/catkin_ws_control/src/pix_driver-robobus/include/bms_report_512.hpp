#pragma once

#include <iostream>

class Bmsreport512{
    public:
        static int32_t ID;
        Bmsreport512();
        void update_bytes(uint8_t bytes_data[8]);
        void Parse();
        double battery_current;
        double battery_votage;
        int battery_soc;

    private:
        uint8_t bytes[8];

        // config detail: {'bit': 23, 'description': 'Battery Total Current', 'is_signed_var': False, 'len': 16, 'name': 'Battery_Current', 'offset': -3200.0, 'order': 'motorola', 'physical_range': '[-3200|3353.5]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
        double decode_battery_current();

        // config detail: {'bit': 7, 'description': 'Battery Total Voltage', 'is_signed_var': False, 'len': 16, 'name': 'Battery_Voltage', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|300]', 'physical_unit': 'V', 'precision': 0.01, 'type': 'double'}
        double decode_battery_voltage();

        // config detail: {'bit': 39, 'description': 'Battery Soc percentage', 'is_signed_var': False, 'len': 8, 'name': 'Battery_Soc', 'offset': 0.0, 'order': 'motorola', 'physical_range': '[0|100]', 'physical_unit': '%', 'precision': 1.0, 'type': 'int'}
        int decode_battery_soc();
};


