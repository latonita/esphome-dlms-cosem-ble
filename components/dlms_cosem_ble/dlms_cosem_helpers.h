#pragma once

#include <string>
#include <dlmssettings.h>

namespace esphome {
namespace dlms_cosem_ble {

float dlms_data_as_float(DLMS_DATA_TYPE value_type, const uint8_t *value_buffer_ptr, uint8_t value_length);
std::string dlms_datetime_as_string(const uint8_t *value_buffer_ptr, uint8_t value_length);
std::string dlms_data_as_string(DLMS_DATA_TYPE value_type, const uint8_t *value_buffer_ptr, uint8_t value_length);

const char *dlms_data_type_to_string(DLMS_DATA_TYPE vt);
const char *dlms_error_to_string(int error);

}  // namespace dlms_cosem
}  // namespace esphome
