#pragma once
#include <cstdint>
#include <cstddef>
#include "esp_err.h"
typedef struct i2c_bus_t* i2c_master_bus_handle_t;
typedef struct i2c_dev_t* i2c_master_dev_handle_t;
typedef enum { I2C_ADDR_BIT_LEN_7 = 0 } i2c_addr_bit_len_t;
typedef struct { i2c_addr_bit_len_t dev_addr_length; uint16_t device_address; uint32_t scl_speed_hz; } i2c_device_config_t;
inline esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t, const i2c_device_config_t*, i2c_master_dev_handle_t*){return -1;}
inline esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t){return ESP_OK;}
inline esp_err_t i2c_master_transmit(i2c_master_dev_handle_t, const uint8_t*, size_t, int){return -1;}
inline esp_err_t i2c_master_probe(i2c_master_bus_handle_t, uint16_t, int){return -1;}
