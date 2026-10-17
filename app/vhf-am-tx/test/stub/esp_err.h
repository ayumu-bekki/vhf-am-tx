#pragma once
typedef int esp_err_t;
#ifndef ESP_OK
#define ESP_OK 0
#endif
inline const char* esp_err_to_name(esp_err_t){return "";}
#ifndef ESP_ERR_NOT_FOUND
#define ESP_ERR_NOT_FOUND 0x105
#endif
