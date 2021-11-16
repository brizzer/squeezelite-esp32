#pragma once
#include "network_manager.h"
#include "cJSON.h"
#ifdef __cplusplus
extern "C" {

#endif
char* wifi_manager_alloc_get_ip_info_json();
/**
 * @brief Tries to get access to json buffer mutex.
 *
 * The HTTP server can try to access the json to serve clients while the wifi manager thread can try
 * to update it. These two tasks are synchronized through a mutex.
 *
 * The mutex is used by both the access point list json and the connection status json.\n
 * These two resources should technically have their own mutex but we lose some flexibility to save
 * on memory.
 *
 * This is a simple wrapper around freeRTOS function xSemaphoreTake.
 *
 * @param xTicksToWait The time in ticks to wait for the semaphore to become available.
 * @return true in success, false otherwise.
 */
bool wifi_manager_lock_json_buffer(TickType_t xTicksToWait);

/**
 * @brief Releases the json buffer mutex.
 */
void wifi_manager_unlock_json_buffer();

bool wifi_manager_lock_sta_ip_string(TickType_t xTicksToWait);
void wifi_manager_unlock_sta_ip_string();

/**
 * @brief gets the string representation of the STA IP address, e.g.: "192.168.1.69"
 */
char* wifi_manager_get_sta_ip_string();

/**
 * @brief thread safe char representation of the STA IP update
 */
void wifi_manager_safe_update_sta_ip_string(esp_ip4_addr_t * ip4);

/**
 * @brief Generates the connection status json: ssid and IP addresses.
 * @note This is not thread-safe and should be called only if wifi_manager_lock_json_buffer call is successful.
 */
void wifi_manager_generate_ip_info_json(update_reason_code_t update_reason_code);

void init_network_status();
void destroy_network_status();
cJSON* wifi_manager_get_basic_info(cJSON** old);

void wifi_manager_update_basic_info();
void network_status_clear_ip();
void wifi_manager_safe_reset_sta_ip_string();
#ifdef __cplusplus
}
#endif