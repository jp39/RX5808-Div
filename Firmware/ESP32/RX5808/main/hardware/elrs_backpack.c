#include "elrs_backpack.h"
#include "elrs_msp.h"
#include "elrs_config.h"
#include "rx5808.h"
#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"

static const char *TAG = "ELRS_BP";

// Broadcast MAC address for binding mode
static const uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Current UID (loaded from NVS or received during binding)
static uint8_t elrs_uid[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Binding state
static elrs_bind_state_t binding_state = ELRS_STATE_UNBOUND;
static elrs_bind_state_t previous_state = ELRS_STATE_UNBOUND;
static uint8_t pending_uid[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static TimerHandle_t binding_timer = NULL;
static uint32_t binding_timeout_ms = 0;
static uint32_t binding_start_time = 0;

// ESP-NOW message structure
typedef struct {
    uint8_t mac_addr[6];
    uint8_t data[250];
    uint8_t data_len;
} espnow_msg_t;

// Task and queue handles
static QueueHandle_t espnow_queue = NULL;
static TaskHandle_t backpack_task_handle = NULL;

// MSP parser instance
static msp_parser_t msp_parser;

// Forward declarations
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);

/**
 * @brief Binding timeout timer callback
 */
static void binding_timeout_callback(TimerHandle_t xTimer) {
    ESP_LOGW(TAG, "Binding timeout");
    binding_state = ELRS_STATE_BIND_TIMEOUT;
    // Timer will be deleted by the binding cancel function
}

/**
 * @brief Reinitialize ESP-NOW with new UID
 */
static bool reinit_espnow_with_uid(const uint8_t uid[6]) {
    esp_err_t ret;

    ESP_LOGI(TAG, "Reinitializing with UID: %02X:%02X:%02X:%02X:%02X:%02X",
             uid[0], uid[1], uid[2], uid[3], uid[4], uid[5]);

    // Deinitialize ESP-NOW first
    esp_now_deinit();

    // Stop WiFi
    ret = esp_wifi_stop();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop WiFi: %s", esp_err_to_name(ret));
        return false;
    }

    // Deinitialize WiFi completely
    ret = esp_wifi_deinit();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to deinit WiFi: %s", esp_err_to_name(ret));
        return false;
    }

    // Re-initialize WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init WiFi: %s", esp_err_to_name(ret));
        return false;
    }

    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi mode: %s", esp_err_to_name(ret));
        return false;
    }

    ret = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi storage: %s", esp_err_to_name(ret));
        return false;
    }

    // Now set the MAC address (WiFi must be initialized but not started)
    ret = esp_wifi_set_mac(WIFI_IF_STA, uid);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set MAC: %s", esp_err_to_name(ret));
        // Try to recover by restarting with old settings
        esp_wifi_start();
        esp_now_init();
        esp_now_register_recv_cb(espnow_recv_cb);
        return false;
    }

    ESP_LOGI(TAG, "MAC set to: %02X:%02X:%02X:%02X:%02X:%02X",
             uid[0], uid[1], uid[2], uid[3], uid[4], uid[5]);

    // Start WiFi
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(ret));
        return false;
    }

    // Set channel
    ret = esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set channel: %s", esp_err_to_name(ret));
    }

    // Reinitialize ESP-NOW
    ret = esp_now_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init ESP-NOW: %s", esp_err_to_name(ret));
        return false;
    }

    // Register receive callback again
    ret = esp_now_register_recv_cb(espnow_recv_cb);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register recv callback: %s", esp_err_to_name(ret));
        return false;
    }

    // Add new peer
    esp_now_peer_info_t peer_info = {0};
    memcpy(peer_info.peer_addr, uid, 6);
    peer_info.channel = 1;
    peer_info.ifidx = WIFI_IF_STA;
    peer_info.encrypt = false;

    ret = esp_now_add_peer(&peer_info);
    if (ret != ESP_OK && ret != ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGE(TAG, "Failed to add peer: %s", esp_err_to_name(ret));
        return false;
    }

    // Update current UID
    memcpy(elrs_uid, uid, 6);

    ESP_LOGI(TAG, "ESP-NOW reinitialized successfully");
    return true;
}

/**
 * @brief Complete the binding process
 */
static void complete_binding(void) {
    // Save UID to NVS
    if (!elrs_config_save_uid(pending_uid)) {
        ESP_LOGE(TAG, "Failed to save UID to NVS");
        binding_state = ELRS_STATE_BIND_TIMEOUT;
        return;
    }

    // Stop binding timer
    if (binding_timer != NULL) {
        xTimerStop(binding_timer, 0);
        xTimerDelete(binding_timer, 0);
        binding_timer = NULL;
    }

    // Reinitialize ESP-NOW with new UID
    if (!reinit_espnow_with_uid(pending_uid)) {
        ESP_LOGE(TAG, "Failed to reinitialize ESP-NOW");
        binding_state = ELRS_STATE_BIND_TIMEOUT;
        return;
    }

    // Update state
    binding_state = ELRS_STATE_BIND_SUCCESS;
    ESP_LOGI(TAG, "Binding completed successfully");

    // After a short delay, transition to BOUND state
    vTaskDelay(pdMS_TO_TICKS(1000));
    binding_state = ELRS_STATE_BOUND;
}

/**
 * @brief Process MSP_SET_VTX_CONFIG command
 *
 * ELRS backpack sends channel index (0-47) in payload[0]
 * Additional bytes may contain power, pit mode, etc. (ignored for now)
 */
static void handle_msp_set_vtx_config(const uint8_t *payload, uint16_t len) {
    if (len < 1) {
        ESP_LOGW(TAG, "MSP_SET_VTX_CONFIG: empty payload");
        return;
    }

    // Channel index is always in first byte (0-47)
    uint8_t channel_index = payload[0];

    if (channel_index > 47) {
        ESP_LOGW(TAG, "Invalid channel index: %d", channel_index);
        return;
    }

    // Calculate band and channel for display
    uint8_t band = channel_index / 8;
    uint8_t channel = channel_index % 8;

    ESP_LOGI(TAG, "MSP_SET_VTX_CONFIG: index=%d (%c%d)",
             channel_index, Rx5808_ChxMap[band], channel + 1);

    // Call RX5808 channel set function (thread-safe with mutex)
    Rx5808_Set_Channel(channel_index);
}

/**
 * @brief Feed a byte to the MSP parser state machine
 *
 * @return true if complete packet received, false otherwise
 */
static bool msp_parser_feed_byte(msp_parser_t *parser, uint8_t byte) {
    switch (parser->state) {
        case MSP_IDLE:
            if (byte == MSP_V2_HEADER_START) {
                parser->state = MSP_HEADER_START;
                parser->checksum = 0;
            }
            break;

        case MSP_HEADER_START:
            if (byte == MSP_V2_HEADER_X) {
                parser->state = MSP_HEADER_X;
            } else {
                parser->state = MSP_IDLE;
            }
            break;

        case MSP_HEADER_X:
            if (byte == MSP_V2_FLAG_REQUEST ||
                byte == MSP_V2_FLAG_RESPONSE ||
                byte == MSP_V2_FLAG_ERROR) {
                parser->flags = byte;  // Store direction ('<', '>', '!')
                // Note: Direction byte is NOT included in CRC
                parser->state = MSP_HEADER_V2_FLAGS;
            } else {
                parser->state = MSP_IDLE;
            }
            break;

        case MSP_HEADER_V2_FLAGS:
            // This reads the 'flags' field from mspHeaderV2_t struct (not the direction)
            // Just add to checksum and skip it
            parser->checksum = crc8_dvb_s2(parser->checksum, byte);
            parser->state = MSP_HEADER_V2_FUNC_L;
            break;

        case MSP_HEADER_V2_FUNC_L:
            parser->function = byte;  // Set low byte
            parser->checksum = crc8_dvb_s2(parser->checksum, byte);
            parser->state = MSP_HEADER_V2_FUNC_H;
            break;

        case MSP_HEADER_V2_FUNC_H:
            parser->function |= (uint16_t)byte << 8;  // OR in high byte
            parser->checksum = crc8_dvb_s2(parser->checksum, byte);
            parser->state = MSP_HEADER_V2_SIZE_L;
            break;

        case MSP_HEADER_V2_SIZE_L:
            parser->payload_size = byte;  // Set low byte
            parser->checksum = crc8_dvb_s2(parser->checksum, byte);
            parser->state = MSP_HEADER_V2_SIZE_H;
            break;

        case MSP_HEADER_V2_SIZE_H:
            parser->payload_size |= (uint16_t)byte << 8;  // OR in high byte
            parser->checksum = crc8_dvb_s2(parser->checksum, byte);
            parser->payload_offset = 0;

            if (parser->payload_size > MSP_MAX_PAYLOAD_SIZE) {
                ESP_LOGW(TAG, "Payload too large: %d", parser->payload_size);
                parser->state = MSP_IDLE;
            } else if (parser->payload_size == 0) {
                parser->state = MSP_CHECKSUM_V2;
            } else {
                parser->state = MSP_PAYLOAD_V2;
            }
            break;

        case MSP_PAYLOAD_V2:
            parser->payload[parser->payload_offset++] = byte;
            parser->checksum = crc8_dvb_s2(parser->checksum, byte);

            if (parser->payload_offset >= parser->payload_size) {
                parser->state = MSP_CHECKSUM_V2;
            }
            break;

        case MSP_CHECKSUM_V2:
            if (byte == parser->checksum) {
                // Valid packet received
                parser->state = MSP_IDLE;
                return true;
            } else {
                ESP_LOGW(TAG, "CRC mismatch: expected=0x%02X, got=0x%02X",
                        parser->checksum, byte);
                parser->state = MSP_IDLE;
            }
            break;

        default:
            parser->state = MSP_IDLE;
            break;
    }

    return false;
}

/**
 * @brief Process MSP packet
 */
static void process_msp_packet(msp_parser_t *parser) {
    ESP_LOGI(TAG, "MSP packet: func=0x%04X, size=%d",
             parser->function, parser->payload_size);

    switch (parser->function) {
        case MSP_SET_VTX_CONFIG:
            handle_msp_set_vtx_config(parser->payload, parser->payload_size);
            break;

        case MSP_ELRS_BIND:
            ESP_LOGI(TAG, "Received MSP_ELRS_BIND packet");
            if (binding_state == ELRS_STATE_BINDING && parser->payload_size >= 6) {
                // Extract UID from payload
                memcpy(pending_uid, parser->payload, 6);

                // Validate UID (first byte must be even for unicast)
                pending_uid[0] &= ~0x01;

                ESP_LOGI(TAG, "Binding to UID: %02X:%02X:%02X:%02X:%02X:%02X",
                         pending_uid[0], pending_uid[1], pending_uid[2],
                         pending_uid[3], pending_uid[4], pending_uid[5]);

                // Complete binding
                complete_binding();
            } else if (binding_state != ELRS_STATE_BINDING) {
                ESP_LOGW(TAG, "Received bind packet but not in binding mode");
            } else {
                ESP_LOGW(TAG, "Bind packet too short: %d bytes", parser->payload_size);
            }
            break;

        default:
            ESP_LOGD(TAG, "Unhandled MSP function: 0x%04X", parser->function);
            break;
    }
}

/**
 * @brief ESP-NOW receive callback (ISR context)
 */
static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (len == 0 || len > 250 || espnow_queue == NULL) {
        return;
    }

    espnow_msg_t msg;
    memcpy(msg.mac_addr, recv_info->src_addr, 6);
    memcpy(msg.data, data, len);
    msg.data_len = len;

    // Send to queue (non-blocking from ISR)
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(espnow_queue, &msg, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

/**
 * @brief ELRS backpack main task
 */
static void elrs_backpack_task(void *param) {
    espnow_msg_t msg;

    ESP_LOGI(TAG, "ELRS backpack task started");

    while (1) {
        // Wait for ESP-NOW messages
        if (xQueueReceive(espnow_queue, &msg, portMAX_DELAY) == pdTRUE) {
            ESP_LOGD(TAG, "ESP-NOW packet from %02X:%02X:%02X:%02X:%02X:%02X, len=%d",
                     msg.mac_addr[0], msg.mac_addr[1], msg.mac_addr[2],
                     msg.mac_addr[3], msg.mac_addr[4], msg.mac_addr[5],
                     msg.data_len);

            // Feed data to MSP parser
            for (int i = 0; i < msg.data_len; i++) {
                if (msp_parser_feed_byte(&msp_parser, msg.data[i])) {
                    // Complete packet received
                    process_msp_packet(&msp_parser);
                    msp_parser_init(&msp_parser);
                }
            }
        }
    }
}

/**
 * @brief Initialize ELRS VRX Backpack
 */
bool ELRS_Backpack_Init(void) {
    esp_err_t ret;

    // Initialize MSP parser
    msp_parser_init(&msp_parser);

    // Initialize NVS (required for WiFi)
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(ret));
        return false;
    }

    // Try to load UID from NVS
    if (elrs_config_load_uid(elrs_uid)) {
        binding_state = ELRS_STATE_BOUND;
        ESP_LOGI(TAG, "Loaded UID from NVS: %02X:%02X:%02X:%02X:%02X:%02X",
                 elrs_uid[0], elrs_uid[1], elrs_uid[2],
                 elrs_uid[3], elrs_uid[4], elrs_uid[5]);
    } else {
        binding_state = ELRS_STATE_UNBOUND;
        ESP_LOGW(TAG, "No UID found in NVS - use UI to bind");
        // Set to all zeros
        memset(elrs_uid, 0, 6);
    }

    // Initialize WiFi in STA mode
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    // Set our MAC address to the UID (BEFORE starting WiFi)
    // If unbound, use zeros (will be changed during binding)
    ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, elrs_uid));
    ESP_LOGI(TAG, "MAC set to: %02X:%02X:%02X:%02X:%02X:%02X",
             elrs_uid[0], elrs_uid[1], elrs_uid[2],
             elrs_uid[3], elrs_uid[4], elrs_uid[5]);

    ESP_ERROR_CHECK(esp_wifi_start());

    // Set WiFi channel to 1
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

    ESP_LOGI(TAG, "WiFi initialized (STA mode, channel 1)");

    // Initialize ESP-NOW
    ret = esp_now_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW init failed: %s", esp_err_to_name(ret));
        return false;
    }

    // Register ESP-NOW receive callback
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    // Add UID as peer (same as our own MAC - creates broadcast domain)
    // Both TX and VRX use the same UID, allowing all devices with
    // matching binding phrase to communicate
    esp_now_peer_info_t peer_info = {0};
    memcpy(peer_info.peer_addr, elrs_uid, 6);
    peer_info.channel = 1;
    peer_info.ifidx = WIFI_IF_STA;
    peer_info.encrypt = false;

    ret = esp_now_add_peer(&peer_info);
    if (ret != ESP_OK && ret != ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGE(TAG, "Failed to add peer: %s", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "ESP-NOW initialized with UID: %02X:%02X:%02X:%02X:%02X:%02X",
             elrs_uid[0], elrs_uid[1], elrs_uid[2],
             elrs_uid[3], elrs_uid[4], elrs_uid[5]);

    // Create message queue
    espnow_queue = xQueueCreate(20, sizeof(espnow_msg_t));
    if (espnow_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create queue");
        return false;
    }

    // Create ELRS backpack task
    BaseType_t task_ret = xTaskCreatePinnedToCore(
        elrs_backpack_task,
        "elrs_backpack",
        4096,  // Increased from 2048 to prevent stack overflow
        NULL,
        3,
        &backpack_task_handle,
        0
    );

    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create task");
        return false;
    }

    ESP_LOGI(TAG, "ELRS backpack initialized successfully");
    return true;
}

elrs_bind_state_t ELRS_Backpack_Get_State(void) {
    return binding_state;
}

bool ELRS_Backpack_Start_Binding(uint32_t timeout_ms) {
    if (binding_state == ELRS_STATE_BINDING) {
        ESP_LOGW(TAG, "Already in binding mode");
        return false;
    }

    ESP_LOGI(TAG, "Starting binding mode (timeout: %lu ms)", timeout_ms);

    // Save previous state
    previous_state = binding_state;
    binding_state = ELRS_STATE_BINDING;
    binding_timeout_ms = timeout_ms;
    binding_start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // Add broadcast MAC as peer to receive broadcast bind packets
    // We keep our own MAC unchanged
    esp_now_peer_info_t peer_info = {0};
    memcpy(peer_info.peer_addr, broadcast_mac, 6);
    peer_info.channel = 1;
    peer_info.ifidx = WIFI_IF_STA;
    peer_info.encrypt = false;

    esp_err_t ret = esp_now_add_peer(&peer_info);
    if (ret != ESP_OK && ret != ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGE(TAG, "Failed to add broadcast peer: %s", esp_err_to_name(ret));
        binding_state = previous_state;
        return false;
    }

    ESP_LOGI(TAG, "Added broadcast peer for binding");

    // Create binding timeout timer
    binding_timer = xTimerCreate(
        "binding_timer",
        pdMS_TO_TICKS(timeout_ms),
        pdFALSE,  // One-shot timer
        NULL,
        binding_timeout_callback
    );

    if (binding_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create binding timer");
        // Remove broadcast peer
        esp_now_del_peer(broadcast_mac);
        binding_state = previous_state;
        return false;
    }

    // Start timer
    xTimerStart(binding_timer, 0);

    ESP_LOGI(TAG, "Binding mode active - waiting for TX...");
    return true;
}

void ELRS_Backpack_Cancel_Binding(void) {
    if (binding_state != ELRS_STATE_BINDING) {
        return;
    }

    ESP_LOGI(TAG, "Canceling binding mode");

    // Stop and delete timer
    if (binding_timer != NULL) {
        xTimerStop(binding_timer, 0);
        xTimerDelete(binding_timer, 0);
        binding_timer = NULL;
    }

    // Remove broadcast peer
    esp_now_del_peer(broadcast_mac);

    // Restore previous state
    binding_state = previous_state;

    // If we were bound before, restore the UID peer
    if (binding_state == ELRS_STATE_BOUND) {
        uint8_t stored_uid[6];
        if (elrs_config_load_uid(stored_uid)) {
            // Add the bound UID as peer again (if not already present)
            esp_now_peer_info_t peer_info = {0};
            memcpy(peer_info.peer_addr, stored_uid, 6);
            peer_info.channel = 1;
            peer_info.ifidx = WIFI_IF_STA;
            peer_info.encrypt = false;
            esp_now_add_peer(&peer_info);
        }
    }

    ESP_LOGI(TAG, "Binding canceled");
}

bool ELRS_Backpack_Is_Bound(void) {
    return (binding_state == ELRS_STATE_BOUND || binding_state == ELRS_STATE_BIND_SUCCESS);
}

void ELRS_Backpack_Unbind(void) {
    ESP_LOGI(TAG, "Unbinding from TX");

    // Clear UID from NVS
    elrs_config_clear_uid();

    // Cancel binding if active
    if (binding_state == ELRS_STATE_BINDING) {
        ELRS_Backpack_Cancel_Binding();
    }

    // Set state to unbound
    binding_state = ELRS_STATE_UNBOUND;

    // Clear UID
    memset(elrs_uid, 0, 6);

    // Reinitialize with zeros (will need rebinding)
    reinit_espnow_with_uid(elrs_uid);

    ESP_LOGI(TAG, "Unbound successfully");
}

void ELRS_Backpack_Get_UID(uint8_t uid[6]) {
    if (uid != NULL) {
        memcpy(uid, elrs_uid, 6);
    }
}

uint32_t ELRS_Backpack_Get_Binding_Timeout_Remaining(void) {
    if (binding_state != ELRS_STATE_BINDING) {
        return 0;
    }

    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t elapsed = current_time - binding_start_time;

    if (elapsed >= binding_timeout_ms) {
        return 0;
    }

    return (binding_timeout_ms - elapsed) / 1000;  // Return seconds
}
