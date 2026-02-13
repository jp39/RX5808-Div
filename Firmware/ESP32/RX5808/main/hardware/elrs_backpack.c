#include "elrs_backpack.h"
#include "elrs_msp.h"
#include "rx5808.h"
#include <string.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "ELRS_BP";

// ELRS UID derived from binding phrase
// Both TX and VRX set their MAC to this UID for ESP-NOW communication
// Generate with: python3 generate_uid.py "your_binding_phrase"
// IMPORTANT: This must match your ELRS binding phrase UID
// The first byte MUST be even (unicast requirement)
static const uint8_t elrs_uid[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

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

    // Initialize WiFi in STA mode
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    // Set our MAC address to the UID (BEFORE starting WiFi)
    // This matches the TX backpack behavior - both sides use the same UID as their MAC
    ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, elrs_uid));
    ESP_LOGI(TAG, "MAC set to UID: %02X:%02X:%02X:%02X:%02X:%02X",
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
