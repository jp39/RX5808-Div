#include "elrs_config.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "ELRS_CFG";

// NVS namespace for ELRS configuration
#define ELRS_NVS_NAMESPACE "elrs"

// NVS key for UID
#define ELRS_NVS_KEY_UID "uid"

// UID length in bytes
#define ELRS_UID_LEN 6

bool elrs_config_load_uid(uint8_t uid[6]) {
    if (uid == NULL) {
        ESP_LOGE(TAG, "UID buffer is NULL");
        return false;
    }

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(ELRS_NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(err));
        return false;
    }

    size_t len = ELRS_UID_LEN;
    err = nvs_get_blob(nvs_handle, ELRS_NVS_KEY_UID, uid, &len);
    nvs_close(nvs_handle);

    if (err != ESP_OK || len != ELRS_UID_LEN) {
        ESP_LOGD(TAG, "Failed to load UID: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "UID loaded: %02X:%02X:%02X:%02X:%02X:%02X",
             uid[0], uid[1], uid[2], uid[3], uid[4], uid[5]);
    return true;
}

bool elrs_config_save_uid(const uint8_t uid[6]) {
    if (uid == NULL) {
        ESP_LOGE(TAG, "UID is NULL");
        return false;
    }

    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(ELRS_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(err));
        return false;
    }

    err = nvs_set_blob(nvs_handle, ELRS_NVS_KEY_UID, uid, ELRS_UID_LEN);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save UID: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return false;
    }

    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit UID: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "UID saved: %02X:%02X:%02X:%02X:%02X:%02X",
             uid[0], uid[1], uid[2], uid[3], uid[4], uid[5]);
    return true;
}

bool elrs_config_clear_uid(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(ELRS_NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(err));
        return false;
    }

    err = nvs_erase_key(nvs_handle, ELRS_NVS_KEY_UID);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "Failed to erase UID: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return false;
    }

    err = nvs_commit(nvs_handle);
    nvs_close(nvs_handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit erase: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "UID cleared (unbound)");
    return true;
}

bool elrs_config_has_uid(void) {
    uint8_t uid[ELRS_UID_LEN];

    // Try to load UID
    if (!elrs_config_load_uid(uid)) {
        return false;
    }

    // Check if UID is not all zeros
    for (int i = 0; i < ELRS_UID_LEN; i++) {
        if (uid[i] != 0) {
            return true;
        }
    }

    ESP_LOGD(TAG, "UID is all zeros (unbound)");
    return false;
}
