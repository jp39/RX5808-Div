#ifndef __ELRS_BACKPACK_H
#define __ELRS_BACKPACK_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize ELRS VRX Backpack
 *
 * Initializes WiFi, ESP-NOW, and creates the backpack task.
 * This function should be called after RX5808_Init().
 *
 * @return true if initialization successful, false otherwise
 */
bool ELRS_Backpack_Init(void);

#endif // __ELRS_BACKPACK_H
