#ifndef __ELRS_BACKPACK_H
#define __ELRS_BACKPACK_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Binding state machine states
 */
typedef enum {
    ELRS_STATE_UNBOUND,      // No UID stored, not operational
    ELRS_STATE_BOUND,        // UID stored, normal operation
    ELRS_STATE_BINDING,      // Actively waiting for bind packet
    ELRS_STATE_BIND_SUCCESS, // Just received bind packet
    ELRS_STATE_BIND_TIMEOUT  // Binding timed out
} elrs_bind_state_t;

/**
 * @brief Initialize ELRS VRX Backpack
 *
 * Initializes WiFi, ESP-NOW, and creates the backpack task.
 * This function should be called after RX5808_Init().
 *
 * @return true if initialization successful, false otherwise
 */
bool ELRS_Backpack_Init(void);

/**
 * @brief Get current binding state
 *
 * @return Current state of the binding state machine
 */
elrs_bind_state_t ELRS_Backpack_Get_State(void);

/**
 * @brief Start binding mode
 *
 * Sets MAC to broadcast, starts timeout timer, waits for bind packet.
 *
 * @param timeout_ms Timeout in milliseconds (e.g., 30000 for 30 seconds)
 * @return true if binding mode started, false if already binding or error
 */
bool ELRS_Backpack_Start_Binding(uint32_t timeout_ms);

/**
 * @brief Cancel binding mode
 *
 * Stops binding and returns to previous state.
 */
void ELRS_Backpack_Cancel_Binding(void);

/**
 * @brief Check if VRX is bound to a TX
 *
 * @return true if a valid UID is stored, false otherwise
 */
bool ELRS_Backpack_Is_Bound(void);

/**
 * @brief Unbind from current TX
 *
 * Clears the stored UID and sets state to UNBOUND.
 */
void ELRS_Backpack_Unbind(void);

/**
 * @brief Get current UID
 *
 * @param uid Buffer to store 6-byte UID
 */
void ELRS_Backpack_Get_UID(uint8_t uid[6]);

/**
 * @brief Get remaining binding timeout in seconds
 *
 * @return Seconds remaining, or 0 if not binding
 */
uint32_t ELRS_Backpack_Get_Binding_Timeout_Remaining(void);

#endif // __ELRS_BACKPACK_H
