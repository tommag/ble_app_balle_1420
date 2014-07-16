/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the license.txt file.
 */

#ifndef BLE_DFUS_H__
#define BLE_DFUS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#define DFUS_UUID_BASE {0x4E, 0xDA, 0x0C, 0x43, 0xB3, 0xB4, 0xFB, 0x64, 0xE0, 0x36, 0x35, 0x8B, 0x00, 0x00, 0xA1, 0x25} 
#define DFUS_UUID_SERVICE 0x1420
#define DFUS_UUID_TRIGGER_CHAR 0x1440

// Forward declaration of the ble_dfus_t type.
typedef struct ble_dfus_s ble_dfus_t;

typedef void (*ble_dfus_trigger_handler_t) (ble_dfus_t * p_dfus, uint8_t new_state);

typedef struct
{
    ble_dfus_trigger_handler_t  dfu_trigger_handler;                    /**< Event handler to be called when Trigger characteristic is written. */
} ble_dfus_init_t;

/**@brief DFU Trigger Service structure. This contains various status information for the service. */
typedef struct ble_dfus_s
{
    uint16_t                    service_handle;
    ble_gatts_char_handles_t    trigger_char_handles;
    uint8_t                     uuid_type;
    uint16_t                    conn_handle;
    ble_dfus_trigger_handler_t  dfu_trigger_handler;
} ble_dfus_t;

/**@brief Function for initializing the DFU Trigger Service.
 *
 * @param[out]  p_dfus      DFU Trigger Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_dfus_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_dfus_init(ble_dfus_t * p_dfus, const ble_dfus_init_t * p_dfus_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the DFU Trigger Service.
 *
 *
 * @param[in]   p_dfus     DFU Trigger Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_dfus_on_ble_evt(ble_dfus_t * p_dfus, ble_evt_t * p_ble_evt);

/**@brief Function for sending a button state notification.
 */

#endif // BLE_DFUS_H__

/** @} */
