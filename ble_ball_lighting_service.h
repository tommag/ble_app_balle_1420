/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @brief Ball Lighting Service module.
 *
 * @details This module implements the Ball Lighting custom Service. 
 * It contains two characteristics : 
 * *White LED intensity (between 0 and 255), written by the GATT client (master device)
 * *IR LED state (on or off), written by the GATT client
 *
 * @note The application must propagate BLE stack events to the Ball Lighting Service module by calling
 *       ble_bls_on_ble_evt() from the from the @ref ble_stack_handler callback.
 */

#ifndef BLE_BLS_H__
#define BLE_BLS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

//UUID management for custom service
#define BLS_UUID_BASE {0xA2, 0xCD, 0x1E, 0x3E, 0x05, 0x95, 0x81, 0xD1, 0xED, 0x71, 0xCA, 0x6F, 0x00, 0x00, 0xA9, 0x39}
#define BLS_UUID_SERVICE 0x1523
#define BLS_UUID_WLED_CHAR 0x1525
#define BLS_UUID_IRLED_CHAR 0x1524

// Forward declaration of the ble_bls_t type. 
typedef struct ble_bls_s ble_bls_t;

/**@brief Ball Lighting Service event handler type. */
typedef void (*ble_bls_evt_handler_t) (ble_bls_t * p_bls, uint8_t new_value);

/**@brief Ball Lighting Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_bls_evt_handler_t         wled_handler;                   /**< Event handler to be called for handling events in the White LED characteristic. */
    ble_bls_evt_handler_t         irled_handler;                  /**< Event handler to be called for handling events in the IR LED characteristic. */
} ble_bls_init_t;

/**@brief Ball Lighting Service structure. This contains various status information for the service. */
typedef struct ble_bls_s
{
    uint16_t                      service_handle;                 /**< Handle of Battery Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      wled_char_handles;              /**< Handles related to the White LED characteristic. */
    ble_gatts_char_handles_t      irled_char_handles;             /**< Handles related to the IR LED characteristic. */
		uint8_t 											uuid_type;
    ble_bls_evt_handler_t         wled_handler;                   /**< Event handler to be called for handling events in the White LED characteristic. */
    ble_bls_evt_handler_t         irled_handler;                  /**< Event handler to be called for handling events in the IR LED characteristic. */

} ble_bls_t;

/**@brief Function for initializing the Ball Lighting Service.
 *
 * @param[out]  p_bls       Ball Lighting Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_bls_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_bls_init(ble_bls_t * p_bls, const ble_bls_init_t * p_bls_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Ball Lighting Service.
 *
 * @param[in]   p_bls      Ball Lighting Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_bls_on_ble_evt(ble_bls_t * p_bls, ble_evt_t * p_ble_evt);

#endif // BLE_BLS_H__

/** @} */
