/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the license.txt file.
 */

#include "ble_dfus.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_dfus      DFU Trigger Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_dfus_t * p_dfus, ble_evt_t * p_ble_evt)
{
    p_dfus->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_dfus      DFU Trigger Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_dfus_t * p_dfus, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_dfus->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_dfus      DFU Trigger Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_dfus_t * p_dfus, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if ((p_evt_write->handle == p_dfus->trigger_char_handles.value_handle) &&
        (p_evt_write->len == 1) &&
        (p_dfus->dfu_trigger_handler != NULL))
    {
        p_dfus->dfu_trigger_handler(p_dfus, p_evt_write->data[0]);
    }
}


void ble_dfus_on_ble_evt(ble_dfus_t * p_dfus, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_dfus, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_dfus, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_dfus, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for adding the DFU trigger characteristic.
 *
 */
static uint32_t trigger_char_add(ble_dfus_t * p_dfus, const ble_dfus_init_t * p_dfus_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = "DFU Trigger";
    char_md.char_user_desc_max_size = 16;
    char_md.char_user_desc_size = 11;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_dfus->uuid_type;
    ble_uuid.uuid = DFUS_UUID_TRIGGER_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint8_t);
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_dfus->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_dfus->trigger_char_handles);
}

uint32_t ble_dfus_init(ble_dfus_t * p_dfus, const ble_dfus_init_t * p_dfus_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_dfus->conn_handle       = BLE_CONN_HANDLE_INVALID;
    p_dfus->dfu_trigger_handler = p_dfus_init->dfu_trigger_handler;

    // Add service
    ble_uuid128_t base_uuid = DFUS_UUID_BASE;
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_dfus->uuid_type);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    ble_uuid.type = p_dfus->uuid_type;
    ble_uuid.uuid = DFUS_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_dfus->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = trigger_char_add(p_dfus, p_dfus_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}
