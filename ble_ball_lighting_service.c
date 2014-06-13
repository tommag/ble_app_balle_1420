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

#include "ble_ball_lighting_service.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"

/**@brief Function for handling the Write event and dispatching to the correct handler (White LED or IR LED).
 *
 * @param[in]   p_bls       Ball Lighting Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_bls_t * p_bls, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    
	//test if the write is directed to the white LED characteristic
    if ((p_evt_write->handle == p_bls->wled_char_handles.value_handle) &&
        (p_evt_write->len == 1) &&
        (p_bls->wled_handler != NULL))
    {
        p_bls->wled_handler(p_bls, p_evt_write->data[0]);
    }
	//else test if it goes to the IR LED characteristic
    else if ((p_evt_write->handle == p_bls->irled_char_handles.value_handle) &&
        (p_evt_write->len == 1) &&
        (p_bls->irled_handler != NULL))
    {
        p_bls->irled_handler(p_bls, p_evt_write->data[0]);
    }
}


void ble_bls_on_ble_evt(ble_bls_t * p_bls, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTS_EVT_WRITE:
            on_write(p_bls, p_ble_evt);
            break;
            
        default:
            // No implementation needed.
            break;
    }
}



/**@brief Function for adding the White LED intensity characteristic.
 *
 * @param[in]   p_bls        Ball Lighting Service structure.
 * @param[in]   p_bls_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t wled_char_add(ble_bls_t * p_bls, const ble_bls_init_t * p_bls_init)
{
    ble_uuid_t          ble_uuid;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_gatts_attr_md_t attr_md;
    
    memset(&char_md, 0, sizeof(char_md));
    //readable and writable
    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = "WLED intensity";
    char_md.char_user_desc_max_size = 16;
    char_md.char_user_desc_size = 14;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
	char_md.p_cccd_md		  = NULL;
    char_md.p_sccd_md         = NULL;
    
	ble_uuid.type = p_bls->uuid_type;
	ble_uuid.uuid = BLS_UUID_WLED_CHAR;
	
    memset(&attr_md, 0, sizeof(attr_md));
	//no security needed (open link)
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
    attr_char_value.p_value      = 0; //Default value : OFF
    
    return sd_ble_gatts_characteristic_add(p_bls->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_bls->wled_char_handles);
}


/**@brief Function for adding the IR LED state characteristic.
 *
 * @param[in]   p_bls        Ball Lighting Service structure.
 * @param[in]   p_bls_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t irled_char_add(ble_bls_t * p_bls, const ble_bls_init_t * p_bls_init)
{
    ble_uuid_t          ble_uuid;
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_gatts_attr_md_t attr_md;
    
    memset(&char_md, 0, sizeof(char_md));
    //readable and writable
    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = "IRLED intensity";
    char_md.char_user_desc_max_size = 16;
    char_md.char_user_desc_size = 15;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
	char_md.p_cccd_md		  = NULL;
    char_md.p_sccd_md         = NULL;
    
	ble_uuid.type = p_bls->uuid_type;
	ble_uuid.uuid = BLS_UUID_IRLED_CHAR;
	
    memset(&attr_md, 0, sizeof(attr_md));
	//no security needed (open link)
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
    attr_char_value.p_value      = 0; //Default value : OFF
    
    return sd_ble_gatts_characteristic_add(p_bls->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_bls->irled_char_handles);
}


uint32_t ble_bls_init(ble_bls_t * p_bls, const ble_bls_init_t * p_bls_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_bls->wled_handler              = p_bls_init->wled_handler;
    p_bls->irled_handler             = p_bls_init->irled_handler;
    
    //handle vendor-specific UUID
    ble_uuid128_t base_uuid = BLS_UUID_BASE;
	err_code = sd_ble_uuid_vs_add(&base_uuid, &p_bls->uuid_type);
	if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		
	//add service
	ble_uuid.type = p_bls->uuid_type;
	ble_uuid.uuid = BLS_UUID_SERVICE;
	  
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_bls->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
		
	//Add characteristics
	err_code = wled_char_add(p_bls, p_bls_init);
	if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
	err_code = irled_char_add(p_bls, p_bls_init);
	if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    
    return NRF_SUCCESS;
}

