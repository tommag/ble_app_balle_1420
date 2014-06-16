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
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "board_balle_1420_v1.h"
#include "app_scheduler.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "ble_error_log.h"
#include "app_gpiote.h"
#include "app_button.h"
#include "ble_debug_assert_handler.h"
#include "pstorage.h"
#include "ble_ball_lighting_service.h"
#include "ble_bas.h" //Battery service
#include "nrf_pwm.h"
#include "debug.h"
#include "nrf_delay.h"

#define DEVICE_NAME                     "Balle 14:20 v1"                           /**< Name of device. Will be included in the advertising data. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

// YOUR_JOB: Modify these according to requirements.
#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            2                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(500, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(1000, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_GPIOTE_MAX_USERS            1                                           /**< Maximum number of users of the GPIOTE handler. */
static app_gpiote_user_id_t 			m_gpiote_user_id;							//GPIOTE library user identifier

#define SEC_PARAM_TIMEOUT               30                                          /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                  0                                           /**< Don't perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

static ble_gap_sec_params_t             m_sec_params;                               /**< Security requirements for this application. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static ble_bls_t												m_bls;																			//Ball lighting service data structure

// YOUR_JOB: Modify these according to requirements (e.g. if other event types are to pass through
//           the scheduler).
#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)                   /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#define SCHED_QUEUE_SIZE                10                                          /**< Maximum number of events in the scheduler queue. */

// Persistent storage system event handler
void pstorage_sys_event_handler (uint32_t p_evt);


//States for the state machine
typedef enum
{
	STATE_INIT,				//init state
	STATE_STANDBY,			//stand-by state : low power mode, radio turned off ; waiting for an accelerometer event
	STATE_FIRST_CONN_ATT,	//First connection attempt : advertising with user feedback (Led)
	STATE_NEW_CONN_ATT,		//New connection attempt : trying to establish a new connection after a connection timeout ; advertising without user feedback
	STATE_ACTIVE,			//Active state : connection established, waiting for commands via BLE
} app_states_t;

//State transition events for the state machine
typedef enum
{
	ON_ACCELEROMETER_EVENT,	//Event: accelerometer reporting a motion event
	ON_ADVERTISING_TIMEOUT,	//Event: advertisting timed out
	ON_CONNECTION_TIMEOUT,	//Event: connection with the Master device timed out
	ON_ACTIVITY_TIMEOUT,	//Event: device in stand-by state for too long
	ON_DISCONNECT_RQ,		//Event: disconnect request sent by the Master device
	ON_CONNECTION_SUCCESS,	//Event: connection success
	ON_LOW_BATT,			//Event: low battery
} app_state_events_t;

static volatile app_states_t m_current_state = STATE_INIT;

/* Actions to do when entering the STANDBY state :
 * Start a timer to detect the activity timeout
 * Perform energy saving optimizations
 * Setup accelerometer to detect motion events
 */
static void action_enter_standby(void)
{
	m_current_state = STATE_STANDBY;
	//TODO
}

/* Actions to do when entering the power down state (after a low battery event or activity timeout)
 * Turn down all peripherals
 * Setup wakeup source
 * Enter power-down
 * 
 * When woken up from power down, the CPU will reset
 */
static void action_enter_power_down(void)
{
	//TODO
}

/* Actions to do when entering the ACTIVE state 
 */
static void action_enter_active(void)
{
	m_current_state = STATE_ACTIVE;
}

/* Handling of the ON_ACCELEROMETER_EVENT event */
static void on_accelerometer_event(void)
{
	switch(m_current_state)
	{
		case STATE_STANDBY:
			m_current_state = STATE_FIRST_CONN_ATT;
			//TODO start user feedback
			//TODO start advertising
			break;
		default:
			break;
	}
}

/* Handling of the ON_ADVERTISING_TIMEOUT event */
static void on_advertising_timeout(void)
{
	switch(m_current_state)
	{
		case STATE_FIRST_CONN_ATT:
			//TODO user feedback
			action_enter_standby();
			break;
		case STATE_NEW_CONN_ATT:
			action_enter_standby();
			break;
		default:
			break;
	}
}

/* Handling of the ON_CONNECTION_TIMEOUT event */
static void on_connection_timeout(void)
{
	switch(m_current_state)
	{
		case STATE_ACTIVE:
			m_current_state = STATE_NEW_CONN_ATT; //Try to establish a new connection immediately
			//TODO start advertising
			break;
		default:
			break;
	}
}

/* Handling of the ON_ACTIVITY_TIMEOUT event */
static void on_activity_timeout(void)
{
	switch(m_current_state)
	{
		case STATE_STANDBY:
			action_enter_power_down();
			break;
		default:
			break;
	}	
}

/* Handling of the ON_DISCONNECT_RQ event */
static void on_disconnect_rq(void)
{
	switch(m_current_state)
	{
		case STATE_ACTIVE:
			//TODO user feedback
			action_enter_standby();
			break;
		default:
			break;
	}
}

/* Handling of the ON_CONNECTION_SUCCESS event */
static void on_connection_success(void)
{
	switch(m_current_state)
	{
		case STATE_FIRST_CONN_ATT:
			//TODO user feedback
			action_enter_active();
			break;
		case STATE_NEW_CONN_ATT:
			action_enter_active();
			break;
		default:
			break;
	}
}

/* Handling of the ON_LOW_BATT event */
static void on_low_batt(void)
{
	switch(m_current_state)
	{
		default:
			action_enter_power_down();
			break;
	}
}

/* State machine main event handler */
static void on_app_event(app_state_events_t event)
{
	switch(event)
	{
		case ON_ACCELEROMETER_EVENT: on_accelerometer_event(); break;
		case ON_ADVERTISING_TIMEOUT: on_advertising_timeout(); break;
		case ON_CONNECTION_TIMEOUT: on_connection_timeout(); break;
		case ON_ACTIVITY_TIMEOUT: on_activity_timeout(); break;
		case ON_DISCONNECT_RQ: on_disconnect_rq(); break;
		case ON_CONNECTION_SUCCESS: on_connection_success(); break;
		case ON_LOW_BATT: on_low_batt(); break;
		default: break;
	}
	
	debug_log("[APPL]: Event 0x%02x occurred, new state : 0x%02x", event, m_current_state);
}

/**@brief Function called when an interrupt is sent by the accelerometer via the Scheduler
 * 
 * @details Assert the interrupt then call the related application event
 * 
 * @param[in] int_num  Interrupt line asserted (1 or 2)
 */
static void on_accelerometer_interrupt(void * p_event_data, uint16_t event_size)
{
	//TODO assert interrupt
	on_app_event(ON_ACCELEROMETER_EVENT);
}

/**@brief Function called when the battery charge stops, either because it is complete or the charger is disconnected
 */
static void on_battery_charge_stop(void * p_event_data, uint16_t event_size)
{
	//Do nothing
}

/**@brief Function called when the battery charge starts
 */
static void on_battery_charge_start(void * p_event_data, uint16_t event_size)
{
	//Do nothing
}

/**@brief Function for error handling, which is called when an error has occurred.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name.
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    // This call can be used for debug purposes during application development.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    //ble_debug_assert_handler(error_code, line_num, p_file_name);
	
	debug_log("[APPL]: ASSERT: %s, %d, error 0x%08x\r\n", p_file_name, line_num, error_code);
	
	nrf_delay_ms(2000);

    // On assert, the system can only recover with a reset.
    NVIC_SystemReset();
}


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for GPIOTE events handling, called when a GPIO interrupt occurs
 * GPIOTE pins and transitions from which events occur are defined in gpiote_init() function
 */
void gpiote_event_handler(uint32_t event_pins_lo_to_hi, uint32_t event_pins_hi_to_lo)
{
	//if ACC_INT1 or ACC_INT2 transitioned from low to high, call the accelerometer handler
	uint32_t int_num = 0;
	if(event_pins_lo_to_hi & (1UL << ACC_INT1_PIN))
	{
		int_num = 1; 
		app_sched_event_put(&int_num, sizeof(int_num), on_accelerometer_interrupt);
	}
	
	if(event_pins_lo_to_hi & (1UL << ACC_INT2_PIN))
	{
		int_num = 2; 
		app_sched_event_put(&int_num, sizeof(int_num), on_accelerometer_interrupt);
	}
	
	//if charger status line transitioned from low to high call the "charger disconnected" handler
	if(event_pins_lo_to_hi & (1UL << CHRG_STAT_PIN))
		app_sched_event_put(NULL, 0, on_battery_charge_stop);
	
	//else call the "charger connected" handler
	if(event_pins_hi_to_lo & (1UL << CHRG_STAT_PIN))
		app_sched_event_put(NULL, 0, on_battery_charge_start);
}

/**@brief Function for the pins initialization.
 *
 * @details Initializes all I/O pins used by the application.
 */
static void pins_init(void)
{
	//Init LED output
	nrf_gpio_cfg_output(IRLED_PIN); //simple output
	
	//Init accelerometer interrupt inputs with pull down resistor
	//The GPIOTE handling library takes care of defining the SENSE fields
	nrf_gpio_cfg_sense_input(ACC_INT1_PIN, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_NOSENSE);
	nrf_gpio_cfg_sense_input(ACC_INT2_PIN, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_NOSENSE);
	
	//Init battery charger status input with pull up resistor
	nrf_gpio_cfg_sense_input(CHRG_STAT_PIN, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_NOSENSE);
}

/**@brief Function for the PWM initialization.
 *
 * @details Initializes the PWM module needed to set the white LEDs intensity
 */
static void pwm_init(void)
{
	nrf_pwm_config_t pwm_config = PWM_DEFAULT_CONFIG;
	
	pwm_config.mode = PWM_MODE_LED_255;
	pwm_config.num_channels = 1;
	pwm_config.gpio_num[0] = WLED_PIN;
	
	nrf_pwm_init(&pwm_config);
	
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, true);

    /* YOUR_JOB: Create any timers to be used by the application.
                 Below is an example of how to create a timer.
                 For every new timer needed, increase the value of the macro APP_TIMER_MAX_TIMERS by
                 one.
    err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
    APP_ERROR_CHECK(err_code); */
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    /* YOUR_JOB: Use an appearance value matching the application's use case.
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
    APP_ERROR_CHECK(err_code); */

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    // YOUR_JOB: Use UUIDs for service(s) used in your application.
    ble_uuid_t adv_uuids[] = {{BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE}};

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;
    advdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling new values received by BLE for the white LEDs intensity
 */
static void wled_handler(ble_bls_t *p_bls, uint8_t new_value)
{
	nrf_pwm_set_value(0, new_value); // channel 0 for the PWM module
}

/**@brief Function for handling new values received by BLE for the IR LEDs state
 */
static void irled_handler(ble_bls_t *p_bls, uint8_t new_state)
{
	nrf_gpio_pin_write(IRLED_PIN, new_state); 
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t err_code;
	ble_bls_init_t init;

	//Add Ball Lighting service
	init.irled_handler = irled_handler;
	init.wled_handler = wled_handler;

	err_code = ble_bls_init(&m_bls, &init);
	APP_ERROR_CHECK(err_code);

	//Add Battery service
	//TODO 
}


/**@brief Function for initializing security parameters.
 */
static void sec_params_init(void)
{
    m_sec_params.timeout      = SEC_PARAM_TIMEOUT;
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
*/
static void timers_start(void)
{
    /* YOUR_JOB: Start your timers. below is an example of how to start a timer.
    uint32_t err_code;

    err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code); */
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;

    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
    //nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    static ble_gap_evt_auth_status_t m_auth_status;
    ble_gap_enc_info_t *             p_enc_info;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
//            nrf_gpio_pin_set(CONNECTED_LED_PIN_NO);
//            nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

            /* YOUR_JOB: Uncomment this part if you are using the app_button module to handle button
                         events (assuming that the button events are only needed in connected
                         state). If this is uncommented out here,
                            1. Make sure that app_button_disable() is called when handling
                               BLE_GAP_EVT_DISCONNECTED below.
                            2. Make sure the app_button module is initialized.
            err_code = app_button_enable();
            APP_ERROR_CHECK(err_code);
            */
            break;

        case BLE_GAP_EVT_DISCONNECTED:
//            nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            /* YOUR_JOB: Uncomment this part if you are using the app_button module to handle button
                         events. This should be done to save power when not connected
                         to a peer.
            err_code = app_button_disable();
            APP_ERROR_CHECK(err_code);
            */
            
                advertising_start();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_SUCCESS,
                                                   &m_sec_params);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_AUTH_STATUS:
            m_auth_status = p_ble_evt->evt.gap_evt.params.auth_status;
            break;

        case BLE_GAP_EVT_SEC_INFO_REQUEST:
            p_enc_info = &m_auth_status.periph_keys.enc_info;
            if (p_enc_info->div == p_ble_evt->evt.gap_evt.params.sec_info_request.div)
            {
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, p_enc_info, NULL);
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                // No keys found for this device
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, NULL, NULL);
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
            {
 //               nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);

                // Configure buttons with sense level low as wakeup source.
   /*             nrf_gpio_cfg_sense_input(WAKEUP_BUTTON_PIN,
                                         BUTTON_PULL,
                                         NRF_GPIO_PIN_SENSE_LOW);
     */           
                // Go to system-off mode (this function will not return; wakeup will cause a reset)                
                err_code = sd_power_system_off();
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
	  ble_bls_on_ble_evt(&m_bls, p_ble_evt);
    /*
    YOUR_JOB: Add service ble_evt handlers calls here, like, for example:
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
    */
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, false);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
    
    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


/**@brief Function for initializing the GPIOTE handler module.
 * This function takes care of setting up the GPIO interrupts
 */
static void gpiote_init(void)
{
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
	
	//Setup pins and transitions to interrupt from
	uint32_t err_code;
	uint32_t lo_to_hi_bitmask = (1UL << ACC_INT1_PIN)
								& (1UL << ACC_INT2_PIN)
								& (1UL << CHRG_STAT_PIN); //pins to be notified of transition low -> high
	uint32_t hi_to_lo_bitmask = (1UL << CHRG_STAT_PIN); //pins to be notified of transition high -> low
	
	err_code = app_gpiote_user_register(&m_gpiote_user_id,
								lo_to_hi_bitmask,
								hi_to_lo_bitmask,
								gpiote_event_handler);
	
	APP_ERROR_CHECK(err_code);
	
	//Enable GPIOTE module
	err_code = app_gpiote_user_enable(m_gpiote_user_id);
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for application main entry.
 */
int main(void)
{
    // Initialize
    pins_init();
	//TODO adc_init();
	debug_init();
    timers_init();
    gpiote_init();	
    ble_stack_init();
	pwm_init(); //must be started AFTER soft device init !
    scheduler_init();    
    gap_params_init();
    advertising_init();
    services_init();
    conn_params_init();
    sec_params_init();
	
	debug_log("Init end \r\n");
	//TODO boot feedback
	m_current_state = STATE_STANDBY;

    // Start execution
    timers_start();
    advertising_start();

    // Enter main loop
    for (;;)
    {
        app_sched_execute();
        power_manage();
    }
}

/**
 * @}
 */
