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
 * TODO documentation
 * watchdog ?
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
#include "ble_debug_assert_handler.h"
#include "pstorage.h"
#include "ble_ball_lighting_service.h"
#include "ble_bas.h" //Battery service
#include "nrf_pwm.h"
#include "debug.h"
#include "nrf_delay.h"
#include "mma8652/mma8652.h"
#include "led_user_feedback.h"

#define DEVICE_NAME                     "Balle 14:20 v1"                           /**< Name of device. Will be included in the advertising data. */

#define APP_ADV_INTERVAL                MSEC_TO_UNITS(40, UNIT_0_625_MS)            /**< The advertising interval (in units of 0.625 ms) Min 20 ms. */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            6                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         8                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(8, UNIT_1_25_MS)              /**< Minimum acceptable connection interval (8 ms). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(25, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (25 ms). */
#define SLAVE_LATENCY                   2                                           /**< Slave latency. */
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
static ble_bls_t						m_bls;										//Ball lighting service data structure
static ble_bas_t						m_bas;										//Battery service data structure

#define SCHED_MAX_EVENT_DATA_SIZE       sizeof(app_timer_event_t)                   /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#define SCHED_QUEUE_SIZE                40                                          /**< Maximum number of events in the scheduler queue. */

// Persistent storage system event handler
void pstorage_sys_event_handler (uint32_t p_evt);

//Battery voltage measurement parameters and variables
//TODO #define BATT_MAX_VOLTAGE_MV				4200	//The maximum battery voltage in mV (for percentage calculations)
//TODO #define	BATT_MIN_VOLTAGE_MV				3000	//The minimum battery voltage in mV, used for % calculation and low battery action
#define BATT_MAX_VOLTAGE_MV				3100	//The maximum battery voltage in mV (for percentage calculations)
#define	BATT_MIN_VOLTAGE_MV				2200	//The minimum battery voltage in mV, used for % calculation and low battery action
#define BATT_MEAS_AVG_FACTOR			3		//The inverse of the weight to use for the running average smoothing of the read value
//Example with 3 : new_voltage = new_meas/3 + old_voltage*2/3
#define BATT_MEAS_INTERVAL_MS			125		//The interval between two measurements (to set up the application timer)			
// 8Hz (125ms) is the highest frequency recommanded for this configuration (see adc_read_batt_voltage_mv function)
#define BATT_NOTIFICATION_INTERVAL_MIN_MS	4500	//The battery service will send notifications at non-periodic intervals. This is the lower boundary.
#define BATT_NOTIFICATION_INTERVAL_MAX_MS	5500	//The battery service will send notifications at non-periodic intervals. This is the upper boundary.

uint32_t m_batt_current_voltage_mv = 0;		//The current, smoothed out, battery voltage
uint32_t m_batt_percentage;					//The current battery percentage calculated from voltage
app_timer_id_t m_adc_timer_id;				//The application timer ID that will start periodically the ADC
app_timer_id_t m_batt_srv_timer_id;			//The application timer ID that will start the battery service notification at random intervals.

//Accelerometer parameters
#define ACCEL_SLEEP_DATARATE			(MMA8652_CTRL_REG1_ASLP_RATE_6_25_HZ) //6.25Hz sample frequency when in SLEEP mode
#define ACCEL_WAKE_DATARATE				(MMA8652_CTRL_REG1_DATARATE_6_25_HZ)	//6.25Hz sample frequency when in WAKE mode
#define ACCEL_SLEEP_POWER_MODE			(MMA8652_CTRL_REG2_SMODS_LP)	//Low power mode when in SLEEP mode
#define ACCEL_WAKE_POWER_MODE			(MMA8652_CTRL_REG2_MODS_LP)	//Low power mode when in WAKE mode
#define	ACCEL_MOTION_THRESHOLD			(16)	//Unit : 0.063 g/LSB, 0 -> 127 counts
#define ACCEL_MOTION_DELAY				(2)	//see Table 57. in LP mode, 6.25Hz : time step = 160ms. Max value: 255

//LED user feedback variables
app_timer_id_t m_led_feedback_timer_id;		//The app. timer ID that will call the user feedback handler

//Activity timeout timer
#define ACTIVITY_TIMEOUT_S				3600	//Power down the device after 1 hour spent in STANDBY mode
app_timer_id_t m_activity_timer_id;				//The app. timer ID that will power down the device after too much time spent in STANDBY state

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

//Forward declarations of functions needed by the state machine
static void advertising_start(void);
static void led_feedback_start(user_feedback_config_t *p_feedback_config);
static void led_feedback_stop(void);
static void wled_intensity_update(uint8_t new_value);


/* Actions to do when entering the STANDBY state :
 * Start a timer to detect the activity timeout
 * Perform energy saving optimizations
 * Setup accelerometer to detect motion events
 */
static void action_enter_standby(void)
{
	m_current_state = STATE_STANDBY;
	//TODO energy optimizations ? slow down ADC interrupt ?
	//TODO accelerometer setup ?
	
	//Start activity timer
	app_timer_start(m_activity_timer_id, APP_TIMER_TICKS(ACTIVITY_TIMEOUT_S*1000, APP_TIMER_PRESCALER), NULL);
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
	//TODO have a look at the GPIOTE module drawing 1mA
	
	debug_log("[APPL]: Low battery / no activity, powering down \r\n");
	
	//Turn off accelerometer
//TODO TEMP	mma8652_standby();
	
	//Turn off both LEDs
	nrf_gpio_pin_clear(IRLED_PIN);
	wled_intensity_update(0);
	
	// Configure battery charge IC with sense level low as wakeup source.
	nrf_gpio_cfg_sense_input(CHRG_STAT_PIN,
							 NRF_GPIO_PIN_PULLUP,
							 NRF_GPIO_PIN_SENSE_LOW);
	
	// Go to system-off mode (this function will not return; wakeup will cause a reset)                
	sd_power_system_off();
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
			
			//Start user feedback (slow heartbeat)
			user_feedback_config_t fb_config;
			fb_config.mode = USER_FB_HEARTBEAT;
			fb_config.intensity = 64;
			fb_config.time_on_ms = 3000;
			led_feedback_start(&fb_config);
		
			//Stop activity timer
			app_timer_stop(m_activity_timer_id);
	
			//Start advertising
			advertising_start();
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
		{
			//User feedback - 3 short pulses
			user_feedback_config_t fb_config;
			fb_config.mode = USER_FB_PULSE;
			fb_config.intensity = 255;
			fb_config.num_pulses = 3;
			fb_config.time_on_ms = 100;
			fb_config.time_off_ms = 100;
			led_feedback_start(&fb_config);

			action_enter_standby();
		}
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
			//Start advertising
			advertising_start();
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
		{
			//User feedback : 1 short bright pulse
			user_feedback_config_t fb_config;
			fb_config.mode = USER_FB_PULSE;
			fb_config.intensity = 255;
			fb_config.num_pulses = 1;
			fb_config.time_on_ms = 100;
			fb_config.time_off_ms = 100;
			led_feedback_start(&fb_config);
		}
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
		{
			//User feedback : 1 long light pulse
			user_feedback_config_t fb_config;
			fb_config.mode = USER_FB_PULSE;
			fb_config.intensity = 127;
			fb_config.num_pulses = 1;
			fb_config.time_on_ms = 1500;
			fb_config.time_off_ms = 100;
			led_feedback_start(&fb_config);
		}
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
	
	debug_log("[APPL]: Event 0x%02x occurred, new state : 0x%02x \r\n", event, m_current_state);
}

/**@brief Function called when an interrupt is sent by the accelerometer via the Scheduler
 * 
 * @details Assert the interrupt then call the related application event
 * 
 * @param[in] int_num  Interrupt line asserted (1 or 2)
 */
static void on_accelerometer_interrupt(void * p_event_data, uint16_t event_size)
{
	on_app_event(ON_ACCELEROMETER_EVENT);
}

/**@brief Funtion called when the activity timer expires (too much time spent in STANDBY mode). 
 */
static void activity_timer_handler(void)
{
	on_app_event(ON_ACTIVITY_TIMEOUT);
}

/**@brief Function called when the battery charge stops, either because it is complete or the charger is disconnected
 */
static void on_battery_charge_stop(void * p_event_data, uint16_t event_size)
{
	//Do nothing
	debug_log("[APPL]: Battery charge stop \r\n");
}

/**@brief Function called when the battery charge starts
 */
static void on_battery_charge_start(void * p_event_data, uint16_t event_size)
{
	//Do nothing
	debug_log("[APPL]: Battery charge start \r\n");
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
	
	nrf_delay_ms(10000); //TODO !

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



/**@brief Function for GPIOTE events handling, called when a GPIO interrupt occurs
 * GPIOTE pins and transitions from which events occur are defined in gpiote_init() function
 */
void gpiote_event_handler(uint32_t event_pins_lo_to_hi, uint32_t event_pins_hi_to_lo)
{
	//if ACC_INT1 or ACC_INT2 transitioned from high to low, call the accelerometer handler
	uint32_t int_num = 0;
	if(event_pins_hi_to_lo & (1UL << ACC_INT1_PIN))
	{
		int_num = 1; 
		app_sched_event_put(&int_num, sizeof(int_num), on_accelerometer_interrupt);
	}
	
	if(event_pins_hi_to_lo & (1UL << ACC_INT2_PIN))
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
	//Init IR LED output
	nrf_gpio_cfg_output(IRLED_PIN); //simple output
	//TODO TEMP
	NRF_GPIO->PIN_CNF[IRLED_PIN] &= ~(GPIO_PIN_CNF_DRIVE_Msk);
	NRF_GPIO->PIN_CNF[IRLED_PIN] |= (GPIO_PIN_CNF_DRIVE_S0H1 << GPIO_PIN_CNF_DRIVE_Pos);

	
	//Init accelerometer interrupt inputs with no pull resistor
	//The GPIOTE handling library takes care of defining the SENSE fields
//TODO TEMP	nrf_gpio_cfg_sense_input(ACC_INT1_PIN, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_NOSENSE);
//	nrf_gpio_cfg_sense_input(ACC_INT2_PIN, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_NOSENSE);
	nrf_gpio_cfg_sense_input(ACC_INT1_PIN, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_NOSENSE);
	nrf_gpio_cfg_sense_input(ACC_INT2_PIN, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_NOSENSE);
	
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
	
	//TODO TEMP
	NRF_GPIO->PIN_CNF[WLED_PIN] &= ~(GPIO_PIN_CNF_DRIVE_Msk);
	NRF_GPIO->PIN_CNF[WLED_PIN] |= (GPIO_PIN_CNF_DRIVE_S0H1 << GPIO_PIN_CNF_DRIVE_Pos);
}

/**@brief Function for updating the intensity of the white LED
 * 
 * @param[in] new_value: the new 8-bit value to pass to the PWM module
 */
static void wled_intensity_update(uint8_t new_value)
{
	nrf_pwm_set_value(0, new_value); // channel 0 for the PWM module
}

/**@brief Function that generates an interval for the BAS notification using the RNG
 * Generate and return an interval in ms between BATT_NOTIFICATION_INTERVAL_MIN_MS and BATT_NOTIFICATION_INTERVAL_MAX_MS,
 * using the Random Number Generator module (via the SoftDevice)
 * 
 * @return an randomly generated interval in ms, between the set boundaries
 */
static uint32_t batt_srv_generate_interval(void)
{
	uint8_t random_number, num_bytes_avail;
	
	//Wait for available random number
	do 
		sd_rand_application_bytes_available_get(&num_bytes_avail);
	while(num_bytes_avail == 0);
		
	
	uint32_t err_code = sd_rand_application_vector_get(&random_number, 1);
	APP_ERROR_CHECK(err_code);
	
	uint32_t interval_ms = BATT_NOTIFICATION_INTERVAL_MIN_MS + (random_number * (BATT_NOTIFICATION_INTERVAL_MAX_MS - BATT_NOTIFICATION_INTERVAL_MIN_MS)) / 255; //RNG returns a 8-bit random number
	
	//debug_log("[APPL]: Random interval generated : %u \r\n", interval_ms);
	
	return interval_ms;
}


/**@brief Function for handling battery service notification timer timeout
 * This function is called when the BAS notification timer times out, at random intervals between
 * BATT_NOTIFICATION_INTERVAL_MIN_MS and BATT_NOTIFICATION_INTERVAL_MAX_MS. This is to avoid patterns
 * when a high number of balls are being managed by the same Master : battery notifications will happen at random times.
 *
 * This function takes the latest calculated battery percentage and passes it to the Battery service, 
 * then set up the timer for the next notification
 */
static void batt_srv_timer_handler(void)
{
	uint32_t err_code;
	
	//Update BAS battery level if a valid connection is established
	if(m_bas.conn_handle != BLE_CONN_HANDLE_INVALID && m_bas.is_notification_supported)
	{
		err_code = ble_bas_battery_level_update(&m_bas, m_batt_percentage); //will return NRF_ERROR_INVALID_STATE if not connected
		//APP_ERROR_CHECK(err_code); //disabled because fire a NRF_ERROR_INVALID_STATE if client hasn't enabled notifications
	}
	
	//Setup new timer
	err_code = app_timer_start(m_batt_srv_timer_id, APP_TIMER_TICKS(batt_srv_generate_interval(), APP_TIMER_PRESCALER) , NULL);
	APP_ERROR_CHECK(err_code);
}


/**@brief Function for reading the battery voltage, using the last ADC measurement
 * 
 * @return The measured value in mV
 */
static uint32_t adc_read_batt_voltage_mv(void)
{
	//ADC configuration : reference = 1200 mV band gap, no input prescaling
	//Battery voltage is measured through a 2.2M/10M voltage divider so we have to 
	//multiply the read value by (10M+2.2M)/2.2M = 61/11
	
	//Read ADC measurement
	uint32_t value = NRF_ADC->RESULT;
	
	//The measurement has to be * (61/11) (voltage divider) and * 1200/1023 (ADC reference/resolution)
	value = ((value*61*1200)/11)/1023;
	
	return value;
}


/**@brief Function for handling the battery voltage measurement
 * This function is called by the ADC IRQ handler at the end of a conversion
 * which must be started by adc_start();
 * 
 * This function takes care of smoothing the variations of the voltage. It stores
 * the new voltage in the global variable m_batt_current_voltage_mv and calculates
 * the battery percentage, stored in m_batt_percentage.
 *
 * This function also takes care of calling the low battery routines.
 */
static void adc_process_new_measurement(void)
{
	//Retrieve the latest measurement and convert it to mV
	uint32_t new_meas = adc_read_batt_voltage_mv();
	
	//Calculate the current voltage based on the previous ones and the new measurement
	//We use a running average of factor 1/BATT_MEAS_AVG_FACTOR
	m_batt_current_voltage_mv = new_meas/BATT_MEAS_AVG_FACTOR + 
		m_batt_current_voltage_mv*(BATT_MEAS_AVG_FACTOR-1)/BATT_MEAS_AVG_FACTOR;
	
	//Calculate the percentage
	//At the moment, very simple algorithm assuming that the voltage variation is linear (obviously false)
	m_batt_percentage = ((m_batt_current_voltage_mv-BATT_MIN_VOLTAGE_MV)*100) / (BATT_MAX_VOLTAGE_MV-BATT_MIN_VOLTAGE_MV);
	
	//Check that the percentage is between 0 and 100 !
	if(m_batt_current_voltage_mv < BATT_MIN_VOLTAGE_MV)
		m_batt_percentage = 0;
	else if(m_batt_percentage > 100)
		m_batt_percentage = 100;
	
	//If the new value is lower than the min allowed voltage and the battery is not charging :
	//go into power down mode
	if((m_batt_current_voltage_mv < BATT_MIN_VOLTAGE_MV) && 
			(nrf_gpio_pin_read(CHRG_STAT_PIN) == 1))
	{
		on_app_event(ON_LOW_BATT);
	}
	
	//debug_log("[APPL]: New battery voltage : %u mV, %u %% \r\n", m_batt_current_voltage_mv, m_batt_percentage);
}


/**@brief Function for triggering an ADC reading 
 * The result will be processed in the ADC IRQ handler
 */
static void adc_start(void)
{
	NRF_ADC->TASKS_START = 1;
}


/**@brief Function for the ADC block initialization
 * Used to read the battery voltage
 * The analog pin to read from must be defined with : 
 * #define BATT_VOLTAGE_AIN_NO		ADC_CONFIG_PSEL_AnalogInput0
 *
 * This function also initializes the battery voltage global variable (for the future averages)
 */
static void adc_init(void)
{
	uint32_t err_code;
	
	//Recommended configuration found at :
	//https://devzone.nordicsemi.com/question/990/how-to-measure-lithium-battery-voltage/
	//10-bit mode, analog input without prescaling, reference : 1.V band gap
	//analog input : set by define BATT_VOLTAGE_AIN_NO
	//no external reference
	
	NRF_ADC->CONFIG = 	(ADC_CONFIG_RES_10bit 						<< ADC_CONFIG_RES_Pos) |
						(ADC_CONFIG_INPSEL_AnalogInputNoPrescaling	<< ADC_CONFIG_INPSEL_Pos) |
						(ADC_CONFIG_REFSEL_VBG						<< ADC_CONFIG_REFSEL_Pos) |
						(BATT_VOLTAGE_AIN_NO						<< ADC_CONFIG_PSEL_Pos) |
						(ADC_CONFIG_EXTREFSEL_None					<< ADC_CONFIG_EXTREFSEL_Pos); 

	//Enable ADC.
	NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;
	
	//Trigger a new conversion and wait for the result
	NRF_ADC->EVENTS_END = 0; //Clear the flag
	adc_start();
	
	//Wait for the end of the conversion
	while(!NRF_ADC->EVENTS_END);
	
	NRF_ADC->EVENTS_END = 0; //Clear the flag
	//Get the result
	m_batt_current_voltage_mv = adc_read_batt_voltage_mv(); //Initialize the voltage variable
	adc_process_new_measurement(); //Initialize the battery percentage before starting the BLE Battery service
	
	//Setup ADC interrupt
	NRF_ADC->INTENSET = ADC_INTENSET_END_Enabled << ADC_INTENSET_END_Pos;
	err_code = sd_nvic_SetPriority(ADC_IRQn, NRF_APP_PRIORITY_LOW);
	APP_ERROR_CHECK(err_code);
	err_code = sd_nvic_EnableIRQ(ADC_IRQn); 
	APP_ERROR_CHECK(err_code);
}

/**@brief ADC interrupt handler, executed at the end of a conversion
 */
void ADC_IRQHandler(void)
{
	//Clear data ready event
	NRF_ADC->EVENTS_END = 0;
	
	//Schedule the read_batt_voltage to be processed at the end of the event queue
	app_sched_event_put(NULL, 0, adc_process_new_measurement);
}

/**@brief Function for the MMA8652 accelerometer initialization
 */
static void accelerometer_init(void)
{
	bool success; 
	//I2C init
	success = mma8652_init();
	APP_ERROR_CHECK_BOOL(success);
	
	/*Only the motion detection functionnality is used so there is no need to change most of the default settings */
	
	//Setup data rates
	success = mma8652_i2c_register_write(MMA8652_REG_CTRL_REG1, 0x00
		| ACCEL_SLEEP_DATARATE		//SLEEP mode sample frequency 
		| ACCEL_WAKE_DATARATE);	//WAKE mode sample frequency
	APP_ERROR_CHECK_BOOL(success);
	
	//Setup auto-sleep and power schemes selection
	success = mma8652_i2c_register_write(MMA8652_REG_CTRL_REG2, 0x00
		| ACCEL_WAKE_POWER_MODE		//WAKE power mode
		| ACCEL_SLEEP_POWER_MODE	//SLEEP power mode
		| MMA8652_CTRL_REG2_SLPE); //Auto-SLEEP enable (not very useful if both datarates and power modes are the same in SLEEP and WAKE modes)
	APP_ERROR_CHECK_BOOL(success);

	//Setup wake up interrupts
	success = mma8652_i2c_register_write(MMA8652_REG_CTRL_REG3, 0x00
		& ~MMA8652_CTRL_REG3_PP_OD	//Push pull interrupt outputs
		& ~MMA8652_CTRL_REG3_IPOL	//Active low interrupt outputs
		| MMA8652_CTRL_REG3_WAKE_FF_MT);	//Wake up from Freefall/motion detection
	APP_ERROR_CHECK_BOOL(success);
	
	//Setup external interrupts
	success = mma8652_i2c_register_write(MMA8652_REG_CTRL_REG4, 0x00
		| MMA8652_CTRL_REG4_INT_EN_FF_MT); //Enable interrupt from freefall/motion detection
	APP_ERROR_CHECK_BOOL(success);
	
	//Setup interrupt routing (to interrupt line 1 or 2)
	success = mma8652_i2c_register_write(MMA8652_REG_CTRL_REG5, 0x00
		| MMA8652_CTRL_REG5_INT_CFG_FF_MT); //Route FF/MT int to INT1
	APP_ERROR_CHECK_BOOL(success);
	
	//Setup auto-sleep inactivity timeout
	success = mma8652_i2c_register_write(MMA8652_REG_ASLP_COUNT,
		0xFF); //Setup maximum time since auto-sleep is not really used. 
	APP_ERROR_CHECK_BOOL(success);
	
	//Setup Freefall/motion detection
	success = mma8652_i2c_register_write(MMA8652_REG_FF_MT_CFG, 0x00 
		& ~MMA8652_FF_MT_CFG_ELE	//ELE bit = 0 => interrupt indicate real time status
		| MMA8652_FF_MT_CFG_OAE		//OAE bit = 1 => motion detection
		| MMA8652_FF_MT_CFG_XEFE	//enable detection on all 3 axis
		| MMA8652_FF_MT_CFG_YEFE	//enable detection on all 3 axis
		| MMA8652_FF_MT_CFG_ZEFE);	//enable detection on all 3 axis
	APP_ERROR_CHECK_BOOL(success);
	
	//Set motion detection threshold
	success = mma8652_i2c_register_write(MMA8652_REG_FF_MT_THS, 
		ACCEL_MOTION_THRESHOLD & MMA8652_FF_MT_THS_MSK);
	APP_ERROR_CHECK_BOOL(success);
	
	//Set motion detection delay
	success = mma8652_i2c_register_write(MMA8652_REG_FF_MT_COUNT,
		ACCEL_MOTION_DELAY);
	APP_ERROR_CHECK_BOOL(success);
	
	//Enter ACTIVE mode
	success = mma8652_active();
	APP_ERROR_CHECK_BOOL(success);
}

/**@brief Function for the LED user feedback initialization
 * This module is used to indicate some events to the user by pulsing the white LED
 */
static void led_feedback_init()
{
	user_feedback_init(wled_intensity_update);
}

/**@brief Function for handling user feedback timer timeout
 * called by the application timer
 * Calls the user feedback module handler and starts again the timer if necessary
 */
static void led_feedback_timer_handler(void)
{
	uint16_t new_time = user_feedback_timer_handler();
	
	if(new_time != 0)
	{
		uint32_t err_code = app_timer_start(m_led_feedback_timer_id, APP_TIMER_TICKS(new_time, APP_TIMER_PRESCALER) , NULL);
		APP_ERROR_CHECK(err_code);
	}
}

/**@brief Start a new user feedback process using the provided configuration struct
 * Set up the user feedback module then start an application timer that will
 * call the module for the next event
 *
 * @param[in] p_feedback_config: a pointer to the config to use
 */
static void led_feedback_start(user_feedback_config_t *p_feedback_config)
{
	uint16_t time = user_feedback_start(p_feedback_config);
	
	if(time != 0)
	{
		uint32_t err_code = app_timer_start(m_led_feedback_timer_id, APP_TIMER_TICKS(time, APP_TIMER_PRESCALER) , NULL);
		APP_ERROR_CHECK(err_code);
	}
}

/**@brief Stop the running user feedback process 
 */
static void led_feedback_stop(void)
{
	user_feedback_stop();
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, true);

	uint32_t err_code;
	
	//Register the ADC timer
    err_code = app_timer_create(&m_adc_timer_id, APP_TIMER_MODE_REPEATED, adc_start);
    APP_ERROR_CHECK(err_code);
	
	//Register the battery service notification timer
	err_code = app_timer_create(&m_batt_srv_timer_id, APP_TIMER_MODE_SINGLE_SHOT, batt_srv_timer_handler);
	APP_ERROR_CHECK(err_code);
	
	//Register the LED user feedback timer
	err_code = app_timer_create(&m_led_feedback_timer_id, APP_TIMER_MODE_SINGLE_SHOT, led_feedback_timer_handler);
	APP_ERROR_CHECK(err_code);
	
	//Register the activity timeout timer
	err_code = app_timer_create(&m_activity_timer_id, APP_TIMER_MODE_SINGLE_SHOT, activity_timer_handler);
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
	wled_intensity_update(new_value);
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
	ble_bls_init_t bls_init;
	ble_bas_init_t bas_init;

	//Add Ball Lighting service
	bls_init.irled_handler = irled_handler;
	bls_init.wled_handler = wled_handler;

	err_code = ble_bls_init(&m_bls, &bls_init);
	APP_ERROR_CHECK(err_code);

	//Add Battery service
	memset(&bas_init, 0, sizeof(bas_init));
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm); //Allow GATT client to modify CCCD config (to disable notifications ?)
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm); //Allow reading on open links
	BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm); //No writing of the battery level by the client
	
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm); //Allow reading of battery level report char.
	
	bas_init.evt_handler = NULL;
	bas_init.initial_batt_level = m_batt_percentage;
	bas_init.p_report_ref = NULL; //No Report Reference descriptor needed
	bas_init.support_notification = true;
	
	err_code = ble_bas_init(&m_bas, &bas_init);
	APP_ERROR_CHECK(err_code);
	
	//TODO device information service ??
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
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE); //TODO un peu violent non ?
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
    uint32_t err_code;
	
	//Start the ADC timer
    err_code = app_timer_start(m_adc_timer_id, APP_TIMER_TICKS(BATT_MEAS_INTERVAL_MS, APP_TIMER_PRESCALER) , NULL);
    APP_ERROR_CHECK(err_code); 
	
	//Start the BAS notification timer
	err_code = app_timer_start(m_batt_srv_timer_id, APP_TIMER_TICKS(batt_srv_generate_interval(), APP_TIMER_PRESCALER) , NULL);
	APP_ERROR_CHECK(err_code); 
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
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
		
			//Set system attributes upon connection to avoid getting BLE_GATTS_EVT_SYS_ATTR_MISSING error
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0);
            APP_ERROR_CHECK(err_code);
			
			//Call application event
			on_app_event(ON_CONNECTION_SUCCESS);
		
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
			
			if(p_ble_evt->evt.gap_evt.params.disconnected.reason == BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION)
				on_app_event(ON_DISCONNECT_RQ);
			else 
				on_app_event(ON_CONNECTION_TIMEOUT); //Handle all failures the same way as a timeout
            break;

		//TODO security, parameters, etc ???
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_SUCCESS,
                                                   &m_sec_params);
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
				on_app_event(ON_ADVERTISING_TIMEOUT);
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
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
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
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_SYNTH_250_PPM, false);

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
	uint32_t hi_to_lo_bitmask = (1UL << ACC_INT1_PIN)
								| (1UL << ACC_INT2_PIN)
								| (1UL << CHRG_STAT_PIN); //pins to be notified of transition high -> low
	uint32_t lo_to_hi_bitmask = (1UL << CHRG_STAT_PIN); //pins to be notified of transition low -> high
	
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
	debug_init();
    pins_init();
	//accelerometer_init();
    timers_init();
    gpiote_init();
	led_feedback_init();
    ble_stack_init();
	pwm_init(); //must be started AFTER soft device init ! otherwise we can't register IRQs
	adc_init(); //must be started AFTER soft device init !
    scheduler_init();    
    gap_params_init();
    advertising_init();
    services_init();
    conn_params_init();
    sec_params_init();
	
	debug_log("[APPL]: Init end \r\n");
	
	//Boot feedback
	user_feedback_config_t fb_config;
	fb_config.mode = USER_FB_PULSE;
	fb_config.intensity = 255;
	fb_config.num_pulses = 1;
	fb_config.time_on_ms = 100;
	fb_config.time_off_ms = 0;
	led_feedback_start(&fb_config);
	
    // Start execution
    timers_start();
	m_current_state = STATE_STANDBY;

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
