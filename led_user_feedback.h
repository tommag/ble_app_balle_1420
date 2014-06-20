/* Implementation of a library to give a simple user feedback using a single LED 
 * controlled via PWM
 * 
 * Two feedbacks are possible : 
 * PULSE type that sends a series of pulses to indicate an event
 * HEARTBEAT type that displays a continuous pattern to indicate an ongoing action
 *
 * 2014 - Tom Magnier : tom@tmagnier.fr
 */
 
#ifndef LED_USER_FEEDBACK_H
#define LED_USER_FEEDBACK_H
 
#include <stdint.h>
#include <stdbool.h>

//The number of different states to use in the HEARTBEAT pattern
#define USER_FB_HEARTBEAT_NB_STATES		10

typedef enum
{
	USER_FB_PULSE,			//Feedback sending a series of pulses (to indicate an event)
	USER_FB_HEARTBEAT,		//Feedback by a continuous pattern to indicate an ongoing action
} user_feedback_mode_t;


typedef struct
{
	user_feedback_mode_t	mode;			//Feedback mode to apply
	uint16_t				time_on_ms;		//For PULSE : duration of the ON state, for HEARTBEAT : period
	uint16_t				time_off_ms;	//For PULSE : duration of the OFF state
	uint8_t					intensity;		//Max. intensity of the LED
	uint8_t					num_pulses;		//Number of pulses to display (for PULSE)
} user_feedback_config_t;

//PWM value update function type
typedef void (*user_feedback_pwm_update_callback_t)(uint8_t value);

//Init the user feedback by providing the PWM callback function
void user_feedback_init(user_feedback_pwm_update_callback_t callback);

uint16_t user_feedback_start(user_feedback_config_t *p_new_config); 

void user_feedback_stop(void);

uint16_t user_feedback_timer_handler(void);

#endif  // LED_USER_FEEDBACK_H
