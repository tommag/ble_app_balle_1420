/* Implementation of a library to give a simple user feedback using a single LED 
 * controlled via PWM
 * 
 * Two feedbacks are possible : 
 * PULSE type that sends a series of pulses to indicate an event
 * HEARTBEAT type that displays a continuous pattern to indicate an ongoing action
 *
 * This module uses an external application timer to handle LED patterns without delays
 * The user_feedback_timer_handler() has to be called to update LED intensity ; this function and the user_feedback_start() 
 * function return the delay time before calling the handler again.
 * The main application has to take care of setting up and updating a timer to call this handler at
 * the required intervals.
 *
 * 2014 - Tom Magnier : tom@tmagnier.fr
 */
 
#include "led_user_feedback.h"

const uint8_t heartbeat_states[USER_FB_HEARTBEAT_NB_STATES] =
	{0,1,3,6,10,15,22,38,70,94,100,94,70,38,22,15,10,6,3,1}; //Values in percentage of the max. intensity

user_feedback_config_t feedback_last_config; //Last config set up
bool feedback_on; //True if the feedback is running
uint16_t feedback_current_state; //Stores the current state 
//For HEARTBEAT this means the index in the states array
//For PULSE there are two states for each pulse (ON and OFF) - for example for a series of 3 pulses, state 1 is the first ON time, state 2 the first OFF time and so on.

user_feedback_pwm_update_callback_t pwm_callback; //The function to call on LED PWM updates
	
/**@brief Function for user feedback module initialization
 *
 * @param[in] callback: The function to call on LED PWM update
 */
void user_feedback_init(user_feedback_pwm_update_callback_t callback)
{
	pwm_callback = callback;
}

/**@brief Start a new user feedback with the provided configuration
 *
 * @param[in] p_new_config: A pointer to the config structure specifying the desired feedback
 *
 * @return the time in ms before calling user_feedback_timer_handler() or 0 if the config is invalid
 */
uint16_t user_feedback_start(user_feedback_config_t *p_new_config)
{
	feedback_last_config = *p_new_config;
	
	feedback_current_state = 0;
	feedback_on = true;
	
	return user_feedback_timer_handler();
}

/**@brief Stop the running user feedback and switch off the LED
 */
void user_feedback_stop(void)
{
	pwm_callback(0);
	feedback_on = false;
}

/**@brief Handle a timer event and update the LED intensity according to the current state
 * 
 * @return the time in ms before calling user_feedback_timer_handler() again or 0 if the user feedback is stopped
 */
uint16_t user_feedback_timer_handler(void)
{
	//If the feedback is stopped by the application, return immediately
	if(!feedback_on)
		return 0;
	
	uint16_t return_time = 0;
	
	if(feedback_last_config.mode == USER_FB_PULSE)
	{
		//In PULSE mode, the state variable represent ON/OFF state and the pulse number.
		//An even number means the LED is OFF and an odd number means the LED is ON.
		//At init : state 0, then 1 (ON) then 2 (OFF), 3 (ON), etc
		
		//light up the LED and return the on_time if we are in an even state
		if(feedback_current_state % 2 == 0)
		{
			pwm_callback(feedback_last_config.intensity);
			feedback_current_state++;
			return_time = feedback_last_config.time_on_ms;
		}
		
		//switch off the LED if we are in an odd state
		else
		{
			pwm_callback(0);
			feedback_current_state++;
			
			//return the off_time if the required number of pulses hasn't been displayed yet, 0 otherwise
			if(feedback_current_state < (feedback_last_config.num_pulses * 2))
			{
				return_time = feedback_last_config.time_off_ms;
			}
			else
			{
				feedback_on = false; //End of the pattern
				return_time = 0;
			}
		}
	}
	else if(feedback_last_config.mode == USER_FB_HEARTBEAT)
	{
		//In HEARTBEAT mode, the state variable represents the index in the states array.
		
		//Switch to next state
		feedback_current_state++;
		if(feedback_current_state == USER_FB_HEARTBEAT_NB_STATES)
			feedback_current_state = 0;
		
		//Values in the state array are in percentage of the max. intensity
		pwm_callback((heartbeat_states[feedback_current_state] * feedback_last_config.intensity) / 100); 
		
		//return the time before next event, which is the period divided by the number of states
		return_time = feedback_last_config.time_on_ms / USER_FB_HEARTBEAT_NB_STATES;
	}
	
	return return_time;
}
