/* Tom Magnier - 08/2014
 * This is a modification of the nRF PWM library to accomodate some of the MIC2860
 * LED driver limitations.
 * With a standard PWM, low duty cycles don't let the driver startup because there
 * is a 32us time between Shutdown and On states. Once the driver has started, it goes
 * into Standby state for 4ms when turned off and the Standby->On transition is much 
 * faster. Thus, low duty cycles are possible when the LED is already on but not when 
 * it is off. 
 *
 * This library tries to solve this problem by allowing the first PWM burst to be longer
 */

#include "nrf_pwm.h"
#include "nrf.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#if(USE_WITH_SOFTDEVICE == 1)
#include "nrf_sdm.h"
#include "app_error.h"
#endif

static uint32_t pwm_max_value, pwm_next_value[PWM_MAX_CHANNELS], pwm_next_max_value, pwm_io_ch[PWM_MAX_CHANNELS], pwm_running[PWM_MAX_CHANNELS];
static uint8_t pwm_gpiote_channel[3];
static uint32_t pwm_num_channels;

/* Minimum number of ticks on the first PWM cycle (LED driver startup) */
#define MIC2860_STARTUP_NB_TICKS			128		/* 32us @ 4MHz timer frequency */

void ppi_enable_channel(uint32_t ch_num, volatile uint32_t *event_ptr, volatile uint32_t *task_ptr)
{
    if(ch_num >= 16)
    {
        return;
    }
    else
    {
#if(USE_WITH_SOFTDEVICE == 1)
		uint32_t err_code;
		
        err_code=sd_ppi_channel_assign(ch_num, event_ptr, task_ptr);
		APP_ERROR_CHECK(err_code);
        sd_ppi_channel_enable_set(1 << ch_num);
#else
        // Otherwise we configure the channel and return the channel number
        NRF_PPI->CH[ch_num].EEP = (uint32_t)event_ptr;
        NRF_PPI->CH[ch_num].TEP = (uint32_t)task_ptr;    
        NRF_PPI->CHENSET = (1 << ch_num);   
#endif
    }
}

uint32_t nrf_pwm_init(nrf_pwm_config_t *config)
{
    if(config->num_channels < 1 || config->num_channels > 3) return 0xFFFFFFFF;
    
    switch(config->mode)
    {
 		case PWM_MODE_LED_4095:  // 0-4095 resolution, 488Hz PWM frequency, 2MHz timer frequency (prescaler 3)
            PWM_TIMER->PRESCALER = 2;
            pwm_max_value = 4095;
            break;
        default:
            return 0xFFFFFFFF;
    }
    pwm_num_channels = config->num_channels;
    for(int i = 0; i < pwm_num_channels; i++)
    {
        pwm_io_ch[i] = (uint32_t)config->gpio_num[i];
        nrf_gpio_cfg_output(pwm_io_ch[i]);
        pwm_running[i] = 0;       
        pwm_gpiote_channel[i] = config->gpiote_channel[i];        
    }
    PWM_TIMER->TASKS_CLEAR = 1;
	PWM_TIMER->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
    PWM_TIMER->CC[3] = pwm_next_max_value = pwm_max_value;
	PWM_TIMER->MODE = TIMER_MODE_MODE_Timer;
    PWM_TIMER->SHORTS = TIMER_SHORTS_COMPARE3_CLEAR_Msk;
    PWM_TIMER->EVENTS_COMPARE[0] = PWM_TIMER->EVENTS_COMPARE[1] = PWM_TIMER->EVENTS_COMPARE[2] = PWM_TIMER->EVENTS_COMPARE[3] = 0;     

    for(int i = 0; i < pwm_num_channels; i++)
    {
        ppi_enable_channel(config->ppi_channel[i*2],  &PWM_TIMER->EVENTS_COMPARE[i], &NRF_GPIOTE->TASKS_OUT[pwm_gpiote_channel[i]]);
        ppi_enable_channel(config->ppi_channel[i*2+1],&PWM_TIMER->EVENTS_COMPARE[3], &NRF_GPIOTE->TASKS_OUT[pwm_gpiote_channel[i]]);        
    }
#if(USE_WITH_SOFTDEVICE == 1)
	uint32_t err_code;
    err_code = sd_nvic_SetPriority(PWM_IRQn, PWM_IRQ_PRIORITY | 0x01);
	APP_ERROR_CHECK(err_code);
    err_code = sd_nvic_EnableIRQ(PWM_IRQn);
	APP_ERROR_CHECK(err_code);
#else
    NVIC_SetPriority(PWM_IRQn, PWM_IRQ_PRIORITY);
    NVIC_EnableIRQ(PWM_IRQn);
#endif
    PWM_TIMER->TASKS_START = 1;
    return 0;
}

void nrf_pwm_set_value(uint32_t pwm_channel, uint32_t pwm_value)
{
    pwm_next_value[pwm_channel] = pwm_value;
    PWM_TIMER->EVENTS_COMPARE[3] = 0;
    PWM_TIMER->SHORTS = TIMER_SHORTS_COMPARE3_CLEAR_Msk | TIMER_SHORTS_COMPARE3_STOP_Msk;
    if((PWM_TIMER->INTENSET & TIMER_INTENSET_COMPARE3_Msk) == 0)
    {
        PWM_TIMER->TASKS_STOP = 1;
        PWM_TIMER->INTENSET = TIMER_INTENSET_COMPARE3_Msk;  
    }
    PWM_TIMER->TASKS_START = 1;
}

void nrf_pwm_set_max_value(uint32_t max_value)
{
    pwm_next_max_value = max_value;
    PWM_TIMER->EVENTS_COMPARE[3] = 0;
    PWM_TIMER->SHORTS = TIMER_SHORTS_COMPARE3_CLEAR_Msk | TIMER_SHORTS_COMPARE3_STOP_Msk;
    if((PWM_TIMER->INTENSET & TIMER_INTENSET_COMPARE3_Msk) == 0)
    {
        PWM_TIMER->TASKS_STOP = 1;
        PWM_TIMER->INTENSET = TIMER_INTENSET_COMPARE3_Msk;  
    }
    PWM_TIMER->TASKS_START = 1;    
}

void nrf_pwm_set_enabled(bool enabled)
{
    if(enabled)
    {
        PWM_TIMER->INTENSET = TIMER_INTENSET_COMPARE3_Msk;
        PWM_TIMER->TASKS_START = 1;
    }
    else
    {
        PWM_TIMER->TASKS_STOP = 1;
        for(uint32_t i = 0; i < pwm_num_channels; i++)
        {
            nrf_gpiote_unconfig(pwm_gpiote_channel[i]);
            nrf_gpio_pin_write(pwm_io_ch[i], 0); 
            pwm_running[i] = 0;
        }       
    }
}

void PWM_IRQHandler(void)
{
    static uint32_t i;
    PWM_TIMER->EVENTS_COMPARE[3] = 0;
    PWM_TIMER->INTENCLR = 0xFFFFFFFF;
    PWM_TIMER->CC[3] = pwm_max_value = pwm_next_max_value;
	PWM_TIMER->SHORTS = TIMER_SHORTS_COMPARE3_CLEAR_Msk;

    for(i = 0; i < pwm_num_channels; i++)
    {
        if(pwm_next_value[i] == 0)
        {
            nrf_gpiote_unconfig(pwm_gpiote_channel[i]);
            nrf_gpio_pin_write(pwm_io_ch[i], 0);
            pwm_running[i] = 0;
        }
        else if (pwm_next_value[i] >= pwm_max_value)
        {
            nrf_gpiote_unconfig(pwm_gpiote_channel[i]);
            nrf_gpio_pin_write(pwm_io_ch[i], 1); 
            pwm_running[i] = 0;
        }
        else
        {
            PWM_TIMER->CC[i] = pwm_next_value[i];
            if(!pwm_running[i])
            {
                nrf_gpiote_task_config(pwm_gpiote_channel[i], pwm_io_ch[i], NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_HIGH);  
                pwm_running[i] = 1;
				
				/* Startup => allow more time to the LED driver if necessary */
				if(pwm_next_value[i] < MIC2860_STARTUP_NB_TICKS) 
				{
					PWM_TIMER->CC[i] = MIC2860_STARTUP_NB_TICKS;
					PWM_TIMER->INTENSET = TIMER_INTENSET_COMPARE3_Msk;
					PWM_TIMER->SHORTS |= TIMER_SHORTS_COMPARE3_STOP_Msk;
				}
            }
        }
    }
    PWM_TIMER->TASKS_START = 1;
}
