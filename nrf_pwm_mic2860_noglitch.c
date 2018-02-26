#include "nrf_pwm_noglitch.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"

#if(USE_WITH_SOFTDEVICE == 1)
#include "nrf_sdm.h"
#endif

static uint32_t pwm_max_value, pwm_io_ch[PWM_MAX_CHANNELS], pwm_period_us;
static uint8_t pwm_gpiote_channel[3], pwm_ppi_chg;
static uint32_t pwm_num_channels;

/* Minimum number of ticks on the first PWM cycle (LED driver startup) */
#define MIC2860_STARTUP_NB_TICKS			128		/* 32us @ 4MHz timer frequency */

static void ppi_enable_channels(uint32_t ch_msk)
{
#if(USE_WITH_SOFTDEVICE == 1)
    sd_ppi_channel_enable_set(ch_msk);
#else
    NRF_PPI->CHENSET = (ch_msk);   
#endif
}

static void ppi_disable_channels(uint32_t ch_msk)
{
#if(USE_WITH_SOFTDEVICE == 1)
    sd_ppi_channel_enable_clr(ch_msk);
#else
    NRF_PPI->CHENCLR = (ch_msk);   
#endif
}

static void ppi_configure_channel(uint32_t ch_num, volatile uint32_t *event_ptr, volatile uint32_t *task_ptr)
{
    if(ch_num >= 16)
    {
        return;
    }
    else
    {
#if(USE_WITH_SOFTDEVICE == 1)
        sd_ppi_channel_assign(ch_num, event_ptr, task_ptr);
#else
        NRF_PPI->CH[ch_num].EEP = (uint32_t)event_ptr;
        NRF_PPI->CH[ch_num].TEP = (uint32_t)task_ptr;    
#endif
    }
}

static void ppi_configure_channel_group(uint32_t ch_grp, uint32_t chg_ch_mask)
{
#if(USE_WITH_SOFTDEVICE == 1)
        sd_ppi_group_assign(ch_grp, chg_ch_mask);
#else
        NRF_PPI->CHG[ch_grp] = chg_ch_mask;
#endif
}

static void ppi_enable_channel_group(uint32_t ch_grp)
{
#if(USE_WITH_SOFTDEVICE == 1)
        sd_ppi_group_task_enable(ch_grp);
#else
        NRF_PPI->TASKS_CHG[ch_grp].EN = 1;  
#endif
}

static void ppi_disable_channel_group(uint32_t ch_grp)
{
#if(USE_WITH_SOFTDEVICE == 1)
        sd_ppi_group_task_disable(ch_grp);
#else
        NRF_PPI->TASKS_CHG[ch_grp].DIS = 1;  
#endif
}

static void pwm_freq_int_set(void)
{
    PWM_TIMER->EVENTS_COMPARE[3] = 0;
    PWM_TIMER->INTENSET          = TIMER_INTENSET_COMPARE3_Msk;
}

void PWM_IRQHandler(void)
{
    PWM_TIMER->EVENTS_COMPARE[3] = 0;
    PWM_TIMER->INTENCLR          = TIMER_INTENCLR_COMPARE3_Msk;
    
    ppi_disable_channels((1 << (PWM_MAX_CHANNELS * 2)) | (1 << (PWM_MAX_CHANNELS * 2 + 1)) | (1 << (PWM_MAX_CHANNELS * 2 + 2)));
    ppi_configure_channel_group(pwm_ppi_chg, 0);
    ppi_disable_channel_group(pwm_ppi_chg);
}

uint32_t nrf_pwm_init(nrf_pwm_config_t *config)
{
    static volatile uint32_t err_code;
    
    if(config->num_channels < 1 || config->num_channels > 2) return 0xFFFFFFFF;
    
    switch(config->mode)
    {
		case PWM_MODE_LED_255:   // 8-bit resolution, 122 Hz PWM frequency, 65 kHz timer frequency (prescaler 8)
            PWM_TIMER->PRESCALER = 8;
            pwm_max_value = 255;     
            pwm_period_us = (pwm_max_value * 2 * 1000000) / (16000000 / 256);        
            break;
		case PWM_MODE_LED_4095:  // 0-4095 resolution, 488Hz PWM frequency, 2MHz timer frequency (prescaler 2)
            PWM_TIMER->PRESCALER = 2;
            pwm_max_value = 4095;
            pwm_period_us = (pwm_max_value * 2 * 1000000) / (16000000 / 4);        
            break;
        default:
            return 0xFFFFFFFF;
    }
    pwm_ppi_chg      = config->ppi_group[0];
    pwm_num_channels = config->num_channels;
    for(int i = 0; i < pwm_num_channels; i++)
    {
        pwm_io_ch[i] = (uint32_t)config->gpio_num[i];
        nrf_gpio_cfg_output(pwm_io_ch[i]);
        pwm_gpiote_channel[i] = config->gpiote_channel[i];        
    }
    PWM_TIMER->TASKS_CLEAR = 1;
	PWM_TIMER->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
    PWM_TIMER->CC[3] = pwm_max_value*2;
	PWM_TIMER->MODE = TIMER_MODE_MODE_Timer;
    PWM_TIMER->SHORTS = TIMER_SHORTS_COMPARE3_CLEAR_Msk;
    PWM_TIMER->EVENTS_COMPARE[0] = PWM_TIMER->EVENTS_COMPARE[1] = PWM_TIMER->EVENTS_COMPARE[2] = PWM_TIMER->EVENTS_COMPARE[3] = 0;     

    for(int i = 0; i < pwm_num_channels; i++)
    {
        ppi_configure_channel(config->ppi_channel[i*2],   &PWM_TIMER->EVENTS_COMPARE[i], &NRF_GPIOTE->TASKS_OUT[pwm_gpiote_channel[i]]);
        ppi_configure_channel(config->ppi_channel[i*2+1], &PWM_TIMER->EVENTS_COMPARE[3], &NRF_GPIOTE->TASKS_OUT[pwm_gpiote_channel[i]]);        
    }
    
    #if(USE_WITH_SOFTDEVICE == 1)
    sd_nvic_SetPriority(PWM_IRQn, 1);
    sd_nvic_ClearPendingIRQ(PWM_IRQn);
    sd_nvic_EnableIRQ(PWM_IRQn);
    #else
    NVIC_SetPriority(PWM_IRQn, 3);
    NVIC_ClearPendingIRQ(PWM_IRQn);
    NVIC_EnableIRQ(PWM_IRQn);
    #endif

    PWM_TIMER->TASKS_START = 1;
    
    return 0;
}

uint32_t nrf_pwm_get_max_value(void)
{
    return pwm_max_value;
}

void nrf_pwm_set_value(uint32_t pwm_channel, uint32_t pwm_value)
{
    if (pwm_value == PWM_TIMER->CC[pwm_channel])
    {
        // No change necessary
        return;
    }
    
    if (PWM_TIMER->CC[pwm_channel] == 0)
    {
        // This PWM is not running
        if (pwm_value == 0)
        {
            // Corner case: This PWM is not running and new value is 0% duty cycle
            NRF_GPIO->OUTCLR = (1 << pwm_io_ch[pwm_channel]);
            PWM_TIMER->CC[pwm_channel] = pwm_value;
            return;
        }
        else if (pwm_value >= pwm_max_value)
        {
            // Corner case: This PWM is not running and new value is 100% duty cycle
            NRF_GPIO->OUTSET = (1 << pwm_io_ch[pwm_channel]);
            PWM_TIMER->CC[pwm_channel] = pwm_value;
            return;
        }
        
        ppi_disable_channels((1 << (pwm_channel * 2)) | (1 << (pwm_channel * 2 + 1)));
        PWM_TIMER->CC[pwm_channel] = pwm_value * 2;
        
        if (NRF_GPIO->OUT & (1 << pwm_io_ch[pwm_channel]))
        {
            // PWM is currently in 100% duty cycle
            nrf_gpiote_task_config(pwm_gpiote_channel[pwm_channel], pwm_io_ch[pwm_channel], NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_HIGH);
            
            ppi_configure_channel_group(pwm_ppi_chg, 0);
            ppi_disable_channel_group(pwm_ppi_chg);
            ppi_configure_channel_group(pwm_ppi_chg, (1 << (pwm_channel * 2)) | (1 << (pwm_channel * 2 + 1)));
            
            ppi_disable_channels(1 << (PWM_MAX_CHANNELS * 2));
            ppi_configure_channel(PWM_MAX_CHANNELS * 2, &PWM_TIMER->EVENTS_COMPARE[3], &NRF_PPI->TASKS_CHG[pwm_ppi_chg].EN);
            ppi_enable_channels(1 << (PWM_MAX_CHANNELS * 2));
        }
        else
        {
            // PWM is currently in 0% duty cycle
            nrf_gpiote_task_config(pwm_gpiote_channel[pwm_channel], pwm_io_ch[pwm_channel], NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
            
            ppi_configure_channel_group(pwm_ppi_chg, 0);
            ppi_disable_channel_group(pwm_ppi_chg);
            ppi_configure_channel_group(pwm_ppi_chg, (1 << (pwm_channel * 2)) | (1 << (pwm_channel * 2 + 1)));
            
            ppi_disable_channels(1 << (PWM_MAX_CHANNELS * 2));
            ppi_configure_channel(PWM_MAX_CHANNELS * 2, &PWM_TIMER->EVENTS_COMPARE[pwm_channel], &NRF_PPI->TASKS_CHG[pwm_ppi_chg].EN);
            ppi_enable_channels(1 << (PWM_MAX_CHANNELS * 2));
        }
        
        // Disable PPI channels next cycle
        pwm_freq_int_set();
        
        return;
    }
    
    if (pwm_value == 0)
    {
        // Corner case: 0% duty cycle
        NRF_GPIO->OUTCLR = (1 << pwm_io_ch[pwm_channel]);

        ppi_configure_channel_group(pwm_ppi_chg, 0);
        ppi_enable_channel_group(pwm_ppi_chg);
        ppi_configure_channel_group(pwm_ppi_chg, (1 << (pwm_channel * 2)) | (1 << (pwm_channel * 2 + 1)));
        ppi_configure_channel(PWM_MAX_CHANNELS * 2, &PWM_TIMER->EVENTS_COMPARE[pwm_channel], &NRF_PPI->TASKS_CHG[pwm_ppi_chg].DIS);
        ppi_enable_channels(1 << (PWM_MAX_CHANNELS * 2));
        
        // Wait for one PWM clock cycle
        nrf_delay_us(pwm_period_us);
        
        PWM_TIMER->CC[pwm_channel] = 0;
        
        ppi_disable_channels(1 << (PWM_MAX_CHANNELS * 2));
        ppi_configure_channel_group(pwm_ppi_chg, 0);
        
        nrf_gpiote_unconfig(pwm_gpiote_channel[pwm_channel]);
    }
    else if (pwm_value >= pwm_max_value)
    {
        // Corner case: 100% duty cycle
        NRF_GPIO->OUTSET = (1 << pwm_io_ch[pwm_channel]);

        ppi_disable_channels(1 << (PWM_MAX_CHANNELS * 2));
        ppi_configure_channel_group(pwm_ppi_chg, 0);
        ppi_enable_channel_group(pwm_ppi_chg);
        ppi_configure_channel_group(pwm_ppi_chg, (1 << (pwm_channel * 2)) | (1 << (pwm_channel * 2 + 1)));
        ppi_configure_channel(PWM_MAX_CHANNELS * 2, &PWM_TIMER->EVENTS_COMPARE[3], &NRF_PPI->TASKS_CHG[pwm_ppi_chg].DIS);
        ppi_enable_channels(1 << (PWM_MAX_CHANNELS * 2));
        
        // Wait for one PWM clock cycle
        nrf_delay_us(pwm_period_us);
        
        ppi_disable_channels(1 << (PWM_MAX_CHANNELS * 2));
        ppi_configure_channel_group(pwm_ppi_chg, 0);
        
        PWM_TIMER->CC[pwm_channel] = 0;
        
        // 100% duty cycle
        nrf_gpiote_unconfig(pwm_gpiote_channel[pwm_channel]);
        return;
    }
    else if ((pwm_value * 2) > PWM_TIMER->CC[pwm_channel])
    {
        // New duty cycle is larger than the current one
        ppi_disable_channels(1 << (PWM_MAX_CHANNELS * 2));
        PWM_TIMER->CC[2] = pwm_value * 2;
        
        ppi_configure_channel(PWM_MAX_CHANNELS * 2, &PWM_TIMER->EVENTS_COMPARE[2], &PWM_TIMER->TASKS_CAPTURE[pwm_channel]);
        ppi_enable_channels(1 << (PWM_MAX_CHANNELS * 2));
     
        // Disable PPI channels next cycle
        pwm_freq_int_set();
    }
    else
    {
        // New duty cycle is smaller than the current one
        ppi_disable_channels((1 << (PWM_MAX_CHANNELS * 2)) | (1 << (PWM_MAX_CHANNELS * 2 + 1)) | (1 << (PWM_MAX_CHANNELS * 2 + 2)));
        ppi_configure_channel_group(pwm_ppi_chg, 0);
        ppi_disable_channel_group(pwm_ppi_chg);
        ppi_configure_channel_group(pwm_ppi_chg, (1 << (PWM_MAX_CHANNELS * 2)) | (1 << (PWM_MAX_CHANNELS * 2 + 1)));
        
        ppi_configure_channel(PWM_MAX_CHANNELS * 2,     &PWM_TIMER->EVENTS_COMPARE[2], &NRF_GPIOTE->TASKS_OUT[pwm_gpiote_channel[pwm_channel]]);
        ppi_configure_channel(PWM_MAX_CHANNELS * 2 + 1, &PWM_TIMER->EVENTS_COMPARE[2], &PWM_TIMER->TASKS_CAPTURE[pwm_channel]);
        ppi_configure_channel(PWM_MAX_CHANNELS * 2 + 2, &PWM_TIMER->EVENTS_COMPARE[3], &NRF_PPI->TASKS_CHG[pwm_ppi_chg].EN);
        
        PWM_TIMER->CC[2] = pwm_value * 2;
        ppi_enable_channels(1 << (PWM_MAX_CHANNELS * 2 + 2));
        
        // Disable PPI channels next cycle
        pwm_freq_int_set();
    }
}
