
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"

#define WRAPVAL 65502
#define CLKDIV 14.5f

uint slice_num_0 = 0;
uint slice_num_5 = 2;
uint slice_num_6 = 3;


void on_pwm_wrap() //**************PWM interrupt.***************
{ 
    pwm_clear_irq(slice_num_5);

    pwm_set_counter(slice_num_6, 0);
    for (size_t i = 0; i < 750; i++){
        pwm_retard_count(slice_num_6);
    }  
}

void CCD_Timers()
{   //**************Main timer    2 megahertz ***************
    gpio_set_function(0, GPIO_FUNC_PWM);
    
    pwm_set_enabled(slice_num_0, true);
    pwm_set_wrap(slice_num_0, 62);
    pwm_set_chan_level(slice_num_0, 0, 31);
   
    //************  ICG timer 133 hertz***************
    gpio_set_function(5, GPIO_FUNC_PWM);
    
    pwm_clear_irq(slice_num_5);
    pwm_set_irq_enabled(slice_num_5, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    pwm_set_wrap(slice_num_5, WRAPVAL);
    pwm_set_clkdiv(slice_num_5, CLKDIV);
    pwm_set_chan_level(slice_num_5, PWM_CHAN_B, 86);

    pwm_set_output_polarity(slice_num_5, false, true);

    //************   SH timer *********************
    gpio_set_function(6, GPIO_FUNC_PWM);
    
    pwm_set_enabled(slice_num_6, true);
    pwm_set_wrap(slice_num_6, 1250);
    pwm_set_chan_level(slice_num_6, 0, 500);

    //*************Enable pwm  *****************
    pwm_set_mask_enabled((1u << slice_num_0) | (1u << slice_num_5) | (1u << slice_num_6));    
    
    printf("hello from ccd_Timers tasc ");
    while (1){
       uint32_t time = time_us_32();
       printf("hello from ccd_Timers tasc %f \n", time ); 
       vTaskDelay(1250);
    }
}





int main()
{
    stdio_init_all();

    xTaskCreate(CCD_Timers, "CCD_Timers", 256, NULL, 1, NULL);
    vTaskStartScheduler();

  
}