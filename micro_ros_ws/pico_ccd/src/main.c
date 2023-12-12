
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"

#define WRAPVAL 65502
#define CLKDIV 14.5f



void led_task()
{   
    gpio_set_function(0, GPIO_FUNC_PWM);
    uint slice_num_0 = pwm_gpio_to_slice_num(0);
    pwm_set_enabled(slice_num_0, true);
    pwm_set_wrap(slice_num_0, 62);
    pwm_set_chan_level(slice_num_0, 0, 31);

    gpio_set_function(5, GPIO_FUNC_PWM);
    uint slice_num_5 = pwm_gpio_to_slice_num(5);

    //pwm_clear_irq(slice_num_5);
    //pwm_set_irq_enabled(slice_num_5, true);
    //irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    //irq_set_enabled(PWM_IRQ_WRAP, true);

    pwm_set_wrap(slice_num_5, WRAPVAL);
    pwm_set_clkdiv(slice_num_5, CLKDIV);
    pwm_set_chan_level(slice_num_5, PWM_CHAN_B, 86);
    pwm_set_mask_enabled((1u << slice_num_5));
    //pwm_set_mask_enabled((1u << slice_num_0));

     
 
}

void on_pwm_wrap() 
{

}

int main()
{
    stdio_init_all();

    xTaskCreate(led_task, "LED_Task", 256, NULL, 1, NULL);
    vTaskStartScheduler();

    while(1){};
}