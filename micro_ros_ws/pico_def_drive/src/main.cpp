#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <math.h>
#include <stdio.h>

#include "hardware/pwm.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"

#include "ICM20600.h"

extern "C"{
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#include <rmw_microros/time_sync.h>
#include <nav_msgs/msg/odometry.h>
#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include <geometry_msgs/msg/twist.h>
}
//Left Motor
#define LEFT_CW		15
#define LEFT_CCW	14
#define LEFT_PWM 	13
//Right Motor
#define RIGHT_CW	12
#define RIGHT_CCW	11
#define RIGHT_PWM 	10




const uint LED_PIN = 25;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

SemaphoreHandle_t mutex;

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist Twist_msg;

double Linear_speed_request;
double  Angular_speed_request;


void IMU_init(ICM20600 &icm20600){
     ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    i2c_init(I2C_CHAN, I2C_BAUD_RATE) ;
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C) ;
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C) ;
    gpio_pull_up(SDA_PIN) ;
    gpio_pull_up(SCL_PIN) ;
    
    icm20600.initialize();
    icm20600.setPowerMode(ICM_6AXIS_LOW_NOISE);


}

void IMU_val(void *params){
    ICM20600 icm20600(true);
    float Linear_velocity;
    while (1)
    {
      int gyro_Z  = icm20600.getGyroscopeZ();
      int acceleration_X = icm20600.getAccelerationX();
      float acceleration_sum = 0;
      int Average = 100;
      
      int Previous_time = time_us_32();

      for (size_t i = 0; i < Average; i++)
      {
        acceleration_sum = acceleration_sum + icm20600.getAccelerationX();
      }
      int Second_time = time_us_32();
      Linear_velocity =(Second_time - Previous_time) * (acceleration_sum / 100);
      
     // printf("velocity_x : %f" ,Linear_velocity);
      
    }
    

}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    msg.data++;
}

void subscription_callback(const void * msgin)
{

  // Cast received message to used type
  const geometry_msgs__msg__Twist * pTwistMsg = (const geometry_msgs__msg__Twist *) msgin;

    Linear_speed_request =  pTwistMsg->linear.x; 
    Angular_speed_request =  pTwistMsg->angular.z;
    double Right_throttle;
    double Left_throttle;


	if (Linear_speed_request < -1.0 ) { Linear_speed_request = -1.0; }
    if (Linear_speed_request > 1.0 ) { Linear_speed_request = 1.0; }

    if (Angular_speed_request < -1.0 ) { Angular_speed_request = -1.0; }
    if (Angular_speed_request > 1.0 ) { Angular_speed_request = 1.0; }

     if (Angular_speed_request == 0.0 )
    {
       Right_throttle = Linear_speed_request;
       Left_throttle = Linear_speed_request ;
    }
   
    if (Angular_speed_request < 0.0 ) 
    {
        Right_throttle = ((Linear_speed_request) - abs(Angular_speed_request));
        Left_throttle = ((Linear_speed_request) + abs(Angular_speed_request));
    }

    else  
    {
        Right_throttle = ((Linear_speed_request) + abs(Angular_speed_request));
        Left_throttle = ((Linear_speed_request) - abs(Angular_speed_request));
    }

   


//Left Motor
    int left_pwm = (int)((float)(0xffff) * abs(Left_throttle));
    pwm_set_gpio_level(LEFT_PWM, left_pwm);
	if (Left_throttle < 0 ){
		gpio_put(LEFT_CW, 1);
		gpio_put(LEFT_CCW, 0);
	}
    else// run counter clockwise
    {		
		gpio_put(LEFT_CW, 0);
		gpio_put(LEFT_CCW, 1);
    }
//Right Motor
    int right_pwm = (int)((float)(0xffff) * abs(Right_throttle));
    pwm_set_gpio_level(RIGHT_PWM, right_pwm);
	if (Right_throttle < 0 ){
		gpio_put(RIGHT_CW, 1);
		gpio_put(RIGHT_CCW, 0);
	}
    else // run counter clockwise
    {		
		gpio_put(RIGHT_CW, 0);
		gpio_put(RIGHT_CCW, 1);
    }


}

void Motor_init(){

    //Left Motor
    gpio_init(LEFT_PWM);
    gpio_set_function(LEFT_PWM, GPIO_FUNC_PWM);
    pwm_set_gpio_level(LEFT_PWM, 0);
    uint slice_num_LEFT = pwm_gpio_to_slice_num(LEFT_PWM);
    pwm_set_enabled(slice_num_LEFT, true);

    gpio_init(LEFT_CW);
    gpio_set_dir(LEFT_CW, GPIO_OUT);
    gpio_init(LEFT_CCW);
    gpio_set_dir(LEFT_CCW, GPIO_OUT);

    //Right Motor
    gpio_init(RIGHT_PWM);
    gpio_set_function(RIGHT_PWM, GPIO_FUNC_PWM);
    pwm_set_gpio_level(RIGHT_PWM, 0);
    uint slice_num_RIGHT = pwm_gpio_to_slice_num(RIGHT_PWM);
    pwm_set_enabled(slice_num_RIGHT, true);

    gpio_init(RIGHT_CW);
    gpio_set_dir(RIGHT_CW, GPIO_OUT);
    gpio_init(RIGHT_CCW);
    gpio_set_dir(RIGHT_CCW, GPIO_OUT);

}


int uros_com()
{
   

//   geometry_msgs__msg__Twist__init(&Twist_msg);
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pico_publisher");

        rcl_ret_t rc = rclc_subscription_init_default(
             &subscriber,
             &node,
             ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
             "/cmd_vel");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback);

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_timer(&executor, &timer);
    //* Add subscription to the executor
    rc = rclc_executor_add_subscription(
      &executor, &subscriber, &Twist_msg,
      &subscription_callback, ON_NEW_DATA);

    gpio_put(LED_PIN, 1);
    msg.data = 0;

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }; return 0;
}

void Task_1(void *params)
{
    ICM20600 icm20600(true);
    IMU_init(icm20600);

    Motor_init();
    uros_com();

    vTaskDelay(500);
   while (1)
   {
    /* code */
   } 
}

int main(){
    stdio_init_all();

    mutex = xSemaphoreCreateMutex();

    xTaskCreate(Task_1, "main_task", 256, NULL, 1, NULL);
    xTaskCreate(IMU_val, "main_task", 256, NULL, 1, NULL);
    vTaskStartScheduler(); 
    return 0;

}