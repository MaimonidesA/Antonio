#include <stdio.h>
#include "hardware/pwm.h"
#include <math.h>
#include "hardware/i2c.h"
#include "ICM20600.h"
#include <time.h>

extern "C"{
#include "rcl/rcl.h"
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
//#include "pico_uart_transports.h"

#include <rmw_microros/time_sync.h>
#include <nav_msgs/msg/odometry.h>
#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include <geometry_msgs/msg/twist.h>
}


// Arrays in which raw measurements will be stored
clock_t clock()
{
    return (clock_t) time_us_64() / 10000;
}

ICM20600 icm20600(true);

int acceleration_X;
int acceleration_Y;
int acceleration_Z;

int gyro_X;
int gyro_Y;
int gyro_Z;

// orientation
double Pitch = 0, roll =0, yaw = 0;
// Accelerometer orientation.
double Accelerometer_Pitch, Accelerometer_roll;
// gyro orientation.
double gyro_Pitch, gyro_roll, gyro_yaw;

clock_t startTime;
clock_t endTime;
double Delta_Time;

double priv_gyro_Pitch;
double Linear_velocity = 0;
double Distance = 0;


int main() {
    stdio_init_all();

    ICM20600 icm20600(true);

     ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    i2c_init(I2C_CHAN, I2C_BAUD_RATE) ;
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C) ;
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C) ;
    gpio_pull_up(SDA_PIN) ;
    gpio_pull_up(SCL_PIN) ;

   /////////////////////////// IMU Set up /////////////////
    icm20600.initialize();
    icm20600.setPowerMode(ICM_6AXIS_LOW_NOISE);
    icm20600.setAccScaleRange(RANGE_2G);
 
    while (1)
    {
        Accelerometer_Pitch = atan((double)(icm20600.getAccelerationX()) / (double)(icm20600.getAccelerationZ()));
        
        //Pitch
        endTime = clock();
        Delta_Time = endTime - startTime;
        gyro_Pitch = Pitch + ((icm20600.getGyroscopeY()+2) * (Delta_Time * Delta_Time / 2)) * M_PI / 180 ;
        priv_gyro_Pitch = gyro_Pitch;
        startTime = endTime;

        Pitch = gyro_Pitch * 0.9 + Accelerometer_Pitch * 0.1;
        
        //roll
        Accelerometer_roll = atan((double)(icm20600.getAccelerationY()) / (double)(icm20600.getAccelerationZ()));
       
        endTime = clock();
        Delta_Time = endTime - startTime;
        gyro_roll = roll + (icm20600.getGyroscopeX() * (Delta_Time * Delta_Time / 2)) * M_PI / 180;
        startTime = endTime;

        roll = gyro_roll * 0.9 + Accelerometer_roll * 0.1;

        printf("%f, ", Pitch);//*/
        printf("%f, ", gyro_Pitch);//*/
        printf("%f, ", Accelerometer_Pitch);//*/
        printf("%f, ", roll);//*/
        printf("%f, ", gyro_roll);//*/
        printf("%f,\n", Accelerometer_roll);//*/
       // printf("pitch : %f gyro_Pitch :%f Acc_Pitch: %f roll : %f gyro_roll : %f Acc_roll : %f\n" , Pitch, gyro_Pitch, Accelerometer_Pitch, roll, gyro_roll);//*/

        /*double acceleration_sum = 0;
        double Average = 1000;
      
        clock_t startTime = clock();

        for (size_t i = 0; i < Average; i++)
        {
            acceleration_sum = acceleration_sum + (double)(icm20600.getAccelerationX());
        }
        clock_t endTime = clock();
        double executionTime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
        Linear_velocity = Linear_velocity + executionTime * ((acceleration_sum / Average)/100.0);

        Distance = Distance + Linear_velocity * executionTime;

        int acceleration_X = icm20600.getAccelerationX();
        int acceleration_Y = icm20600.getAccelerationY();
        int acceleration_Z = icm20600.getAccelerationZ();

        printf("Distance : %f velocity_x : %f time :%.8f  Average acc : %f acce   X : %i     Y : %i      Z : %i\n" 
              ,Distance ,Linear_velocity, executionTime, ((acceleration_sum / Average)/100.0) ,acceleration_X, acceleration_Y, acceleration_Z);

    /*
      int gyro_X  = icm20600.getGyroscopeX();
      int gyro_Y  = icm20600.getGyroscopeY();
      int gyro_Z  = icm20600.getGyroscopeZ();
      
      int acceleration_X = icm20600.getAccelerationX();
      int acceleration_Y = icm20600.getAccelerationY();
      int acceleration_Z = icm20600.getAccelerationZ();
      
      printf("acce   X : %i     Y : %i      Z : %i  gyro  X: %i     Y :%i    Z :%i \n" ,acceleration_X, acceleration_Y, acceleration_Z, gyro_X, gyro_Y, gyro_Z );
      
     */
    }
   

}

   //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<   Set up DMA channels   >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
   /* uint sample_channel = dma_claim_unused_channel(true);
    uint control_channel = dma_claim_unused_channel(true);
    dma_channel_config c2 = dma_channel_get_default_config(sample_channel);
    dma_channel_config c3 = dma_channel_get_default_config(control_channel);

    //----------------------------Channel 2.-------------------------
    channel_config_set_transfer_data_size(&c2, DMA_SIZE_8);
    channel_config_set_read_increment(&c2, false);
    channel_config_set_write_increment(&c2, true);

    channel_config_set_dreq(&c2, DREQ_I2C0_RX);

    dma_channel_configure(
        sample_channel,
        &c2,
        Pixel_array_buffer, 
        &i2c_hw->fifo,
        Number_of_pixels,
        false
        );//_____________________________________________

    //--------------------------Channel 3.------------------
    channel_config_set_transfer_data_size(&c3, DMA_SIZE_32);
    channel_config_set_read_increment(&c3, false);
    channel_config_set_write_increment(&c3, false);
    channel_config_set_chain_to(&c3, sample_channel);

    dma_channel_configure(
       control_channel,
       &c3,
       &dma_hw->ch[sample_channel].write_addr, 
       &First_pixel_buffer_pointer,
       1,
       true
       );//____________________________________________*/
