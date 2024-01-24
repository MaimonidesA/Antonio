#include <stdio.h>
#include "hardware/pwm.h"
#include <math.h>
#include "hardware/i2c.h"
#include "ICM20600.h"
#include <time.h>
#include "SimpleKalmanFilter.h"
#include "AK09918.h"


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
// === the fixed point macros (16.15) ========================================
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)

#define BAT 21
// Arrays in which raw measurements will be stored
clock_t clock()
{
    return (clock_t) time_us_32() / 1000000;
}

AK09918_err_type_t err;

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

double startTime;
double endTime ;
double Delta_Time;

double priv_gyro_Pitch;
double Linear_velocity = 0;
double priv_Linear_velocity = 0;
double Distance = 0;

double Acc_X;
double prev_Acc_X;
double Sum_Acc_X;
double chige_betwin_megur_A;
double clibrate_Acc_X;
double kalman_Acc_X;
double X_position;
int cont_Sum_Acc_X;
double G ;
SimpleKalmanFilter simpleKalmanFilter(10, 2, 0.01);

int main() {
    
    stdio_init_all();

    gpio_init(BAT);
    gpio_set_dir(BAT, GPIO_IN);

    startTime = clock();

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
    icm20600.setAccOutputDataRate(ACC_RATE_1K_BW_218);
    //icm20600.setAccAverageSample(ACC_AVERAGE_32);
   // icm20600.setSampleRateDivier(2);

    AK09918 ak09918;

    err = ak09918.isDataReady();
    while (err != AK09918_ERR_OK) {
        printf("Waiting Sensor\n");
        sleep_ms(100);
        err = ak09918.isDataReady();

    sleep_ms(4000);

    for (size_t i = 0; i < 4000; i++)
    {

    G = G + sqrt(sqrt( (double)icm20600.getAccelerationZ() * (double)icm20600.getAccelerationZ() + (double)icm20600.getAccelerationY() * (double)icm20600.getAccelerationY()) *
             sqrt( (double)icm20600.getAccelerationZ() * (double)icm20600.getAccelerationZ() + (double)icm20600.getAccelerationY() * (double)icm20600.getAccelerationY()) + (double)icm20600.getAccelerationX() * (double)icm20600.getAccelerationX());
             sleep_ms(1);
    }
    G = G / 4000;

    for (size_t i = 0; i < 4000; i++)
    {
         //Pitch
        Delta_Time = (time_us_32() - endTime) / 1000000;
        endTime = time_us_32();
        acceleration_Z = icm20600.getAccelerationZ();
        acceleration_X = icm20600.getAccelerationX();
        acceleration_Y = icm20600.getAccelerationY();

        Accelerometer_Pitch = atan(acceleration_X / sqrt( acceleration_Z * acceleration_Z + acceleration_Y * acceleration_Y));
        Pitch = Accelerometer_Pitch ;
        
        //roll
        Accelerometer_roll = atan(acceleration_Y / sqrt( acceleration_Z * acceleration_Z + acceleration_X * acceleration_X));
        roll = Accelerometer_roll ;
        
        Acc_X =  (icm20600.getAccelerationX() - sin(Pitch) * G) ;
        Sum_Acc_X = Sum_Acc_X + Acc_X;
        chige_betwin_megur_A = chige_betwin_megur_A + abs(Acc_X - prev_Acc_X);
        prev_Acc_X = Acc_X;
        printf("%f,\n ", Acc_X);//******************7*/
    }

    clibrate_Acc_X = Sum_Acc_X / 4000;
    chige_betwin_megur_A = chige_betwin_megur_A / 4000;
    
 
    while (1)
    {
        //Pitch
        Delta_Time = (time_us_32() - endTime) / 1000000;
        endTime = time_us_32();
        acceleration_Z = icm20600.getAccelerationZ();
        acceleration_X = icm20600.getAccelerationX();
        acceleration_Y = icm20600.getAccelerationY();

        Accelerometer_Pitch = atan(acceleration_X / sqrt( acceleration_Z * acceleration_Z + acceleration_Y * acceleration_Y));
 
        gyro_Pitch = gyro_Pitch - (icm20600.getGyroscopeY()+2) * Delta_Time * M_PI / 180 ;
    
        Pitch = (Pitch - (icm20600.getGyroscopeY()+2) * Delta_Time * M_PI / 180) * 0.999 + Accelerometer_Pitch * 0.001;
        
        //roll

        Accelerometer_roll = atan(acceleration_Y / sqrt( acceleration_Z * acceleration_Z + acceleration_X * acceleration_X));
       
        gyro_roll = gyro_roll + icm20600.getGyroscopeX() * Delta_Time * M_PI / 180;
        
        roll = (roll + icm20600.getGyroscopeX() * Delta_Time * M_PI / 180) * 0.995 + Accelerometer_roll * 0.005;
        

         
    }
   

}


