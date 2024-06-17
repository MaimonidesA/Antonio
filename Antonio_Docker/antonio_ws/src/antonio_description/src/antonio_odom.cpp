#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class antonioOdomPublisher : public rclcpp::Node
{
  public:
    antonioOdomPublisher()
    : Node("antonio_odom_Publisher"), count_(0)
    {
      publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/antonio_odom", 10);
      
      sub_Wheels_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/Wheels/odom", 10,
      std::bind(&antonioOdomPublisher::handle_Wheels_odom, this, std::placeholders::_1));

      sub_IMU_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/IMU_msgs", 10,
      std::bind(&antonioOdomPublisher::handle_IMU, this, std::placeholders::_1));
    }

  private:
    nav_msgs::msg::Odometry odom;
    double wheels_pose_x, wheels_pose_y;
    double wheels_yaw_val, wheels_x_val ;
    double IMU_pitch, IMU_roll, IMU_yaw, wheels_yaw; 
    double IMU_acc_x, IMU_acc_y, IMU_acc_z, IMU_yaw_val ;  
    u_int32_t time_stamp_nanosec;
    double time_stamp_sec;
    double delta_time;
    u_int32_t priv_time;
    double x_val =0;
    double roll = 0, pitch = 0, yaw = 0;
    double yaw_val = 0;
    double acc_x = 0;    
    int G = 1003;
    
    void handle_Wheels_odom(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
    {
     wheels_yaw_val = msg->twist.twist.angular.z;
     wheels_x_val = msg->twist.twist.linear.x;
     wheels_pose_x = msg->pose.pose.position.x;
     wheels_pose_y = msg->pose.pose.position.y;
    }

    void handle_IMU(const std::shared_ptr<sensor_msgs::msg::Imu> msg)
    {
      
      time_stamp_nanosec = msg->header.stamp.nanosec;
 

      IMU_yaw_val = msg->angular_velocity.z;

      IMU_acc_x = msg->linear_acceleration.x;
      IMU_acc_y = msg->linear_acceleration.y;
      IMU_acc_z = msg->linear_acceleration.z;

      
      delta_time = (double(time_stamp_nanosec  - priv_time)) / 1000000000;
      priv_time = time_stamp_nanosec;

    
      odom.pose.pose.orientation.x = msg->orientation.x;
      odom.pose.pose.orientation.y = msg->orientation.y;
      odom.pose.pose.orientation.z = msg->orientation.z;
      odom.pose.pose.orientation.w = msg->orientation.w;
        
      //YAW
  tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf2::Matrix3x3 m(q);
  
    m.getRPY(roll, pitch, yaw);

    if (yaw < 0 ){yaw += 2*M_PI;}
    //if (yaw < 2*M_PI ){yaw += 2*M_PI;}
      
      RCLCPP_INFO(this->get_logger(),"yaw: %f, position_x: %f, position_y: %f,  ",yaw, odom.pose.pose.position.x, odom.pose.pose.position.y);  
      //acceleration X
      acc_x = IMU_acc_x - sin(IMU_pitch) * G;

      // velocity X
      x_val = x_val + acc_x * delta_time;
        
     // if (wheels_x_val < 0 && x_val > 0){x_val *= -1;}
    //  if (wheels_x_val > 0 && x_val < 0){x_val *= -1;}
      if (abs(x_val) > abs(wheels_x_val + 0.1) || abs(x_val) < abs(wheels_x_val - 0.1)){x_val = wheels_x_val;}
     // if (x_val != wheels_x_val){RCLCPP_INFO(this->get_logger(),"yahooooooooooooooooooooo  ");}
      //if (wheels_x_val == 0){x_val = 0;}
      //pose
      odom.pose.pose.position.x += cos(yaw) * x_val * delta_time;
      odom.pose.pose.position.y += sin(yaw) * x_val * delta_time;
      //odom.pose.pose.position.x = wheels_pose_x; 
      //odom.pose.pose.position.y = wheels_pose_y;
      
      publisher_->publish(odom);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_Wheels_odom_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_IMU_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<antonioOdomPublisher>());
  rclcpp::shutdown();
  return 0;
}