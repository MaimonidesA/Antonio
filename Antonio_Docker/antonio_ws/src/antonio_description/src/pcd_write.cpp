#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pcl_conversions/pcl_conversions.h"



#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>
#include <vector>

//#include "KF.hpp"
#include <Eigen/Dense>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
 #include <rclcpp/qos.hpp>

using namespace std::chrono_literals;

class antonioPcdWrie : public rclcpp::Node
{
  public:
      
    antonioPcdWrie()
    : Node("antonio_Pcd_Wrie"), count_(0)
    {
      publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/Pcd_Wrie", 10);

      timer_ = this->create_wall_timer(
      5000ms, std::bind(&antonioPcdWrie::Pcd_Wrie, this));
      
      sub_antonio_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/antonio_odom", 10,
      std::bind(&antonioPcdWrie::handle_antonio_odom, this, std::placeholders::_1));

     sub_cloud_in_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
     "/cloud", rclcpp::SensorDataQoS(),
     std::bind(&antonioPcdWrie::handle_cloud_in, this, std::placeholders::_1));
     
    }

  private:
    nav_msgs::msg::Odometry antonio_odom;

    sensor_msgs::msg::PointCloud2 cloud_in;

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud_now;

    pcl::PointCloud<pcl::PointXYZ>* pcl_cloudPtr = &pcl_cloud;

    sensor_msgs::msg::PointCloud2* cloud_inPtr = &cloud_in;
    
    
   void handle_cloud_in(const std::shared_ptr<sensor_msgs::msg::PointCloud2> msg)
   {       
       if (cloud_inPtr == nullptr){RCLCPP_INFO(this->get_logger(),"NOT_ENOUGH_DATA_TO_GET_RESULTS;");}
           
       pcl::fromROSMsg(*msg, pcl_cloud_now);
       pcl_cloud += pcl_cloud_now;
   }
//
   void handle_antonio_odom(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
   {

   }
//
  void Pcd_Wrie()
  { 
    if (pcl_cloudPtr != nullptr)
    {
      pcl::io::savePCDFileASCII ("test_pcd.pcd", pcl_cloud);
      RCLCPP_INFO(this->get_logger()," data points to test_pcd.pcd.");
    }
  }
      

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_antonio_odom_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_in_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<antonioPcdWrie>());
  rclcpp::shutdown();
  return 0;
}