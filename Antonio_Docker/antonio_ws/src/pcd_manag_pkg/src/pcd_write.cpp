#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"


#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>
#include <vector>


#include <Eigen/Dense>

#include "std_msgs/msg/string.hpp"

#include "rclcpp/rclcpp.hpp"

#include "tf2/buffer_core.h"
#include "tf2_ros/buffer_client.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"

#include "tf2/time.h"
#include "tf2_ros/visibility_control.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include <tf2_ros/transform_listener.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
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
      tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
      auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(this->get_node_base_interface(),
                                                                       this->get_node_timers_interface());
      tf2_buffer_->setCreateTimerInterface(timer_interface);
      tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

      pcl_cloud_new = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      voxel_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

      publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/Pcd_Wrie", 10);

      timer_ = this->create_wall_timer(
      10000ms, std::bind(&antonioPcdWrie::Pcd_Wrie, this));
      
      sub_antonio_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/antonio_odom", 10,
      std::bind(&antonioPcdWrie::handle_antonio_odom, this, std::placeholders::_1));
//
     //sub_cloud_in_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
     //"/cloud", rclcpp::SensorDataQoS(),
     //std::bind(&antonioPcdWrie::handle_cloud_in, this, std::placeholders::_1));
    sub_cloud_in_.subscribe(this, "/cloud", rmw_qos_profile_sensor_data);

    tf_point_cloud_sub_ =
         std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(sub_cloud_in_, *tf2_buffer_, map_id, 5,
                                                                                this->get_node_logging_interface(),
                                                                                this->get_node_clock_interface(), 5s);
      tf_point_cloud_sub_->registerCallback(&antonioPcdWrie::handle_cloud_in, this);
     
    }

  protected:
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
    geometry_msgs::msg::TransformStamped transform;
    nav_msgs::msg::Odometry antonio_odom;
    sensor_msgs::msg::PointCloud2 cloud_in;
   
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud; 
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_new;
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_cloud;
    
    bool update_pcl_cloud = false;
    const std::string map_id = "map";
    int pcd_file_cont = 0;
    
   void handle_cloud_in(const std::shared_ptr<sensor_msgs::msg::PointCloud2> msg)
   {// RCLCPP_INFO(this->get_logger(),"enter to: 'void handle_cloud_in()' ");
    // pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_new (new pcl::PointCloud<pcl::PointXYZ>);
      transform = tf2_buffer_->lookupTransform(map_id, msg->header.frame_id,
                                               msg->header.stamp,  tf2::durationFromSec(1.5));
                                             
       sensor_msgs::msg::PointCloud2 cloud2_on_map;
       
       tf2::doTransform(*msg, cloud2_on_map, transform);
         
       pcl::fromROSMsg(cloud2_on_map, *pcl_cloud_new);
       //RCLCPP_INFO(this->get_logger(),"exit: 'void handle_cloud_in()' ");  
   }


  void handle_antonio_odom(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
  {

   // pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (msg->twist.twist.linear.x != 0 || msg->twist.twist.angular.z != 0)
    {//RCLCPP_INFO(this->get_logger(),"enter to: 'void handle_antonio_odom()' ");
      *pcl_cloud += *pcl_cloud_new;
      update_pcl_cloud = true;
    }
    //RCLCPP_INFO(this->get_logger(),"exit: 'void handle_antonio_odom()' ");
  }
//
  void Pcd_Wrie()
  {  
    if (update_pcl_cloud)
    {RCLCPP_INFO(this->get_logger(),"enter to: 'void Pcd_Wrie()' ");

      pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;

      //voxel_filter.setInputCloud(pcl_cloud);
      //voxel_filter.setLeafSize(0.001,0.001,0.001);
      //voxel_filter.filter(*voxel_cloud);
//
      update_pcl_cloud = false;
      std::string pcd_file = "test_pcd_" + std::to_string(pcd_file_cont) + "_.pcd";
      pcd_file_cont++;
      pcl::io::savePCDFileASCII (pcd_file, *pcl_cloud);
      RCLCPP_INFO(this->get_logger()," data points to %s." ,pcd_file.c_str());
      *pcl_cloud = *pcl_cloud_new;
    }
  }
      
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_antonio_odom_;
    //rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_in_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_cloud_in_;
    std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> tf_point_cloud_sub_;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<antonioPcdWrie>());
  
  rclcpp::shutdown();
  return 0;
}