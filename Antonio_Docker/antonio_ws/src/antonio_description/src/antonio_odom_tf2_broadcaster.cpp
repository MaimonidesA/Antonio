
#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

class FramePublisher : public rclcpp::Node
{
public:
  FramePublisher()
  : Node("odom_tf2_frame_publisher")
  {
    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    sub_Wheels_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/antonio_odom", 10,
      std::bind(&FramePublisher::handle_Wheels_odom, this, std::placeholders::_1));

     sub_Z_pose_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/ccd_Z", 10,
      std::bind(&FramePublisher::handle_Z_pose, this, std::placeholders::_1));

  }

private:
  geometry_msgs::msg::TransformStamped t;

  void handle_Z_pose(const std::shared_ptr<geometry_msgs::msg::Pose> msg)
  {
    if (msg->position.z != INFINITY){
    t.transform.translation.z = (msg->position.z) ;
  }else{RCLCPP_INFO(this->get_logger(),"***************** No CCD detect ************");}
  }
  
  void handle_Wheels_odom(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
  {

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";

    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
   
    t.transform.rotation.x = msg->pose.pose.orientation.x;
    t.transform.rotation.y = msg->pose.pose.orientation.y;
    t.transform.rotation.z = msg->pose.pose.orientation.z;
    t.transform.rotation.w = msg->pose.pose.orientation.w;
    
    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_Z_pose_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_Wheels_odom_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_IMU_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
 // std::string turtlename_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}
