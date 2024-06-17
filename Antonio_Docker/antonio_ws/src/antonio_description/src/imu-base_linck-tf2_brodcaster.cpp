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

      sub_IMU_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/IMU/odom", 10,
      std::bind(&FramePublisher::handle_IMU, this, std::placeholders::_1));

      sub_Wheels_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/Wheels/odom", 10,
      std::bind(&FramePublisher::handle_Wheels_odom, this, std::placeholders::_1));

  }

private:
  geometry_msgs::msg::TransformStamped t;

  double IMU_pitch, IMU_roll, IMU_yaw, wheels_yaw;

  void handle_Wheels_odom(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
  {
   wheels_yaw = msg->pose.pose.orientation.z;
  }


  void handle_IMU(const std::shared_ptr<sensor_msgs::msg::Imu> msg)
  {
    IMU_pitch = (msg->orientation.y) * -1;
    IMU_roll = msg->orientation.x;
    IMU_yaw = msg->orientation.z;

     // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "IMU_odom";
    t.child_frame_id = "imu_pose";


    tf2::Quaternion q;
    q.setRPY(IMU_roll, IMU_pitch, IMU_yaw );
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    
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