#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>
#include <vector>

#include "KF.hpp"
#include <Eigen/Dense>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
using namespace std::chrono_literals;

class antonioOdomPublisher : public rclcpp::Node
{
  public:
    antonioOdomPublisher()
    : Node("antonio_odom_Publisher"), count_(0)
    {
      publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/antonio_odom", 10);

      timer_ = this->create_wall_timer(
      10ms, std::bind(&antonioOdomPublisher::odom_publisher, this));
      
      sub_Wheels_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/Wheels/odom", 10,
      std::bind(&antonioOdomPublisher::handle_Wheels_odom, this, std::placeholders::_1));

      sub_Left_IMU_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/Left_imu/imu/data", 10,
      std::bind(&antonioOdomPublisher::handle_IMU_Left, this, std::placeholders::_1));

      sub_right_IMU_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/right_imu/filter/free_acceleration", 10,
      std::bind(&antonioOdomPublisher::handle_IMU_right_acc, this, std::placeholders::_1));

      sub_right_IMU_Quaternion_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
      "/right_imu/filter/quaternion", 10,
      std::bind(&antonioOdomPublisher::handle_right_IMU_quaternion, this, std::placeholders::_1));

      sub_left_IMU_Quaternion_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
      "/Left_imu/filter/quaternion", 10,
      std::bind(&antonioOdomPublisher::handle_left_IMU_quaternion, this, std::placeholders::_1));
    }

  private:
    nav_msgs::msg::Odometry odom;
    double wheels_pose_x, wheels_pose_y;
    double wheels_yaw_val, wheels_x_val ;
    double IMU_pitch, IMU_roll, IMU_yaw, wheels_yaw; 
    double IMU_acc_x, IMU_acc_y, IMU_acc_z, IMU_yaw_val ;  
    u_int32_t time_stamp_nanosec;
    double time_stamp_sec;
    double delta_time = 0.01;
    u_int32_t priv_time;
    double x_val =0;
    double roll = 0, pitch = 0, yaw = 0;
    double yaw_val = 0;
    double acc_x = 0;    
    int G = 9.81;

    // left_imu
    double L_imu_orientation_x;
    double L_imu_orientation_y;
    double L_imu_orientation_z;
    double L_imu_orientation_w;

    double L_imu_acceleration_x;

    double L_roll = 0, L_pitch = 0, L_yaw = 0;
    
    // rught_imu
    double R_imu_orientation_x;
    double R_imu_orientation_y;
    double R_imu_orientation_z;
    double R_imu_orientation_w;

    double R_imu_acceleration_x;
    
    double R_roll = 0, R_pitch = 0, R_yaw = 0;

    // kalman filter parameters
    int n = 3; // Number of states
    int m = 3; // Number of measurements


    void handle_right_IMU_quaternion(const std::shared_ptr<geometry_msgs::msg::QuaternionStamped> msg)
    {
      R_imu_orientation_x = msg->quaternion.x;
      R_imu_orientation_y = msg->quaternion.y;
      R_imu_orientation_z = msg->quaternion.z;
      R_imu_orientation_w = msg->quaternion.w;
    }

    void handle_left_IMU_quaternion(const std::shared_ptr<geometry_msgs::msg::QuaternionStamped> msg)
    {
      L_imu_orientation_x = msg->quaternion.x;
      L_imu_orientation_y = msg->quaternion.y;
      L_imu_orientation_z = msg->quaternion.z;
      L_imu_orientation_w = msg->quaternion.w;
    }

    void init_kalman()
    {
      Eigen::MatrixXd A(n, n); // System dynamics matrix
      Eigen::MatrixXd C(m, n); // Output matrix
      Eigen::MatrixXd Q(n, n); // Process noise covariance
      Eigen::MatrixXd R(m, m); // Measurement noise covariance
      Eigen::MatrixXd P(n, n); // Estimate error covariance
      
      // Discrete LTI projectile motion, measuring position only
      A << 1, delta_time, 0, 0, 1, delta_time, 0, 0, 1;
      C << 1, 0, 0;
      
      // Reasonable covariance matrices
      Q << .05, .05, .0, .05, .05, .0, .0, .0, .0;
      R << 0, 0, 5;
      P << .1, .1, .1, .1, 10000, 10, .1, 10, 100;

      KalmanFilter kf(delta_time,A, C, Q, R, P);
 
      kf.init();

       // Feed measurements into filter, output estimated states

      Eigen::VectorXd y(m);
    }

    void odom_publisher()
    {
      

           //*************************************************************YAW/Orientation
      Eigen::Vector4f L_q_v(
        L_imu_orientation_x,
        L_imu_orientation_y,
        L_imu_orientation_z,
        L_imu_orientation_w);

        Eigen::Vector4f R_q_v(
        R_imu_orientation_x,
        R_imu_orientation_y,
        R_imu_orientation_z,
        R_imu_orientation_w);

        std::vector<Eigen::Vector4f> Average_LR({L_q_v, R_q_v});
        Eigen::Vector4f ave_quat = quaternionAverage(Average_LR);

        tf2::Quaternion ave_q(
            ave_quat[0],
            ave_quat[1],
            ave_quat[2],
            ave_quat[3]);
      tf2::Matrix3x3 ave_m(ave_q);
      ave_m.getRPY(roll, pitch, yaw);
      if (yaw < 0 ){yaw += 2*M_PI;}

      odom.pose.pose.orientation.x = ave_quat[0];
      odom.pose.pose.orientation.y = ave_quat[1];
      odom.pose.pose.orientation.z = ave_quat[2];
      odom.pose.pose.orientation.w = ave_quat[3];

      // ********************************************************************velocity X
      x_val = x_val + R_imu_acceleration_x * delta_time;
      odom.twist.twist.linear.x = x_val;

      //if (abs(x_val) > abs(wheels_x_val + 0.1) || abs(x_val) < abs(wheels_x_val - 0.1)){x_val = wheels_x_val;}
      //if (x_val != wheels_x_val){RCLCPP_INFO(this->get_logger(),"yahooooooooooooooooooooo  ");}
      if (wheels_x_val == 0){x_val = 0;}

      
      //***********************************************************************pose
      odom.pose.pose.position.x += cos(yaw) * x_val * delta_time;
      odom.pose.pose.position.y += sin(yaw) * x_val * delta_time;

      RCLCPP_INFO(this->get_logger(),"wheels_x_val: %f, pos.y: %f, pos.x: %f,x_val: %f   ",wheels_x_val, odom.pose.pose.position.y, odom.pose.pose.position.x, x_val);
     
      publisher_->publish(odom);
    }
   
    Eigen::Vector4f quaternionAverage(std::vector<Eigen::Vector4f> quaternions)
      {
      	if (quaternions.size() == 0)
      	{
      		std::cerr << "Error trying to calculate the average quaternion of an empty set!\n";
      		return Eigen::Vector4f::Zero();
      	}

      	// first build a 4x4 matrix which is the elementwise sum of the product of each quaternion with itself
      	Eigen::Matrix4f A = Eigen::Matrix4f::Zero();

      	for (int q=0; q<quaternions.size(); ++q)
      		A += quaternions[q] * quaternions[q].transpose();

      	// normalise with the number of quaternions
      	A /= quaternions.size();

      	// Compute the SVD of this 4x4 matrix
      	Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

      	Eigen::VectorXf singularValues = svd.singularValues();
      	Eigen::MatrixXf U = svd.matrixU();

      	// find the eigen vector corresponding to the largest eigen value
      	int largestEigenValueIndex;
      	float largestEigenValue;
      	bool first = true;

      	for (int i=0; i<singularValues.rows(); ++i)
      	{
      		if (first)
      		{
      			largestEigenValue = singularValues(i);
      			largestEigenValueIndex = i;
      			first = false;
      		}
      		else if (singularValues(i) > largestEigenValue)
      		{
      			largestEigenValue = singularValues(i);
      			largestEigenValueIndex = i;
      		}
      	}

      	Eigen::Vector4f average;
      	average(0) = U(0, largestEigenValueIndex);
      	average(1) = U(1, largestEigenValueIndex);
      	average(2) = U(2, largestEigenValueIndex);
      	average(3) = U(3, largestEigenValueIndex);

      	return average;
      }
   
    void handle_Wheels_odom(const std::shared_ptr<nav_msgs::msg::Odometry> msg)
    {
     wheels_yaw_val = msg->twist.twist.angular.z;
     wheels_x_val = msg->twist.twist.linear.x;
     wheels_pose_x = msg->pose.pose.position.x;
     wheels_pose_y = msg->pose.pose.position.y;
    }
    
    void handle_IMU_right_acc(const std::shared_ptr<geometry_msgs::msg::Vector3Stamped> msg)
    {
      R_imu_acceleration_x = msg->vector.x;  // unit	m/s2
    }
    void handle_IMU_Left(const std::shared_ptr<sensor_msgs::msg::Imu> msg)
    {
      time_stamp_nanosec = msg->header.stamp.nanosec;

      IMU_yaw_val = msg->angular_velocity.z;
      odom.twist.twist.angular.z = IMU_yaw_val;

      IMU_acc_x = msg->linear_acceleration.x;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_Wheels_odom_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_Left_IMU_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_right_IMU_;

    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_right_IMU_Quaternion_;
    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_left_IMU_Quaternion_;

    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<antonioOdomPublisher>());
  rclcpp::shutdown();
  return 0;
}