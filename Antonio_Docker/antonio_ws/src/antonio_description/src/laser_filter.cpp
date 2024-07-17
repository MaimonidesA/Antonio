#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>
#include <vector>

//#include "KF.hpp"

#include <sensor_msgs/msg/laser_scan.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"




//using namespace std::chrono_literals;

class antonioScanFilter : public rclcpp::Node
{
  public:
    antonioScanFilter()
    : Node("antonio_Scan_Filter"), count_(0)
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_filtered", 10);

      liser_in_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan_F", 10,
      std::bind(&antonioScanFilter::filter_scan_by_angle, this, std::placeholders::_1));
    }

  private:
    sensor_msgs::msg::LaserScan scan_F_filtered;
    float Alpha_min = 0; //1.570796327 = 90 deg
    float Alpha_max = M_PI ;//0.19344649044796824
    float liser_in_baf;
    float Alpha = 0;
    void filter_scan_by_angle(const std::shared_ptr<sensor_msgs::msg::LaserScan> msg)
    {
       scan_F_filtered.ranges.resize(msg->ranges.size());
       scan_F_filtered.intensities.resize(msg->intensities.size());
        
       Alpha = 1.570796327;
       for (int i = 0; i <  msg->ranges.size() ; i++)
       {
         
         if (Alpha > Alpha_min && Alpha < Alpha_max)
         {
             scan_F_filtered.ranges[i] = msg->ranges[i];
             scan_F_filtered.intensities[i] = msg->intensities[i];
         }
         else {scan_F_filtered.ranges[i] = INFINITY;}

         Alpha += msg->angle_increment;
         if (Alpha >= M_PI){Alpha -= M_PI*2;}

         //RCLCPP_INFO(this->get_logger(),"msg->ranges[i]: %f,",msg->ranges[i]);
        }
       
        scan_F_filtered.header.frame_id = msg->header.frame_id;
        scan_F_filtered.angle_min = Alpha_min;
        scan_F_filtered.angle_max = Alpha_max;
        scan_F_filtered.angle_increment = msg->angle_increment;
        scan_F_filtered.time_increment = msg->time_increment;
        scan_F_filtered.header.stamp = this->now();
        scan_F_filtered.scan_time = msg->scan_time;
        scan_F_filtered.range_min = msg->range_min;
        scan_F_filtered.range_max = msg->range_max;
        publisher_->publish(scan_F_filtered);
      
    }
   

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr liser_in_;

    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<antonioScanFilter>());
  rclcpp::shutdown();
  return 0;
}