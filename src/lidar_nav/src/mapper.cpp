#include <memory>
#include <iostream>
#include <string>
#include <stdio.h>
#include <iterator>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <cmath>
#include <math.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/header.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

class Mapper : public rclcpp::Node
{
public:
  struct MyEuler {
    float x;
    float y;
    float z;
  };

  MyEuler QuatToEuler(float x, float y, float z, float w) {
    float t0 = 2.0 * (w * x + y * z);
    float t1 = 1.0 - 2.0 * (x * x + y * y);
    float roll_x = atan2(t0, t1);

    float t2 = 2.0 * (w * y - z * x);
    if(t2 > 1){
      t2 = 1.0;
    } else if(t2 < -1){
      t2 = -1.0;
    }
    float pitch_y = asin(t2);

    float t3 = 2.0 * (w * z + x * y);
    float t4 = 1.0 -2.0 * (y * y + z * z);
    float yaw_z = atan2(t3, t4);

    roll_x = roll_x * 180 / M_PI;
    pitch_y = pitch_y * 180 / M_PI;
    yaw_z = yaw_z * 180 / M_PI;
    
    MyEuler results;
    results.x = roll_x;
    results.y = pitch_y;
    results.z = yaw_z;

    return results;
  }

  Mapper()
  : Node("mapper")
  {

    pointsOutput.open("points.csv");
    msgOutput.open("message.csv");
    pointsOutput << "x,y,z" << std::endl;
    msgOutput << "width,height,is_dense" << std::endl;




    auto cloud_callback =
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void {
        
        // RCLCPP_INFO(this->get_logger(), std::to_string(msg->data.));
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // pcl::fromROSMsg(*msg, *cloud); // Convert ROS PointCloud2 to PCL
        // msgOutput << cloud->width << "," << cloud->height << "," << cloud->is_dense << std::endl;

        //  for (const auto& point : cloud->points) {
          // pointsOutput << point.x << "," << point.y << "," << point.z << std::endl;
          // std::cout << point << std::endl;
        //  }
      };

    auto imu_callback =
      [this](sensor_msgs::msg::Imu::SharedPtr msg) -> void {
        MyEuler rotations = QuatToEuler(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        std::cout << rotations.x  << ", " << rotations.y << ", " << rotations.z << std::endl;
      };

    cloud_subscription_ =
      this->create_subscription<sensor_msgs::msg::PointCloud2>("unilidar/cloud", 10, cloud_callback);
    imu_subscription_ = 
      this->create_subscription<sensor_msgs::msg::Imu>("unilidar/imu", 10, imu_callback);

  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  std::ofstream pointsOutput;
  std::ofstream msgOutput;

  
  // int unit = 15; //centimeters
  // int starting_grid = 6 //meters
  // int rows = starting_grid / unit / 100;
  // int cols = 200;
  // int grid[rows][cols]

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mapper>());
  rclcpp::shutdown();
  return 0;
}
