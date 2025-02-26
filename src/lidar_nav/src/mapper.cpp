#include <memory>
#include <iostream>
#include <string>
#include <stdio.h>
#include <iterator>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>

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
  Mapper()
  : Node("mapper")
  {

    pointsOutput.open("points.csv");
    msgOutput.open("message.csv");
    pointsOutput << "x,y,z" << std::endl;
    msgOutput << "width,height,is_dense" << std::endl;

    auto topic_callback =
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void {
        // RCLCPP_INFO(this->get_logger(), std::to_string(msg->data.));
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::fromROSMsg(*msg, *cloud); // Convert ROS PointCloud2 to PCL
        msgOutput << cloud->width << "," << cloud->height << "," << cloud->is_dense << std::endl;

         for (const auto& point : cloud->points) {
          pointsOutput << point.x << "," << point.y << "," << point.z << std::endl;
          std::cout << point << std::endl;
         }
      };
    subscription_ =
      this->create_subscription<sensor_msgs::msg::PointCloud2>("unilidar/cloud", 10, topic_callback);

  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
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
