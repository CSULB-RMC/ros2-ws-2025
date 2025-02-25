#include <memory>
#include "iostream"
#include <string>
#include <stdio.h>
#include <iterator>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

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
    auto topic_callback =
        [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) -> void
    {
      // RCLCPP_INFO(this->get_logger(), std::to_string(msg->data.));
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

      pcl::fromROSMsg(*msg, *cloud); // Convert ROS PointCloud2 to PCL

      // std::cout << cloud->points[0].x << std::endl;
      for (const auto &point : cloud->points)
      {
        std::cout << point << std::endl;
      }

      // const auto& point = *cloud;
      // std::cout << "x: " << point.x << ", y: " << point.y << ", z: " << point.z << std::endl;
    };
    subscription_ =
        this->create_subscription<sensor_msgs::msg::PointCloud2>("unilidar/cloud", 10, topic_callback);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  // int rows = 200;
  // int cols = 200;
  // std::vector<std::vector<int>> grid(rows, std::vector<int>(cols, 0))
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mapper>());
  rclcpp::shutdown();
  return 0;
}
