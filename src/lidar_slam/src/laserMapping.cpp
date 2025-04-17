#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <Eigen/Core>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "IMU_Processing.hpp"
#include "parameters.h"
#include "Estimator.h"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#define MAXN (720000)
#define PUBFRAME_PERIOD (20)

const float MOV_THRESHOLD = 1.5f;

std::mutex mtx_buffer;
std::condition_variable sig_buffer;

std::string root_dir = ROOT_DIR;

int feats_down_size = 0;
int time_log_counter = 0;
int scan_count = 0;
int publish_count = 0;

int frame_ct = 0;
double time_update_last = 0.0;
double time_current = 0.0;
double time_predict_last_const = 0.0;
double t_last = 0.0;

std::shared_ptr<ImuProcess> p_imu(new ImuProcess());
bool init_map = false;
bool flg_first_scan = true;
PointCloudXYZI::Ptr ptr_con(new PointCloudXYZI());

double T1[MAXN];
double s_plot[MAXN];
double s_plot2[MAXN];
double s_plot3[MAXN];
double s_plot11[MAXN];

double match_time = 0;
double solve_time = 0;
double propag_time = 0;
double update_time = 0;

bool lidar_pushed = false;
bool flg_exit = false;
bool flg_reset = false;

std::vector<BoxPointType> cub_needrm;

std::deque<PointCloudXYZI::Ptr> lidar_buffer;
std::deque<double> time_buffer;
std::deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_deque;

PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body_space(new PointCloudXYZI());
PointCloudXYZI::Ptr init_feats_world(new PointCloudXYZI());

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

V3D euler_cur;

MeasureGroup Measures;

sensor_msgs::msg::Imu imu_last, imu_next;
sensor_msgs::msg::Imu::ConstSharedPtr imu_last_ptr;
nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::PoseStamped msg_body_pose;

void SigHandle(int sig)
{
    flg_exit = true;
    RCLCPP_INFO(rclcpp::get_logger("laserMapping"), "Caught signal %d", sig);
    sig_buffer.notify_all();
}

class LaserMappingNode : public rclcpp::Node
{
public:
    LaserMappingNode() : Node("laser_mapping")
    {
        // Initialize publishers
        pubLaserCloudFullRes_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pointlio/cloud_registered", 100);
        pubLaserCloudFullResBody_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pointlio/cloud_registered_body", 100);
        pubOdomAftMapped_ = this->create_publisher<nav_msgs::msg::Odometry>("/pointlio/odom", 100);
        pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/pointlio/path", 100);

        // Initialize subscribers
        subPcl_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lidar_topic", 10, std::bind(&LaserMappingNode::standardPclCallback, this, std::placeholders::_1));
        subImu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_topic", 10, std::bind(&LaserMappingNode::imuCallback, this, std::placeholders::_1));
    }

private:
    void standardPclCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mtx_buffer);
        scan_count++;
        // Process the point cloud message
        sig_buffer.notify_all();
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mtx_buffer);
        imu_deque.emplace_back(msg);
        sig_buffer.notify_all();
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullResBody_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subPcl_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaserMappingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
