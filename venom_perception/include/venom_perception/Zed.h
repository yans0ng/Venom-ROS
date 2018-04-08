#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>


#ifndef VENOM_PERCEPTION_ZED_H
#define VENOM_PERCEPTION_ZED_H
namespace venom {
enum PerceptionType {
  ODOM = 0,
  DEPTH = 1,
  RGB = 2,
  CLOUD = 3,
};

class Zed {
public:
  Zed() {
    nh_ = ros::NodeHandle();
  }
  void Enable(PerceptionType type);
  void Disable(PerceptionType type);
  geometry_msgs::Pose GetPose();
  cv::Mat GetRGB();
  cv::Mat GetDepth();
  void SetROI(int x1, int x2, int y1, int y2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetCloud();
  std_msgs::Header GetHeader();

private:
  ros::Subscriber odom_sub_;  // odometry
  geometry_msgs::PoseWithCovarianceStamped pose_;
  void OdometryCallback(const nav_msgs::Odometry& msg);

  ros::Subscriber depth_sub_; // depth image
  cv_bridge::CvImagePtr depth_ptr_;
  void DepthCallback(const sensor_msgs::ImageConstPtr& msg);

  ros::Subscriber rgb_sub_;   // rgb image
  cv_bridge::CvImagePtr rgb_ptr_;
  void RGBCallback(const sensor_msgs::ImageConstPtr& msg);

  ros::Subscriber cloud_sub_;
  std_msgs::Header header_;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_; // point cloud
  std::pair<int,int> roi_x_{0,0};
  std::pair<int,int> roi_y_{0,0};
  void CloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

  bool verbose_ = false;
  ros::NodeHandle nh_;
};
}
#endif
