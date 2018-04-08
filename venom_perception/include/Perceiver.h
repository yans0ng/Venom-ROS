#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>

namespace venom {
class Perceiver {
public:
  Perceiver() {
    depth_flag_ = true;
    cloud_flag_ = true;
    ros::NodeHandle nh;
    cloud_sub_ = nh.subscribe("/zed/point_cloud/cloud_registered", 10, &Perceiver::CloudCallback, this);
    odom_sub_ = nh.subscribe("/zed/odom", 10, &Perceiver::OdometryCallback, this);
    depth_sub_ = nh.subscribe("/zed/depth/depth_registered", 10, &Perceiver::DepthImageCallback, this);
    rgb_sub_ = nh.subscribe("/zed/rgb/image_rect_color",10,&Perceiver::RGBImageCallback, this);
    cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/venom/enemy_drone", 1);
    start_.first = start_.second = end_.first = end_.second = 0;
  }
  void SavePointCloud(const std::string& filename) {
    pcl::io::savePCDFile (filename, pcl_cloud_);
  }
  sensor_msgs::PointCloud2::ConstPtr GetRosPointCloud() {
    // Convert to ROS data type
    sensor_msgs::PointCloud2* cloud_ptr = new sensor_msgs::PointCloud2;
    pcl_conversions::fromPCL(pcl_cloud_, *cloud_ptr);
    sensor_msgs::PointCloud2::ConstPtr output(cloud_ptr);
    return output;
  }
  void SavePose(const std::string& filename) {
    std::ofstream os;
    os.open(filename);
    os << pose_;
    os.close();
  }

  bool RGBReady() {
    return (rgb_ptr_ != NULL);
  }
  cv::Mat GetRGBImage() {
    return cv::Mat(rgb_ptr_->image);
  }

  void SetVerbose(bool x) {
    verbose_ = x;
  }

  void MarkPoints(std::pair<int,int> start, std::pair<int,int> end) {
    start_ = start;
    end_ = end;
  }

private:
  bool cloud_flag_;
  bool depth_flag_;
  bool verbose_ = false;
  std::pair<int,int> start_, end_;
  pcl::PCLPointCloud2 pcl_cloud_; // Filtered and processed
  ros::Subscriber cloud_sub_;
  ros::Publisher cloud_pub_; 
  void CloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud_raw_;
    pcl::fromROSMsg(*cloud_msg,pcl_cloud_raw_);
    for (int i = start_.first; i < end_.first; i++) {
      for (int j = start_.second; j < end_.second; j++) {
        pcl::PointXYZRGB& temp = pcl_cloud_raw_.at(i,j);
	temp.r = 255;
	temp.g = 0;
	temp.b = 0;
      }
    }
    sensor_msgs::PointCloud2 enemy_cloud;
    pcl::toROSMsg(pcl_cloud_raw_, enemy_cloud);
    cloud_pub_.publish(enemy_cloud);

    // Container for original and filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize(0.1,0.1,0.1);
    sor.filter(pcl_cloud_); // TODO: what about the original data? Memory leak?

    // Check distances
    if (depth_flag_) {
      cloud_flag_ = true;
      return;
    }
    pcl::PointCloud<pcl::PointXYZ> pcl_xyz;
    pcl::fromPCLPointCloud2(pcl_cloud_, pcl_xyz);
    if (verbose_)
     ROS_DEBUG_STREAM("Number of point: " << pcl_xyz.size());
    if (pcl_xyz.size() < 5000)
      cloud_flag_ = false;
    else
      cloud_flag_ = true;
  }

  geometry_msgs::PoseWithCovarianceStamped pose_;
  ros::Subscriber odom_sub_;
  void OdometryCallback(const nav_msgs::Odometry& msg) {
    pose_.header = msg.header;
    pose_.pose = msg.pose;
  }


  ros::Subscriber depth_sub_;
  cv_bridge::CvImagePtr cv_ptr;
  void DepthImageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    // NOTE: cv::countNonZero only apply to grayscale images, since it is hard to
    // convert to grayscale, I count zero pixels on my own.
    float zero_count = 0;
    for (int i = 0; i < cv_ptr->image.rows; i++)
      for (int j = 0; j < cv_ptr->image.cols; j++)
	if (cv_ptr->image.at<int>(i,j) < 5)
	  zero_count++;
    float zero_ratio = zero_count / cv_ptr->image.cols / cv_ptr->image.rows;
    if (zero_ratio > 0.2)
      depth_flag_ = false;
    else
      depth_flag_ = true;
  }

  ros::Subscriber rgb_sub_;
  cv_bridge::CvImagePtr rgb_ptr_;
  void RGBImageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
      rgb_ptr_ = cv_bridge::toCvCopy(msg);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    if (verbose_)
      ROS_DEBUG_STREAM("image size: " << rgb_ptr_->image.rows << ", " << rgb_ptr_->image.cols);
  }
};
} // namespace venom
