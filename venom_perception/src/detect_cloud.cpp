#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <venom_perception/Zed.h>
#include <opencv2/opencv.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/centroid.h>

// Callback function from YOLO
// Update bounding box indices
int px1=0,px2=0,py1=0,py2=0;
bool trigger = false;
static void bb_callback(std_msgs::Int32MultiArray::ConstPtr msg) {
  px1 = msg->data[0];
  py1 = msg->data[1];
  px2 = msg->data[2];
  py1 = msg->data[3];
  trigger = true;
}

int main (int argc, char** argv) {
  ros::init(argc, argv, "detect_drone");
  ros::NodeHandle nh;
  ros::Publisher target_pub = nh.advertise<geometry_msgs::Point>("/venom/target_pos", 10);
  ros::Subscriber bb_sub = nh.subscribe<std_msgs::Int32MultiArray>("/venom/bounding_box", 1, bb_callback);
  venom::Zed zed;
  zed.Enable(venom::PerceptionType::RGB);
  zed.Enable(venom::PerceptionType::CLOUD);
  ros::Rate rate(10);
  while (ros::ok()) {
    cv::Mat rgb = zed.GetRGB();
    if (px1 == 0 && px2 == 0 && py1 == 0 && py2 == 0) {
      ROS_DEBUG("Nothing detected");
      ros::spinOnce();
      rate.sleep();
      continue;
    }
    ROS_INFO_STREAM("Detect bounding box: ("<< px1 << "," << py1 << ") to ("
                    << px2 << "," << py2 << ")");
    if (trigger) {
      zed.SetROI(px1,px2,py1,py2);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi = zed.GetCloud();

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
      sor.setInputCloud (roi);
      sor.setMeanK (50);
      sor.setStddevMulThresh (1.0);
      sor.filter (*filtered);

      if (!filtered->empty()) {
      pcl::PointXYZ centroid;
      pcl::computeCentroid(*filtered,centroid);
      geometry_msgs::Point target_pos;

      target_pos.x = centroid.x;
      target_pos.y = centroid.y;
      target_pos.z = centroid.z;
      target_pub.publish(target_pos);
      }

    }
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
