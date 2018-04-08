#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


ros::Publisher pose_pub;
ros::Publisher pose_cov_pub;
geometry_msgs::PoseWithCovarianceStamped pose_cov;
geometry_msgs::PoseStamped pose;
static void nav_to_geometry(const nav_msgs::Odometry& msg) {
  pose_cov.header = msg.header;
  pose_cov.pose = msg.pose;

  pose.header = msg.header;
  pose.pose = msg.pose.pose;

  pose_cov_pub.publish(pose_cov);
  pose_pub.publish(pose);
}

int main (int argc, char** argv) {
  ros::init(argc, argv, "vision_pose_converter");
  ros::NodeHandle nh;
  ros::Subscriber odom_sub = nh.subscribe("/zed/odom", 10, nav_to_geometry);

  pose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose",10);
  pose_cov_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/vision_pose/pose_cov",10);
  //while (ros::ok()) {
  //  ros::Duration(0.5).sleep();
  //  ros::spinOnce();
  //}
  ros::spin();

  return 0;
}
