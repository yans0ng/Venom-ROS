#include "util.h" // venom
#include <venom_offb/Navigator.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <eigen_conversions/eigen_msg.h> // matrix manipulation
#include <cmath> // std
#include <list>
#include <iostream>
#include <signal.h>
#include <venom_perception/Zed.h>

venom::Navigator* nav;

void exit_handler(int s) {
  ROS_WARN("Force quitting...\n");
  nav->Land();
  delete nav;
  exit(1);
}

geometry_msgs::Point target_pos;
static void point_callback(geometry_msgs::Point::Ptr msg) {
  target_pos = *msg;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "hunt_drone", ros::init_options::NoSigintHandler);
  signal(SIGINT, exit_handler);
  char c = ' ';
  int rc;
  
  ros::NodeHandle nh;
  ros::Subscriber target_sub = nh.subscribe("/venom/target_pos",10,point_callback);

  venom::Zed zed;
  zed.Enable(venom::PerceptionType::ODOM);

  nav = new venom::Navigator();
  //double tol = 0.3;
  //nav->SetTolerence(tol);
  //nav->SetVerbose(true);
  nav->TakeOff(2.0);
  ros::Duration d(0.5);

  geometry_msgs::PoseStamped cmd;
  cmd.pose.position.x = 0.0;
  cmd.pose.position.y = 0.0;
  cmd.pose.position.z = 2.0;
  cmd.pose.orientation.x = 0.0;
  cmd.pose.orientation.y = 0.0;
  cmd.pose.orientation.z = 0.0;
  cmd.pose.orientation.w = 1.0;
  Eigen::Affine3d t;
  ROS_INFO("Searching target...");
  while (ros::ok() ) {
    int rc = venom::wait_key(0,1000,c);
    if (c == 'q' || rc < 0)
      break;
    tf::poseMsgToEigen (cmd.pose, t);
    t.rotate (Eigen::AngleAxisd (M_PI/10.0, Eigen::Vector3d::UnitZ()));
    tf::poseEigenToMsg(t, cmd.pose);
    nav->SetPoint(cmd);
    ros::spinOnce();
    d.sleep();
  }
  c = 'x';
  //target_pos.x = 0.5;
  //target_pos.y = 0.5;
  //target_pos.z = 0.2;
  ROS_INFO("Begin z-axis following");
  while (ros::ok() && nav->GetStatus() != venom::NavigatorStatus::OFF) {
    int rc = venom::wait_key(0,1000,c);
    if (c == 'q' || rc < 0)
      break;
    if (target_pos.x==0 && target_pos.y==0 && target_pos.z==0) {
      d.sleep();
      ros::spinOnce();
    }

    geometry_msgs::Pose curr = zed.GetPose();
    double theta = atan2(target_pos.y,target_pos.x);
 
    double dist = 0.0; // TODO: uncomment to move forward
    //double dist = std::max(sqrt(target_pos.y*target_pos.y + target_pos.x*target_pos.x)-0.2, 0.0);

    tf::poseMsgToEigen (cmd.pose, t);
    t.rotate (Eigen::AngleAxisd (theta, Eigen::Vector3d::UnitZ()));
    t.translation() << cmd.pose.position.x + dist*cos(theta), cmd.pose.position.y+dist*sin(theta), cmd.pose.position.z+target_pos.z;

    tf::poseEigenToMsg(t, cmd.pose);
    nav->SetPoint(cmd);

    d.sleep();
    ros::spinOnce();
    target_pos.x = 0.0; // clear buffer
    target_pos.y = 0.0;
    target_pos.z = 0.0;
  }


  nav->Land(0.8);
  delete nav;
  return 0;
}
