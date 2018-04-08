#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <venom_offb/Navigator.h>
#include <venom_perception/Zed.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include "util.h"

int px1=0,px2=0,py1=0,py2=0;
//int cx = 360, cy = 540; // 720p
int cx = 128, cy = 128; // cv2.resize to 256x256
//int cx = 320, cy = 240; // VGA
int tolx = cx/12, toly = cy/12;
bool trigger = false;
static void bb_callback(std_msgs::Int32MultiArray::ConstPtr msg) {
  px1 = std::max(msg->data[0],0);
  py1 = std::max(msg->data[1],0);
  px2 = std::min(msg->data[2],cx*2);
  py2 = std::min(msg->data[3],cy*2);
  trigger = true;
}

venom::Navigator* nav;                                                          
                                                                                
void exit_handler(int s) {                                                      
  ROS_WARN("Force quitting...\n");                                              
  nav->Land();                                                                  
  delete nav;                                                                   
  exit(1);                                                                      
}

int main (int argc, char** argv) {
  ros::init(argc, argv, "visual_servo", ros::init_options::NoSigintHandler);
  signal(SIGINT, exit_handler);
  ros::NodeHandle nh;
  ros::Subscriber bb_sub = nh.subscribe<std_msgs::Int32MultiArray>("/venom/bounding_box", 1, bb_callback);

  venom::Zed zed;
  zed.Enable(venom::PerceptionType::ODOM);

  nav = new venom::Navigator();
  nav->TakeOff(1.0);

  ros::Duration d(0.5);
  ROS_INFO("Searching target...");
  geometry_msgs::PoseStamped cmd;
  cmd.pose.position.x = 0.0;
  cmd.pose.position.y = 0.0;
  cmd.pose.position.z = 1.0;
  cmd.pose.orientation.x = 0.0;
  cmd.pose.orientation.y = 0.0;
  cmd.pose.orientation.z = 0.0;
  cmd.pose.orientation.w = 1.0;
  Eigen::Affine3d t;
  char c = 'x';
  while (ros::ok()) {
    venom::wait_key(0,1000,c);
    if (c == 'q')
      break;
    tf::poseMsgToEigen (cmd.pose, t);
    t.rotate (Eigen::AngleAxisd (M_PI/10.0, Eigen::Vector3d::UnitZ()));
    tf::poseEigenToMsg(t, cmd.pose);
    nav->SetPoint(cmd);
    ros::spinOnce();
    d.sleep();
  }
  c = 'x';
  ROS_INFO("Begin z-axis following");
  while (ros::ok()) {
    venom::wait_key(0,1000,c);
    if (c == 'q')
      break;
    if (px1!=0 || px2!=0 || py1 != 0 || py2 != 0) {
      // Compute the center of bounding box
      int midx = (px1 + px2)/2, midy = (py1 + py2)/2;
      ROS_INFO_STREAM("target center (" << midx << ", " << midy << ")");

      // TODO: set dist = 0.2 if you want to move forward.
      double theta = 0.0, dist = 0.2, dz = 0.0;
      if (cy - midy > toly ) {
        ROS_INFO("Go up");
        dz = 0.10;
      } else if (midy - cy > toly ) {
        ROS_INFO("Go down");
        dz = -0.10;
      }
      if (midx - cx > tolx ) {
        ROS_INFO("Turn right");
        theta = -M_PI/6.0;
      } else if (cx - midx > tolx ) {
        ROS_INFO("Turn left");
        theta = M_PI/6.0;
      }

      // Matrix tranformation
      tf::poseMsgToEigen (cmd.pose, t);
      t.rotate (Eigen::AngleAxisd (theta, Eigen::Vector3d::UnitZ()));
      tf::poseEigenToMsg(t, cmd.pose);
      cmd.pose.position.x += dist * cos(theta);
      cmd.pose.position.y += dist * sin(theta);
      cmd.pose.position.z += dz;
      nav->SetPoint(cmd);

      px1 = py1 = px2 = py2 = 0; // clear buffered values
    }
    d.sleep();
    ros::spinOnce();
  }
  nav->Land(0.8);
  return 0;
}
