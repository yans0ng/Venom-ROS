#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <iostream>
#include <vector>
#include <signal.h>
#include <math.h>
#include <thread>

#ifndef VENOM_NAVIGATOR_H
#define VENOM_NAVIGATOR_H
namespace venom {

enum NavigatorStatus {
  OFF = 0,
  IDLE = 1,
  BUSY = 2,
};

class Navigator {
public:
  Navigator();
  ~Navigator();
  NavigatorStatus GetStatus();
  void TakeOff(double h = 1.0);
  void Land(double h = 1.0);
  void SetPoint(const geometry_msgs::PoseStamped& ps){ setpoint_ = ps;}
  double Error(geometry_msgs::PoseStamped pose);
  void SetTolerence(double tol);
  void SetVerbose(bool verbose);

private:
  mavros_msgs::State state_;
  geometry_msgs::PoseStamped pose_;
  geometry_msgs::PoseStamped setpoint_;
  char command_;

  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;

  ros::Subscriber pose_sub_;
  ros::Subscriber state_sub_;
  ros::Subscriber command_sub_;

  ros::Publisher setpoint_pub_;
  void StateCallback(const mavros_msgs::State::ConstPtr& msg){ state_ = *msg;}
  void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){ pose_ = *msg;}
  void CommandCallback(const std_msgs::Char::ConstPtr& msg){ command_ = msg->data;}

  // Separate thread to control setpoint_
  bool nav_active_ = false;
  std::thread navigate_;
  double tolerence_ = 0.15;

  bool InitNavProcess();
  bool EndNavProcess();
  void NavProcess();

  bool verbose_ = false;
};
} // namespace venom
#endif
