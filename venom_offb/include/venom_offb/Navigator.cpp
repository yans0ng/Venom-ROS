#include "Navigator.h"

namespace venom {

Navigator::Navigator() {

  setpoint_.pose.position.x = 0.0; 
  setpoint_.pose.position.y = 0.0;
  setpoint_.pose.position.z = 0.0;
  setpoint_.pose.orientation.x = 0.0;
  setpoint_.pose.orientation.y = 0.0;
  setpoint_.pose.orientation.z = 0.0;
  setpoint_.pose.orientation.w = 0.0;

  nav_active_ = false;

  ros::NodeHandle nh;
  arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &Navigator::PoseCallback, this);
  state_sub_ = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &Navigator::StateCallback, this);
  command_sub_ = nh.subscribe<std_msgs::Char>("/venom/high_level", 10, &Navigator::CommandCallback, this);

  setpoint_pub_ = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

  while(ros::ok() && !state_.connected){
      ros::spinOnce();
      ros::Duration(0.5).sleep();
      ROS_INFO("Waiting for FCU connection");
  }
}

Navigator::~Navigator() {
  pose_sub_.shutdown();
  state_sub_.shutdown();
  command_sub_.shutdown();
  Land();
}

NavigatorStatus Navigator::GetStatus() {
  if (!state_.connected || !nav_active_)
    return NavigatorStatus::OFF;
  else if (Error(setpoint_) > tolerence_)
    return NavigatorStatus::BUSY;
  else
    return NavigatorStatus::IDLE;
}

void Navigator::TakeOff(double h) {
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";
  set_mode_client_.call(offb_set_mode);

  ros::Time last_request = ros::Time::now();

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  setpoint_.pose.position.x = 0.0; 
  setpoint_.pose.position.y = 0.0;
  setpoint_.pose.position.z = h;
  setpoint_.pose.orientation.x = 0.0;
  setpoint_.pose.orientation.y = 0.0;
  setpoint_.pose.orientation.z = 0.0;
  setpoint_.pose.orientation.w = 0.0;

  ROS_INFO("Buffering setpoints (3 sec)...");
  ros::Duration d(0.1);
  for (int i = 0; i < 30; i++) {
    setpoint_pub_.publish(setpoint_);
    ros::spinOnce();
    d.sleep();
  }
  ROS_INFO("Buffer done");

  if (!state_.connected) {
    ROS_ERROR("Not connected");
    return;
  }
  if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent )
    ROS_INFO("Offboard enabled");
  if (arming_client_.call(arm_cmd) && arm_cmd.response.success)
    ROS_INFO("Vehicle armed");
  if (InitNavProcess() )
    ROS_INFO("Init SUCCESS");

  while (Error(setpoint_) > 0.2) {
    setpoint_pub_.publish(setpoint_);
    ros::spinOnce();
    d.sleep();
  }
}

void Navigator::Land(double h) {
  if (!EndNavProcess()) {
    ROS_DEBUG("Thread not running");
    return;
  }

  h = std::max(0.1, h); // make sure h is sufficiently high
  setpoint_.pose.position.x = 0.0;
  setpoint_.pose.position.y = 0.0;
  setpoint_.pose.position.z = h;

  ROS_INFO("Landing...");
  ros::Duration d(0.01);
  while (Error(setpoint_) > 0.3) {
    setpoint_pub_.publish(setpoint_);
    ros::spinOnce();
    d.sleep();
  }

  if (!state_.connected) {
    ROS_ERROR("Not connected");
    return;
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "AUTO.LAND";
  //offb_set_mode.request.custom_mode = "STABILIZED";

  if( set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent )
    ROS_INFO("Switched to AUTO.LAND mode");
  else
    ROS_ERROR("Fail to switch mode");
  
  ROS_INFO("Disarming...");
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;
  if (arming_client_.call(arm_cmd) && arm_cmd.response.success)
    ROS_INFO("Vehicle disarmed");
}

double Navigator::Error(geometry_msgs::PoseStamped pose) {
  double error = 0;
  error += fabs(pose_.pose.position.x - pose.pose.position.x);
  error += fabs(pose_.pose.position.y - pose.pose.position.y);
  error += fabs(pose_.pose.position.z - pose.pose.position.z);
  if (verbose_) {
    ROS_INFO_STREAM("current: \n" << pose_.pose.position);
    ROS_INFO_STREAM("command: \n" << pose.pose.position);
    ROS_INFO_STREAM("error: " << error);
    ROS_INFO("-------------------");
  }
  return error;
}

bool Navigator::InitNavProcess() {
  if (nav_active_) {
    ROS_DEBUG("Thread already running");
    return false;
  }
  nav_active_ = true;
  navigate_ = std::thread(&Navigator::NavProcess, this);
  return true;
}

bool Navigator::EndNavProcess() {
  if (!nav_active_) {
    ROS_DEBUG("Thread not running");
    return false;
  }
  nav_active_ = false;
  navigate_.join();
  return true;
}

void Navigator::NavProcess() {
  ros::Duration d(0.1);
  while (ros::ok() && GetStatus()) {
    setpoint_pub_.publish(setpoint_);
    d.sleep();
  }
  if (nav_active_)
    ROS_WARN("NavProcess terminated with exception");
}

void Navigator::SetTolerence(double tol) {
  tolerence_ = tol;
}

void Navigator::SetVerbose(bool verbose) {
  verbose_ = verbose;
}

} // namespace venom
