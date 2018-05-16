#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include "geometry_msgs/Wrench.h"

class TeleopDrone
{
public:
  TeleopDrone(ros::NodeHandle nh);

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);

  ros::NodeHandle nh_;
  int force1_, force2_, force3_, torque1_, torque2_, torque3_;
  double f_scale_, t_scale_;

  ros::Publisher wrench_pub_; //publish control wrench signal
  ros::Subscriber joy_sub_;
};

TeleopDrone::TeleopDrone(ros::NodeHandle  nh) :nh_(nh),force1_(1),force2_(2),force3_(3),torque1_(4),torque2_(5),torque3_(6)
{
  nh_.param("axis_force1", force1_, force1_);
  nh_.param("axis_force2", force2_, force2_);
  nh_.param("axis_force3", force3_, force3_);
  nh_.param("axis_torque1", torque1_, torque1_);
  nh_.param("axis_torque2", torque2_, torque2_);
  nh_.param("axis_torque3", torque3_, torque3_);
  nh_.param("scale_force", f_scale_, f_scale_);
  nh_.param("scale_torque", t_scale_, t_scale_);

  wrench_pub_ = nh_.advertise<geometry_msgs::Wrench>("/drone/vel_cmd", 100);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 100, &TeleopDrone::joyCallback, this);
}

void TeleopDrone::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
  geometry_msgs::Wrench msg;
  msg.force.x = f_scale_ * (1.0 - joy->axes[force1_])/2.0;
  msg.force.y = f_scale_ * (1.0 - joy->axes[force2_])/2.0;
  msg.force.z = f_scale_ * joy->axes[force3_];
  msg.torque.x = t_scale_ * joy->axes[torque1_] * -1.0;
  msg.torque.y = t_scale_ * joy->axes[torque2_];
  msg.torque.z = t_scale_ * joy->axes[torque3_];
  wrench_pub_.publish(msg);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "teleop_drone");

  TeleopDrone teleop_drone(ros::NodeHandle("~"));
  // ros::Rate loop_rate(20);
  ros::spin();
  //ros::spinOnce();
  // loop_rate.sleep();
  return 0;
}
