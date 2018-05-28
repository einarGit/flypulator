#ifndef _FAKE_SENSOR_PLUGIN_HH_
#define _FAKE_SENSOR_PLUGIN_HH_

#include <vector>
#include <iostream>
#include <math.h>
#include <thread>
#include <fstream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>

#include <gazebo_msgs/LinkStates.h>

#include <Eigen/Dense>

#include <flypulator_common_msgs/UavStateStamped.h>

#define PI (M_PI)

namespace gazebo
{

class FakeSensorPlugin : public ModelPlugin
{
  
public:
  FakeSensorPlugin() {}

public:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    ROS_INFO_STREAM("Loading FakeSensorPlugin ...");
    if (_model->GetJointCount() == 0)
    {
      ROS_ERROR("Invalid joint count, plugin not loaded");
      return;
    }

    // Store the model pointer for convenience
    this->model = _model;
    this->world = _model->GetWorld();

    this->link0 = _model->GetChildLink("base_link");
 
    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized())
    {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "gazebo_client",
                ros::init_options::NoSigintHandler);
      ROS_WARN_STREAM("Create gazebo_client node");
    }
    // Create our ROS node.
    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
    ROS_INFO_STREAM("fake_sensor_plugin get node:" << this->rosNode->getNamespace());
    
    // real states of the drone
    this->pub_real_state = this->rosNode->advertise<flypulator_common_msgs::UavStateStamped>("/drone/real_state", 100);
    // measured states of the drone
    this->pub_meas_state = this->rosNode->advertise<flypulator_common_msgs::UavStateStamped>("/drone/meas_state", 100);

    //subscribe to model link states to get position and orientation for coordinate transformation
    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<gazebo_msgs::LinkStates>(
            "/gazebo/link_states", 100, boost::bind(&FakeSensorPlugin::OnlinkMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
    this->rosSubLink = this->rosNode->subscribe(so);

    // Spin up the queue helper thread.
    this->rosQueueThread = std::thread(std::bind(&FakeSensorPlugin::QueueThread, this));

    ROS_INFO_STREAM("FakeSensorPlugin Loaded !");
  }

// publish drone state
public:
  void OnlinkMsg(const gazebo_msgs::LinkStatesConstPtr &_msg)
  {
    static int loop_cnt = 0;

    if(loop_cnt >= (ouput_rate_divider-1))
    {
      loop_cnt = 0; // reset loop counter
      // ROS_INFO_STREAM("I am fake sensor:"<<this->world->GetSimTime().Double());
      math::Pose drone_pose = this->link0->GetWorldPose();
      math::Vector3 drone_vel_linear = this->link0->GetWorldLinearVel(); 
      math::Vector3 drone_vel_angular = this->link0->GetWorldAngularVel(); 
      math::Vector3 drone_acc_linear = this->link0->GetWorldLinearAccel();
      math::Vector3 drone_acc_angular = this->link0->GetWorldAngularAccel();

      flypulator_common_msgs::UavStateStamped uav_state_msg;
      uav_state_msg.header.stamp = ros::Time(this->world->GetSimTime().Double());
      // pose
      uav_state_msg.pose.position.x = drone_pose.pos.x;
      uav_state_msg.pose.position.y = drone_pose.pos.y;
      uav_state_msg.pose.position.z = drone_pose.pos.z;
      uav_state_msg.pose.orientation.w = drone_pose.rot.w;
      uav_state_msg.pose.orientation.x = drone_pose.rot.x;
      uav_state_msg.pose.orientation.y = drone_pose.rot.y;
      uav_state_msg.pose.orientation.z = drone_pose.rot.z;
      // velocity
      uav_state_msg.velocity.linear.x = drone_vel_linear.x;
      uav_state_msg.velocity.linear.y = drone_vel_linear.y;
      uav_state_msg.velocity.linear.z = drone_vel_linear.z;
      uav_state_msg.velocity.angular.x = drone_vel_angular.x;
      uav_state_msg.velocity.angular.y = drone_vel_angular.y;
      uav_state_msg.velocity.angular.z = drone_vel_angular.z;
      // acceleration
      uav_state_msg.acceleration.linear.x = drone_acc_linear.x;
      uav_state_msg.acceleration.linear.y = drone_acc_linear.y;
      uav_state_msg.acceleration.linear.z = drone_acc_linear.z;
      uav_state_msg.acceleration.angular.x = drone_acc_angular.x;
      uav_state_msg.acceleration.angular.y = drone_acc_angular.y;
      uav_state_msg.acceleration.angular.z = drone_acc_angular.z;
    
      // publish real states of the simulated drone
      pub_real_state.publish(uav_state_msg);
  
      // publish measured states of the simulated drone
      // TODO: add artifical noise to measured states
      pub_meas_state.publish(uav_state_msg);
    }
    else
      loop_cnt++;

    ros::spinOnce();
  }

private:
  void QueueThread()
  {
    static const double timeout = 1;
    while (this->rosNode->ok())
    {
      this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }

  /// \brief Pointer to the model.
private:
  physics::ModelPtr model;
  physics::WorldPtr world;

private:
  physics::LinkPtr link0;

  /// \brief A node use for ROS transport
private:
  std::unique_ptr<ros::NodeHandle> rosNode;

private:
 ros::Subscriber rosSubLink;

  /// \brief A ROS callbackqueue that helps process messages
private:
  ros::CallbackQueue rosQueue;

  /// \brief A thread the keeps running the rosQueue
private:
  std::thread rosQueueThread;

private:
  ros::Publisher pub_real_state;
  ros::Publisher pub_meas_state;

  int ouput_rate_divider = 4; // output rate = 1000Hz/ouput_rate_divider
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(FakeSensorPlugin)
} // namespace gazebo

#endif /*_FAKE_SENSOR_PLUGIN_HH_*/
