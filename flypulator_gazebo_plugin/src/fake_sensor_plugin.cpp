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

#include <boost/random/normal_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/seed_seq.hpp>

#define PI (M_PI)

namespace gazebo
{

   int g_output_rate_divider = 10; // output rate = 1000Hz/ouput_rate_divider

   float g_sigma_p = 0;//1 / 3.0f;
   float g_sigma_v = 0; //sqrt(2)*g_sigma_p/(g_output_rate_divider/1000) / 3.0f;
   float g_sigma_phi = 0;//M_PI / 180.0f * 1 / 3.0f;
   float g_sigma_omega = 0;//sqrt(2)*g_sigma_phi/(g_output_rate_divider/1000) / 3.0f;
   //https://stackoverflow.com/questions/25193991/how-to-initialize-boostmt19937-with-multiple-values-without-using-c11
   boost::random::seed_seq seed_x ({1ul, 2ul, 3ul, 4ul});
   boost::random::seed_seq seed_y ({5ul, 6ul, 7ul, 8ul});
   boost::random::seed_seq seed_z ({9ul, 10ul, 11ul, 12ul});
   boost::random::seed_seq seed_v_x ({13ul, 14ul, 15ul, 16ul});
   boost::random::seed_seq seed_v_y ({17ul, 18ul, 19ul, 20ul});
   boost::random::seed_seq seed_v_z ({21ul, 22ul, 23ul, 24ul});
   boost::random::seed_seq seed_roll ({25ul, 26ul, 27ul, 28ul});
   boost::random::seed_seq seed_pitch ({29ul, 30ul, 31ul, 32ul});
   boost::random::seed_seq seed_yaw ({33ul, 34ul, 35ul, 36ul});
   boost::random::seed_seq seed_om_x ({37ul, 38ul, 39ul, 40ul});
   boost::random::seed_seq seed_om_y ({41ul, 42ul, 43ul, 44ul});
   boost::random::seed_seq seed_om_z ({45ul, 46ul, 47ul, 48ul});

   boost::variate_generator<boost::mt19937, boost::normal_distribution<> > g_noise_generator_x (boost::mt19937(seed_x), boost::normal_distribution<>(0,g_sigma_p));
   boost::variate_generator<boost::mt19937, boost::normal_distribution<> > g_noise_generator_y (boost::mt19937(seed_y), boost::normal_distribution<>(0,g_sigma_p));
   boost::variate_generator<boost::mt19937, boost::normal_distribution<> > g_noise_generator_z (boost::mt19937(seed_z), boost::normal_distribution<>(0,g_sigma_p));
   boost::variate_generator<boost::mt19937, boost::normal_distribution<> > g_noise_generator_v_x (boost::mt19937(seed_v_x), boost::normal_distribution<>(0,g_sigma_v));
   boost::variate_generator<boost::mt19937, boost::normal_distribution<> > g_noise_generator_v_y (boost::mt19937(seed_v_y), boost::normal_distribution<>(0,g_sigma_v));
   boost::variate_generator<boost::mt19937, boost::normal_distribution<> > g_noise_generator_v_z (boost::mt19937(seed_v_z), boost::normal_distribution<>(0,g_sigma_v));
   boost::variate_generator<boost::mt19937, boost::normal_distribution<> > g_noise_generator_roll (boost::mt19937(seed_roll), boost::normal_distribution<>(0,g_sigma_phi));
   boost::variate_generator<boost::mt19937, boost::normal_distribution<> > g_noise_generator_pitch (boost::mt19937(seed_pitch), boost::normal_distribution<>(0,g_sigma_phi));
   boost::variate_generator<boost::mt19937, boost::normal_distribution<> > g_noise_generator_yaw (boost::mt19937(seed_yaw), boost::normal_distribution<>(0,g_sigma_phi));
   boost::variate_generator<boost::mt19937, boost::normal_distribution<> > g_noise_generator_om_x (boost::mt19937(seed_om_x), boost::normal_distribution<>(0,g_sigma_omega));
   boost::variate_generator<boost::mt19937, boost::normal_distribution<> > g_noise_generator_om_y (boost::mt19937(seed_om_y), boost::normal_distribution<>(0,g_sigma_omega));
   boost::variate_generator<boost::mt19937, boost::normal_distribution<> > g_noise_generator_om_z (boost::mt19937(seed_om_z), boost::normal_distribution<>(0,g_sigma_omega));


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

    float three_sigma_p = 0;
    float three_sigma_v = 0;
    float three_sigma_phi = 0;
    float three_sigma_omega = 0;
    // take noise values from parameter server
    // and change distributions if value read from file
    // https://stackoverflow.com/questions/36289180/boostrandomvariate-generator-change-parameters-after-construction
    if (ros::param::get("state/three_sigma_p", three_sigma_p)){
        ROS_DEBUG("Three_sigma_p load successfully from parameter server");
        boost::normal_distribution<> new_dist ( 0, three_sigma_p / 3.0f );
        g_noise_generator_x.distribution() = new_dist;
        g_noise_generator_y.distribution() = new_dist;
        g_noise_generator_z.distribution() = new_dist;
    } 
    if (ros::param::get("state/three_sigma_v", three_sigma_v)){
        ROS_DEBUG("Three_sigma_v load successfully from parameter server");
        boost::normal_distribution<> new_dist ( 0, three_sigma_v / 3.0f );
        g_noise_generator_v_x.distribution() = new_dist;
        g_noise_generator_v_y.distribution() = new_dist;
        g_noise_generator_v_z.distribution() = new_dist;
    } 
    if (ros::param::get("state/three_sigma_phi", three_sigma_phi)){
        ROS_DEBUG("Three_sigma_phi load successfully from parameter server");
        boost::normal_distribution<> new_dist ( 0, three_sigma_phi * M_PI/180.0f / 3.0f );
        g_noise_generator_roll.distribution() = new_dist;
        g_noise_generator_pitch.distribution() = new_dist;
        g_noise_generator_yaw.distribution() = new_dist;
    } 
    if (ros::param::get("state/three_sigma_omega", three_sigma_omega)){
        ROS_DEBUG("Three_sigma_omega load successfully from parameter server");
        boost::normal_distribution<> new_dist ( 0, three_sigma_omega * M_PI/180.0f / 3.0f );
        g_noise_generator_om_x.distribution() = new_dist;
        g_noise_generator_om_y.distribution() = new_dist;
        g_noise_generator_om_z.distribution() = new_dist;
    } 

    
 
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

    if(loop_cnt >= (g_output_rate_divider-1))
    {
      loop_cnt = 0; // reset loop counter
      // ROS_INFO_STREAM("I am fake sensor:"<<this->world->GetSimTime().Double());
      math::Pose drone_pose = this->link0->GetWorldPose();
      math::Vector3 drone_vel_linear = this->link0->GetWorldLinearVel(); 
      math::Vector3 drone_vel_angular = this->link0->GetRelativeAngularVel(); 
      math::Vector3 drone_acc_linear = this->link0->GetWorldLinearAccel();
      math::Vector3 drone_acc_angular = this->link0->GetRelativeAngularAccel();

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

      //ROS_INFO("x noise = %f ", g_noise_generator_x());
      // ROS_INFO("y noise = %f ", g_noise_generator_y());

      flypulator_common_msgs::UavStateStamped uav_state_meas_msg;
      // pose
      uav_state_meas_msg.pose.position.x = drone_pose.pos.x + g_noise_generator_x();
      uav_state_meas_msg.pose.position.y = drone_pose.pos.y + g_noise_generator_y();
      uav_state_meas_msg.pose.position.z = drone_pose.pos.z + g_noise_generator_z();

      // add attitude noise using roll pitch yaw representation
      math::Vector3 eul (drone_pose.rot.GetRoll(), drone_pose.rot.GetPitch(), drone_pose.rot.GetYaw());
      eul.x = eul.x + g_noise_generator_roll();
      eul.y = eul.y + g_noise_generator_pitch();
      eul.z = eul.z + g_noise_generator_yaw();
      math::Quaternion q_n (eul);
      uav_state_meas_msg.pose.orientation.w = q_n.w;
      uav_state_meas_msg.pose.orientation.x = q_n.x;
      uav_state_meas_msg.pose.orientation.y = q_n.y;
      uav_state_meas_msg.pose.orientation.z = q_n.z;

      // velocity
      uav_state_meas_msg.velocity.linear.x = drone_vel_linear.x + g_noise_generator_v_x();
      uav_state_meas_msg.velocity.linear.y = drone_vel_linear.y + g_noise_generator_v_y();
      uav_state_meas_msg.velocity.linear.z = drone_vel_linear.z + g_noise_generator_v_z();
      uav_state_meas_msg.velocity.angular.x = drone_vel_angular.x + g_noise_generator_om_x();
      uav_state_meas_msg.velocity.angular.y = drone_vel_angular.y + g_noise_generator_om_y();
      uav_state_meas_msg.velocity.angular.z = drone_vel_angular.z + g_noise_generator_om_z();
      // acceleration // still very noisy
      uav_state_meas_msg.acceleration.linear.x = drone_acc_linear.x;
      uav_state_meas_msg.acceleration.linear.y = drone_acc_linear.y;
      uav_state_meas_msg.acceleration.linear.z = drone_acc_linear.z;
      uav_state_meas_msg.acceleration.angular.x = drone_acc_angular.x;
      uav_state_meas_msg.acceleration.angular.y = drone_acc_angular.y;
      uav_state_meas_msg.acceleration.angular.z = drone_acc_angular.z;

      uav_state_meas_msg.header.stamp = ros::Time(this->world->GetSimTime().Double());
    
      // publish real states of the simulated drone
      pub_real_state.publish(uav_state_msg);
  
      // publish measured states of the simulated drone
      pub_meas_state.publish(uav_state_meas_msg);
    }
    else
      loop_cnt++;

    ros::spinOnce();
  }

private:
  void QueueThread()
  {
    static const double timeout = 0.01;
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

};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(FakeSensorPlugin)
} // namespace gazebo

#endif /*_FAKE_SENSOR_PLUGIN_HH_*/
