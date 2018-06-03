#ifndef _PROPULSION_PLUGIN_HH_
#define _PROPULSION_PLUGIN_HH_

#include <vector>
#include <iostream>
#include <math.h>
#include <thread>
#include <fstream>
#include <algorithm>

#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/gazebo_client.hh>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>

#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Vector3.h>

#include <sensor_msgs/JointState.h>

#include <tf/transform_broadcaster.h>

#include <Eigen/Dense>

#include <flypulator_common_msgs/Vector6dMsg.h>
#include <flypulator_common_msgs/RotorVelStamped.h>

#include <motor_model.hpp>

namespace gazebo
{
/// \brief A plugin to control drone
class PropulsionPlugin : public ModelPlugin
{
  bool write_data_2_file = true; // new version with more data (contains also hub force and roll moment)
  bool WRITE_CSV_FILE = false; // if save the test_data to .csv
  bool add_wrench_to_drone = true; // if add force and torque to drone in gazebo
  bool use_ground_effect = false; // if enable ground effect
  bool use_motor_dynamic = true; // if enable motor dynamic
  bool use_simple_aerodynamic = true; // use f=k*omega² and tau = b * omega² // care, aerodynamic k and b have to be adapted in controller as well!

  double test_data[12]; // test data for debug
  double ground_effect_coeff; // ground effect coefficient

  int N = 6;         //number of energie
  double c = 0.016;  //blade chord width
  double R = 0.75;   //blade radius
  double a = 5.7;    //2D_lift_curve_slope
  double th0 = 0.7;  //Profile inclination angle
  double thtw = 0.5; //Inclination change along radius
  double pa;         //blade pitch angle
  double B = 0.98;   //tip loss factor
  double pho = 1.2;  //air density
  double A;          //wing area
  double ki = 1.15;  //factor to calculate torque
  double k = 4.65;   //factor to calculate torque
  //coefficients to calculate induced velocity Vi in vortex ring state
  double k0 = 1.15;
  double k1 = -1.125;
  double k2 = -1.372;
  double k3 = -1.718;
  double k4 = -0.655;
  double CD0 = 0.04;      //Profile_Drag_Coefficient from literatur
  double rv = 13.6 * M_PI / 180.0;     //rotor_axis_vertical_axis_angle cos(rv)=cos(pitch)*cos(yaw)
  double m = 6.15;               //drone_masse
  double g = 9.81;        //gravity acceleration constant
  double s;               //rotor solidity
  double Vwind_x = 1e-20; //wind velocity in global x
  double Vwind_y = 1e-20; //wind velocity in global y
  double Vwind_z = 1e-20; //wind velocity in global z
  double Vdrone_x;        //drone velocity in global x
  double Vdrone_y;        //drone velocity in global y
  double Vdrone_z;        //drone velocity in global z
  double Vx = 1e-20;      //air velocity in global x
  double Vy = 1e-20;      //air velocity in global y
  double Vz = 1e-20;      //air velocity in global z
  double Vi_h;            //induced velocity in the hovering case
  
  double k_simple_aero = 0.000055;
  double b_simple_aero = 0.0000011;

  double rotor_vel_raw[6] = {0};  // rotor velocity with sign, used for motor dynamic
  double rotor_vel_cmd[6] = {0};               // blade spinning velocity commands
  double rotor_vel[6] = {0};                   // blade spinning velocity
  int di_blade_rot[6] = {1, -1, 1, -1, 1, -1}; //default blade rotating direction 1 counterclockwise; -1 clockwise
  int di_force[6] = {1, 1, 1, 1, 1, 1};        //thrust force direction
  int di_vel[6] = {1, 1, 1, 1, 1, 1};          //real rotate direction

  bool bidirectional = true; //bidirectional option
  
  // if the rotor vel smaller than 325, we will get negativ CT and moment,
  // caused by the aero dynamic eq.(Hiller's 4.57)
  double vel_min = 0.01;      // min rotor speed
  double vel_max = 100000;      //max rotor speed
                     
  Eigen::Matrix3d T_trans; //transformation matrix from global coordinate to body coordinate

  Eigen::Vector3d force_rotor [6]; // aerodynamic forces
  Eigen::Vector3d torque_rotor [6]; // aerodynamic torques

  std::string RESULT_CSV_PATH = "/home/jinyao/ros_ws/flypulator/result.csv";


  std::string file_path = "/home/jan/flypulator_ws/src/flypulator/flypulator_gazebo_plugin/rotor_data.csv";

  gazebo::physics::LinkPtr rotor_link_ptr[6]; // pointer to rotor links



  /// \brief Constructor
public:
  PropulsionPlugin() {}

  /// \brief The load function is called by Gazebo when the plugin is
  /// inserted into simulation
  /// \param[in] _model A pointer to the model that this plugin is
  /// attached to.
  /// \param[in] _sdf A pointer to the plugin's SDF element.
public:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    ROS_INFO_STREAM("Loading PropulsionPlugin ...");
    if (_model->GetJointCount() == 0)
    {
      ROS_ERROR("Invalid joint count, plugin not loaded");
      return;
    }

    // save test data
    if (WRITE_CSV_FILE)
    {
      std::ofstream fout(RESULT_CSV_PATH, std::ios::out);
      fout.close();
    }

    if (write_data_2_file){
      result_file.open(file_path);
      result_file << "time,rotor,omega,force_x,force_y,force_z,torque_x,torque_y,torque_z" << std::endl;
    }

    // Store the model pointer for convenience.
    this->model = _model;
    // Get the first joint. We are making an assumption about the model
    // having six joints that is the rotational joint.
    this->joint1 = _model->GetJoint("blade_joint_1");
    this->joint2 = _model->GetJoint("blade_joint_2");
    this->joint3 = _model->GetJoint("blade_joint_3");
    this->joint4 = _model->GetJoint("blade_joint_4");
    this->joint5 = _model->GetJoint("blade_joint_5");
    this->joint6 = _model->GetJoint("blade_joint_6");
    //set joint velocity using joint motors to set joint velocity
    this->joint1->SetParam("fmax", 0, 1000000.0); //fmax: maximum joint force or torque
    this->joint2->SetParam("fmax", 0, 1000000.0);
    this->joint3->SetParam("fmax", 0, 1000000.0);
    this->joint4->SetParam("fmax", 0, 1000000.0);
    this->joint5->SetParam("fmax", 0, 1000000.0);
    this->joint6->SetParam("fmax", 0, 1000000.0);
    this->joint1->SetParam("vel", 0, 0);
    this->joint2->SetParam("vel", 0, 0);
    this->joint3->SetParam("vel", 0, 0);
    this->joint4->SetParam("vel", 0, 0);
    this->joint5->SetParam("vel", 0, 0);
    this->joint6->SetParam("vel", 0, 0);
    //get the six blade link
    
    this->link0 = _model->GetChildLink("base_link");
    for (int i = 0; i<6; i++){
        rotor_link_ptr[i] = _model->GetChildLink(std::string("blade_link_") + std::to_string(i+1));
    }

    //load aerodynamic parameters
    // this->readParamsFromServer();    
    
    //calculation of constants
    //m = this->link0->GetInertial()->GetMass();
    s = (N * c) / (M_PI * R);                              //rotor solidity
    A = M_PI * pow(R, 2);                                  //wing area
    Vi_h = - 1/B * sqrt((m * g) / (2 * N * pho * A * cos(rv))); //induced airflow velocity in hovering case // multiplied by 1/B according to Hiller eq. 4.58
    pa = th0 - 0.75 * thtw;                              //blade pitch angle

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
    ROS_INFO_STREAM("propulsion_plugin get node:" << this->rosNode->getNamespace());

    this->pub_ratio = this->rosNode->advertise<flypulator_common_msgs::Vector6dMsg>("/drone/thrust_moment_ratio", 10);
    this->pub_joint_state = this->rosNode->advertise<sensor_msgs::JointState>("/drone/joint_states", 50);

    for (int i = 0; i<6; i++){
        this->pub_link_wrench[i] = this->rosNode->advertise<geometry_msgs::WrenchStamped>(std::string("/drone/blade_") + std::to_string(i+1) + std::string("_wrench"), 100);
    }
    

    //Create a wind velocity topic and subscribe to it
    ros::SubscribeOptions s1 =
        ros::SubscribeOptions::create<geometry_msgs::Vector3>(
            "/drone/wind_cmd", 100, boost::bind(&PropulsionPlugin::OnRosWindMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
    this->rosSubWind = this->rosNode->subscribe(s1);

    //subscribe to model link states to get position and orientation for coordinate transformation
    ros::SubscribeOptions s2 =
        ros::SubscribeOptions::create<gazebo_msgs::LinkStates>(
            "/gazebo/link_states", 100, boost::bind(&PropulsionPlugin::OnlinkMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
    this->rosSubLink = this->rosNode->subscribe(s2);

    //subscribe to control signal,six global velocity for drone
    ros::SubscribeOptions s3 =
        ros::SubscribeOptions::create<flypulator_common_msgs::RotorVelStamped>(
            "/drone/rotor_cmd", 100, boost::bind(&PropulsionPlugin::OnControlMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
    this->rosSubControl = this->rosNode->subscribe(s3);

    // Spin up the queue helper thread.
    this->rosQueueThread = std::thread(std::bind(&PropulsionPlugin::QueueThread, this));

    //update world to apply constant force
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&PropulsionPlugin::OnUpdate, this, _1));

    ROS_INFO_STREAM("PropulsionPlugin Loaded !");
  }

public:
  void OnUpdate(const common::UpdateInfo &_info)
  {
    // ROS_INFO_STREAM("propulsion plugin: OnUpdate()!");
    // this->SetVelocity();
    // this->SetForce();
    // this->SetTorque();

    // publish tf base_link ---> world
    tfPublisher();
    jointStatePubliher();
    wrenchPublisher();

    if (WRITE_CSV_FILE)
    {
      // if (rotor_vel[0] < 2500 && rotor_vel[0] >= 100)
      {
        std::ofstream result_file(RESULT_CSV_PATH, std::ios::app);
        result_file.setf(std::ios::fixed, std::ios::floatfield);
        result_file.precision(5);
        result_file << this->model->GetWorld()->GetSimTime().Double() << ","
                    << test_data[0] << ","
                    << test_data[1] << ","
                    << test_data[2] << ","
                    << test_data[3] << ","
                    << test_data[4] << ","
                    << test_data[5] << ","
                    << test_data[6] << ","
                    << test_data[7] << ","
                    << test_data[8] << ","
                    << test_data[9] << ","
                    << test_data[10] << ","
                    << test_data[11] << ","
                    << std::endl;
        result_file.close();
  }
    }
  }

  //calculate aerodynamic
public:
  void OnlinkMsg(const gazebo_msgs::LinkStatesConstPtr &msg)
  {
    // ROS_INFO_STREAM("propulsion_plugin: get LinkStatesMsg!");

    // ROS_INFO_STREAM(Vx<<","<<Vx<<","<<Vx);

    double curr_time = this->model->GetWorld()->GetSimTime().Double();
    static double last_time = curr_time;
    double dt = curr_time - last_time;
    last_time = curr_time;
    // ROS_ERROR_STREAM("dt:"<<dt);
    if (dt < 0)
    {
      ROS_WARN_STREAM("Move backward in time, reset Motor!");
      motor1.reset();
      motor2.reset();
      motor3.reset();
      motor4.reset();
      motor5.reset();
      motor6.reset();
      return;
    }
    
    // motors dynamic
    if(use_motor_dynamic){
      rotor_vel_raw[0] = motor1.update(rotor_vel_cmd[0], dt);
      rotor_vel_raw[1] = motor2.update(rotor_vel_cmd[1], dt);
      rotor_vel_raw[2] = motor3.update(rotor_vel_cmd[2], dt);
      rotor_vel_raw[3] = motor4.update(rotor_vel_cmd[3], dt);
      rotor_vel_raw[4] = motor5.update(rotor_vel_cmd[4], dt);
      rotor_vel_raw[5] = motor6.update(rotor_vel_cmd[5], dt);
    }
    else{
      for(int i=0; i<6; i++)
        rotor_vel_raw[i] = rotor_vel_cmd[i];
    }

  // check direction of the rotors' velocity
  for(int i=0; i<6; i++){
    if(bidirectional){ // use bi-direction
      if (rotor_vel_raw[i] < 0){
        rotor_vel[i] = -rotor_vel_raw[i];
        di_vel[i] = -di_blade_rot[i]; //inverse rotating direction
        di_force[i] = -1;
      }
      else{
        rotor_vel[i] = rotor_vel_raw[i];
        di_vel[i] = di_blade_rot[i]; //keep default rotating direction
        di_force[i] = 1;
      }
    }
    else{ // use uni-direction
    
      if (rotor_vel_raw[i] < 0)
        rotor_vel[i] = 0;
      else
        rotor_vel[i] = rotor_vel_raw[i];

      di_vel[i] = di_blade_rot[i];
      di_force[i] = 1;
    }
  }

    // clamp rotors angular velocity
    for(int i=0; i<6; i++)
      rotor_vel[i] = clamp(rotor_vel[i], vel_min, vel_max);

    // altitude/height of the drone to the gorund
    double drone_height = this->link0->GetWorldPose().pos.z; 
  
    // TODO: only approximate ground effect, neglect the tilting angle of the drone and propeller
    // calculate ground effect coefficient, T_eff = T_nom/ground_effect_coeff
    if(drone_height > 0.35*R) // R = blade radius
      ground_effect_coeff = 1.0 - pow(R/(4*drone_height),2); // min.~0.5
    else
      ground_effect_coeff = 0.5;

    // if not using ground effect reset the coeff
    if(!use_ground_effect)
      ground_effect_coeff = 1.0;

    // ROS_INFO_STREAM("k:"<<motor1.getK()<<","<<"T:"<<motor1.getT()<<","<<"omega:"<<motor1.getOmega());

    Vdrone_x = this->model->GetRelativeLinearVel().x;
    Vdrone_y = this->model->GetRelativeLinearVel().y;
    Vdrone_z = this->model->GetRelativeLinearVel().z;
    Vx = Vwind_x - Vdrone_x;
    Vy = Vwind_y - Vdrone_y;
    Vz = Vwind_z - Vdrone_z;
    Eigen::Vector3d V_airflow (Vx, Vy, Vz);



    for (int i = 0; i < 6; i++){
        Eigen::Quaterniond q (msg->pose[2+i].orientation.w, msg->pose[2+i].orientation.x, msg->pose[2+i].orientation.y, msg->pose[2+i].orientation.z);
        Eigen::Matrix3d t_matrix = q.toRotationMatrix();
        Eigen::Matrix3d t_matrix_trans = t_matrix.transpose();
        Eigen::Vector3d v_local = t_matrix_trans * V_airflow;

        // now we are in local rotor frame {R}_i
        double v_z = v_local.z();
        double v_xy = sqrt(pow(v_local.x(), 2) + pow(v_local.y(), 2));
        double C = v_z / Vi_h;
        double v_i = 0;

        // calculate induced velocity, depends on climb ratio C 
        // TODO fix discontinuity with Bezier Spline (Hiller, App. A.1.2)
        if (C >= 0)
        {
          // Vi1 = -(sqrt(pow((Vzz1 / 2), 2) + pow(Vi_h, 2)) + Vzz1 / 2); //Vi induced airflow velocity
          v_i =  - v_z/2.0f + sqrt(pow((v_z / 2.0f), 2) + pow(Vi_h, 2));
        }
        else if (C >= -2 && C < 0)
        {
          v_i = - Vi_h * (k0 + k1 * C + k2 * pow(C, 2) + k3 * pow(C, 3) + k4 * pow(C, 4));
        }
        else
        {
          v_i = - v_z / 2 - sqrt(pow((v_z / 2), 2) - pow(Vi_h, 2));
        }

        double lambda = ( v_i + v_z ) / (rotor_vel[i] * R); // Hiller, eq. 4.27
        double mu = v_xy / (rotor_vel[i] * R); // Hiller, eq. 4.40
        double CT = s * a * ( (pa/3) * (pow(B,3) + 3*mu*mu*B/2.0 ) - lambda*B*B/2.0); // Hiller's (4.57)

        force_rotor[i].z() = di_force[i] * 0.5 * pho * CT * A * pow( rotor_vel[i] * R , 2 );

        double alpha = atan2(v_local.y(), v_local.x()); //orientation of Vxy in blade coordinate
        double f_hub = 0.25 * s * pho * A * CD0 * pow(rotor_vel[i]* R, 2) * v_xy; //H-force, Hiller eq. 4.63 & 4.64
        force_rotor[i].x() = f_hub * cos(alpha);                                    //H-force in x direction
        force_rotor[i].y() = f_hub * sin(alpha);                                    //H-force in y direction

        double CQ = ki * lambda * CT + 0.25 * s * CD0 * (1 + k * pow(mu, 2)); // Hiller's (4.62)

        torque_rotor[i].z() = 0.5 * pho * pow((rotor_vel[i] * R), 2) * A * R * CQ * di_blade_rot[i]; // Hiller's (4.60)

        double moment_roll = 0.125 * s * a * pho * R * A * mu * ((4 / 3) * th0 - thtw - lambda) * pow((rotor_vel[i] * R), 2); // Hiller eq. 4.65 and 4.66
        torque_rotor[i].x() = moment_roll * cos(alpha);
        torque_rotor[i].y() = moment_roll * sin(alpha);

        // apply to uav
        if(add_wrench_to_drone){
           if (use_simple_aerodynamic){
              rotor_link_ptr[i]->AddRelativeForce(math::Vector3(0, 0, di_force[i]*k_simple_aero*pow(rotor_vel[i],2)));
              rotor_link_ptr[i]->AddRelativeTorque(math::Vector3(0, 0, di_blade_rot[i]*b_simple_aero*pow(rotor_vel[i],2)));
           }
           else{
              rotor_link_ptr[i]->AddRelativeForce(math::Vector3(force_rotor[i].x(), force_rotor[i].y(), force_rotor[i].z()/ground_effect_coeff));
              rotor_link_ptr[i]->AddRelativeTorque(math::Vector3(torque_rotor[i].x(), torque_rotor[i].y(), torque_rotor[i].z()));
           }       
        }       

    } // end for loop

        //print to file
    if (write_data_2_file){
        streamDataToFile();
    }

    this->SetVelocity();

    // save data for csv output
    test_data[0] = rotor_vel[0]* di_vel[0];
    test_data[1] = rotor_vel[1]* di_vel[1];
    test_data[2] = rotor_vel[2]* di_vel[2];
    test_data[3] = rotor_vel[3]* di_vel[3];
    test_data[4] = rotor_vel[4]* di_vel[4];
    test_data[5] = rotor_vel[5]* di_vel[5];
    test_data[6] = torque_rotor[0].z();
    test_data[7] = torque_rotor[1].z();
    test_data[8] = torque_rotor[2].z();
    test_data[9] = torque_rotor[3].z();
    test_data[10] = torque_rotor[4].z();
    test_data[11] = torque_rotor[5].z();

    // test_data[0] = CT1;
    // test_data[1] = CT2;
    // test_data[2] = CT3;
    // test_data[3] = CT4;
    // test_data[4] = CT5;
    // test_data[5] = CT6;
    ros::spinOnce();


    /*
    double ratio1, ratio2, ratio3, ratio4, ratio5, ratio6;
    ratio1 = -3 * R * CQ1 * Sgn(this->link1->GetRelativeAngularVel().z) / (s * a * pa * pow(B, 3)) * di_force[0];
    ratio2 = -3 * R * CQ2 * Sgn(this->link2->GetRelativeAngularVel().z) / (s * a * pa * pow(B, 3)) * di_force[1];
    ratio3 = -3 * R * CQ3 * Sgn(this->link3->GetRelativeAngularVel().z) / (s * a * pa * pow(B, 3)) * di_force[2];
    ratio4 = -3 * R * CQ4 * Sgn(this->link4->GetRelativeAngularVel().z) / (s * a * pa * pow(B, 3)) * di_force[3];
    ratio5 = -3 * R * CQ5 * Sgn(this->link5->GetRelativeAngularVel().z) / (s * a * pa * pow(B, 3)) * di_force[4];
    ratio6 = -3 * R * CQ6 * Sgn(this->link6->GetRelativeAngularVel().z) / (s * a * pa * pow(B, 3)) * di_force[5];

    flypulator_common_msgs::Vector6dMsg _msg;
    _msg.x1 = ratio1;
    _msg.x2 = ratio2;
    _msg.x3 = ratio3;
    _msg.x4 = ratio4;
    _msg.x5 = ratio5;
    _msg.x6 = ratio6;
    
    this->pub_ratio.publish(_msg);

    */
    
  }


public:
  void OnRosWindMsg(const geometry_msgs::Vector3ConstPtr &_wind_msg)
  {
    Vwind_x = _wind_msg->x;
    Vwind_y = _wind_msg->y;
    Vwind_z = _wind_msg->z;
  }

public:
  void OnControlMsg(const flypulator_common_msgs::RotorVelStampedConstPtr &_msg)
  {
    // ROS_INFO_STREAM("propulsion plugin: get control message!");
    if (_msg->velocity.size() != 6)
    {
      ROS_ERROR("Dimention of rotor cmd does not match joint number!");
      return;
    }

    for (int i = 0; i < 6; i++)
            rotor_vel_cmd[i] = _msg->velocity[i];

    // ROS_INFO_STREAM("aero:"<<_msg->velocity[0]<<","<<_msg->velocity[1]<<","<<_msg->velocity[2]<<","<<_msg->velocity[3]<<","<<_msg->velocity[4]<<","<<_msg->velocity[5]);
    // ROS_INFO_STREAM("aero:"<<rotor_vel[0]<<","<<rotor_vel[1]<<","<<rotor_vel[2]<<","<<rotor_vel[3]<<","<<rotor_vel[4]<<","<<rotor_vel[5]);
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
  //return sgn information
public:
  int Sgn(const double &num)
  {
    if (num < 0)
      return -1;
    else if (num > 0)
      return 1;
    else
      return 0;
  }

public:
  double clamp(double x, double low, double high)
  {
    if(x > high)
      return high;
    if(x < low)
      return low;
    return x;
  }

  //apply velocity to joints with 3 metnods
public:
  void SetVelocity()
  {
    this->joint1->SetParam("vel", 0, rotor_vel[0] * di_vel[0]);
    this->joint2->SetParam("vel", 0, rotor_vel[1] * di_vel[1]);
    this->joint3->SetParam("vel", 0, rotor_vel[2] * di_vel[2]);
    this->joint4->SetParam("vel", 0, rotor_vel[3] * di_vel[3]);
    this->joint5->SetParam("vel", 0, rotor_vel[4] * di_vel[4]);
    this->joint6->SetParam("vel", 0, rotor_vel[5] * di_vel[5]);
    // ROS_INFO_STREAM("aero:"<<rotor_vel[0]<<","<<rotor_vel[1]<<","<<rotor_vel[2]<<","<<rotor_vel[3]<<","<<rotor_vel[4]<<","<<rotor_vel[5]);
    // ROS_INFO_STREAM("aero plugin: SetVelocity()!");
  }

  //load parameter from ROS parameter server
private:
  void readParamsFromServer()
  {
    ROS_INFO_STREAM("propulsion_plugin: loading aerodynamic parameters...");
    this->rosNode->param("rotor_number", N, N);
    this->rosNode->param("blade_chord_width", c, c);
    this->rosNode->param("blade_radius", R, R);
    this->rosNode->param("2D_lift_curve_slope", a, a);
    this->rosNode->param("profile_inclination_angle", th0, th0);
    this->rosNode->param("radial_inclination_change", thtw, thtw);
    this->rosNode->param("TLF", B, B);
    this->rosNode->param("air_density", pho, pho);
    this->rosNode->param("Profile_Drag_Coefficient", CD0, CD0);
    this->rosNode->param("rotor_axis_vertical_axis_angle", rv, rv);
    this->rosNode->param("minimal_rotor_velocity", vel_min, vel_min);
    this->rosNode->param("maximal_rotor_velocity", vel_max, vel_max);
    this->rosNode->param("default_blade1_rotation_direction", di_blade_rot[0], di_blade_rot[0]);
    this->rosNode->param("default_blade2_rotation_direction", di_blade_rot[1], di_blade_rot[1]);
    this->rosNode->param("default_blade3_rotation_direction", di_blade_rot[2], di_blade_rot[2]);
    this->rosNode->param("default_blade4_rotation_direction", di_blade_rot[3], di_blade_rot[3]);
    this->rosNode->param("default_blade5_rotation_direction", di_blade_rot[4], di_blade_rot[4]);
    this->rosNode->param("default_blade6_rotation_direction", di_blade_rot[5], di_blade_rot[5]);
    this->rosNode->param("bidirectional_optional", bidirectional, bidirectional);
    ROS_INFO_STREAM("propulsion_plugin: aerodynamic parameters loaded!");
  }

private:
  void tfPublisher()
  {
    static tf::TransformBroadcaster T_br;

    tf::Transform T_tmp;
    math::Pose pose_tmp;

    pose_tmp = this->link0->GetWorldPose();
    T_tmp.setOrigin(tf::Vector3(pose_tmp.pos.x, pose_tmp.pos.y, pose_tmp.pos.z));
    T_tmp.setRotation(tf::Quaternion(pose_tmp.rot.x, pose_tmp.rot.y, pose_tmp.rot.z, pose_tmp.rot.w));
    T_br.sendTransform(tf::StampedTransform(T_tmp, ros::Time::now(), "world", "base_link"));

    // pose_tmp = this->link1->GetRelativePose();
    // T_tmp.setOrigin(tf::Vector3(pose_tmp.pos.x, pose_tmp.pos.y, pose_tmp.pos.z));
    // T_tmp.setRotation(tf::Quaternion(pose_tmp.rot.x,pose_tmp.rot.y,pose_tmp.rot.z,pose_tmp.rot.w));
    // T_br.sendTransform(tf::StampedTransform(T_tmp, ros::Time::now(), "base_link", "blade_link_1"));

    // pose_tmp = this->link2->GetRelativePose();
    // T_tmp.setOrigin(tf::Vector3(pose_tmp.pos.x, pose_tmp.pos.y, pose_tmp.pos.z));
    // T_tmp.setRotation(tf::Quaternion(pose_tmp.rot.x,pose_tmp.rot.y,pose_tmp.rot.z,pose_tmp.rot.w));
    // T_br.sendTransform(tf::StampedTransform(T_tmp, ros::Time::now(), "base_link", "blade_link_2"));

    // pose_tmp = this->link3->GetRelativePose();
    // T_tmp.setOrigin(tf::Vector3(pose_tmp.pos.x, pose_tmp.pos.y, pose_tmp.pos.z));
    // T_tmp.setRotation(tf::Quaternion(pose_tmp.rot.x,pose_tmp.rot.y,pose_tmp.rot.z,pose_tmp.rot.w));
    // T_br.sendTransform(tf::StampedTransform(T_tmp, ros::Time::now(), "base_link", "blade_link_3"));

    // pose_tmp = this->link4->GetRelativePose();
    // T_tmp.setOrigin(tf::Vector3(pose_tmp.pos.x, pose_tmp.pos.y, pose_tmp.pos.z));
    // T_tmp.setRotation(tf::Quaternion(pose_tmp.rot.x,pose_tmp.rot.y,pose_tmp.rot.z,pose_tmp.rot.w));
    // T_br.sendTransform(tf::StampedTransform(T_tmp, ros::Time::now(), "base_link", "blade_link_4"));

    // pose_tmp = this->link5->GetRelativePose();
    // T_tmp.setOrigin(tf::Vector3(pose_tmp.pos.x, pose_tmp.pos.y, pose_tmp.pos.z));
    // T_tmp.setRotation(tf::Quaternion(pose_tmp.rot.x,pose_tmp.rot.y,pose_tmp.rot.z,pose_tmp.rot.w));
    // T_br.sendTransform(tf::StampedTransform(T_tmp, ros::Time::now(), "base_link", "blade_link_5"));

    // pose_tmp = this->link6->GetRelativePose();
    // T_tmp.setOrigin(tf::Vector3(pose_tmp.pos.x, pose_tmp.pos.y, pose_tmp.pos.z));
    // T_tmp.setRotation(tf::Quaternion(pose_tmp.rot.x,pose_tmp.rot.y,pose_tmp.rot.z,pose_tmp.rot.w));
    // T_br.sendTransform(tf::StampedTransform(T_tmp, ros::Time::now(), "base_link", "blade_link_6"));
  }
// publish joint state
private:
  void jointStatePubliher()
  {
    //publish joint state
    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.name.resize(6);
    joint_state_msg.position.resize(6);
    joint_state_msg.velocity.resize(6);
    joint_state_msg.header.stamp = ros::Time::now();
    joint_state_msg.header.frame_id = "base_link";
    joint_state_msg.name[0] = this->joint1->GetName();
    joint_state_msg.name[1] = this->joint2->GetName();
    joint_state_msg.name[2] = this->joint3->GetName();
    joint_state_msg.name[3] = this->joint4->GetName();
    joint_state_msg.name[4] = this->joint5->GetName();
    joint_state_msg.name[5] = this->joint6->GetName();

    joint_state_msg.position[0] = this->joint1->GetAngle(0).Radian();
    joint_state_msg.position[1] = this->joint2->GetAngle(0).Radian();
    joint_state_msg.position[2] = this->joint3->GetAngle(0).Radian();
    joint_state_msg.position[3] = this->joint4->GetAngle(0).Radian();
    joint_state_msg.position[4] = this->joint5->GetAngle(0).Radian();
    joint_state_msg.position[5] = this->joint6->GetAngle(0).Radian();

    pub_joint_state.publish(joint_state_msg);
  }  

private:
  void wrenchPublisher()
  {
    geometry_msgs::WrenchStamped wrench_msg_tmp;
    wrench_msg_tmp.header.stamp = ros::Time::now();

    for (int i = 0; i<6; i++){
        wrench_msg_tmp.header.frame_id = std::string("motor_link_") + std::to_string(i+1);
        wrench_msg_tmp.wrench.force.x = force_rotor[i].x();
        wrench_msg_tmp.wrench.force.y = force_rotor[i].y();
        wrench_msg_tmp.wrench.force.z = force_rotor[i].z();
        wrench_msg_tmp.wrench.torque.x = torque_rotor[i].x();
        wrench_msg_tmp.wrench.torque.y = torque_rotor[i].y();
        wrench_msg_tmp.wrench.torque.z = torque_rotor[i].z();
        this->pub_link_wrench[i].publish(wrench_msg_tmp);
    }

  }

  void streamDataToFile(){
        result_file.setf(std::ios::fixed, std::ios::floatfield);
        result_file.precision(5);
        result_file << this->model->GetWorld()->GetSimTime().Double() << ",";
        for (int i = 0; i<6; i++){
          result_file << "rotor" << i << "," << rotor_vel[i]* rotor_vel_raw[i] << ","
                      << force_rotor[i].x() << "," 
                      << force_rotor[i].y() << "," 
                      << force_rotor[i].z() << ","
                      << torque_rotor[i].x() << ","
                      << torque_rotor[i].y() << ","
                      << torque_rotor[i].z() << ",";
        }
        result_file << std::endl;            
                    
        //result_file.close();

  }

  // dynamic model of the motors
private:
  flypulator::MotorModel motor1;
  flypulator::MotorModel motor2;
  flypulator::MotorModel motor3;
  flypulator::MotorModel motor4;
  flypulator::MotorModel motor5;
  flypulator::MotorModel motor6;

  /// \brief Pointer to the model.
private:
  physics::ModelPtr model;
  // \brief A node used for transport
private:
  transport::NodePtr node;

  /// \brief A subscriber to a named topic.
private:
  transport::SubscriberPtr sub;
  /// \brief Pointer to the joint.
private:
  physics::JointWrench wrench;

private:
  physics::JointPtr joint1;

private:
  physics::JointPtr joint2;

private:
  physics::JointPtr joint3;

private:
  physics::JointPtr joint4;

private:
  physics::JointPtr joint5;

private:
  physics::JointPtr joint6;

private:
  physics::LinkPtr link0;

  /// \brief A node use for ROS transport
private:
  std::unique_ptr<ros::NodeHandle> rosNode;

private:
  event::ConnectionPtr updateConnection;
  /// \brief A ROS subscriber
private:
  ros::Subscriber rosSub;

private:
  ros::Subscriber rosSubWind;

private:
  ros::Subscriber rosSubLink;

private:
  ros::Subscriber rosSubControl;
  //private: ros::Publisher  rosPub;
  /// \brief A ROS callbackqueue that helps process messages
private:
  ros::CallbackQueue rosQueue;

  /// \brief A thread the keeps running the rosQueue
private:
  std::thread rosQueueThread;

private:
  ros::Publisher pub_ratio;
  ros::Publisher pub_joint_state;
  ros::Publisher pub_link_wrench [6];

  std::ofstream result_file; 
};



// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(PropulsionPlugin)
} // namespace gazebo

#endif /*_PROPULSION_PLUGIN_HH_*/
