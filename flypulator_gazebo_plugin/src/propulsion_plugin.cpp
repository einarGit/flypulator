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

#define PI (M_PI) 

namespace gazebo
{
/// \brief A plugin to control drone
class PropulsionPlugin : public ModelPlugin
{
  bool WRITE_CSV_FILE = false; // if save the test_data to .csv
  bool add_wrench_to_drone = true; // if add force and torque to drone in gazebo
  bool use_ground_effect = true; // if enable ground effect
  bool use_motor_dynamic = true; // if enable motor dynamic

  double test_data[12]; // test data for debug
  double ground_effect_coeff; // ground effect coefficient

  int N = 6;         //number of energie
  double c = 0.016;  //blade chord width
  double R = 0.15;   //blade radius
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
  double rv = 0.7854;     //rotor_axis_vertical_axis_angle cos(rv)=cos(pitch)*cos(yaw)
  double m;               //drone_masse
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
  double force_1, force_2, force_3, force_4, force_5, force_6;//thrust
  double moment_1, moment_2, moment_3, moment_4, moment_5, moment_6;//torque
  double fh_x1, fh_x2, fh_x3, fh_x4, fh_x5, fh_x6;
  double fh_y1, fh_y2, fh_y3, fh_y4, fh_y5, fh_y6;//H Force
  double moment_R1x, moment_R1y, moment_R2x, moment_R2y, moment_R3x, moment_R3y;
  double moment_R4x, moment_R4y, moment_R5x, moment_R5y, moment_R6x, moment_R6y; //roll moment
  double force_x, force_y, force_z, torque_x, torque_y, torque_z; //input wrench
  
  double rotor_vel_cmd[6] = {0};               // blade spinning velocity commands
  double rotor_vel[6] = {0};                   // blade spinning velocity
  int di_blade_rot[6] = {1, -1, 1, -1, 1, -1}; //default blade rotating direction 1 counterclockwise; -1 clockwise
  int di_force[6] = {1, 1, 1, 1, 1, 1};        //thrust force direction
  int di_vel[6] = {1, 1, 1, 1, 1, 1};          //real rotate direction

  bool bidirectional = false; //bidirectional option
  
  // if the rotor vel smaller than 325, we will get negativ CT and moment,
  // caused by the aero dynamic eq.(Hiller's 4.57)
  double vel_min = 350;      // min rotor speed
  double vel_max = 2500;      //max rotor speed
  //input control signal in velocity DOF 6, unused
  //double Vx_input, Vy_input, Vz_input, Wx_input, Wy_input, Wz_input;
  double Kv0 = 890;  //KV Value of the BLDC motor
  double Um0 = 10;   //nominal no-load voltage
  double Im0 = 0.5;  //nominal no-load current
  double Rm = 0.101; //resitance
                     
  Eigen::Matrix3d T_trans; //transformation matrix from global coordinate to body coordinate

  std::string RESULT_CSV_PATH = "/home/jinyao/ros_ws/flypulator/result.csv";

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
    this->link1 = _model->GetChildLink("blade_link_1");
    this->link2 = _model->GetChildLink("blade_link_2");
    this->link3 = _model->GetChildLink("blade_link_3");
    this->link4 = _model->GetChildLink("blade_link_4");
    this->link5 = _model->GetChildLink("blade_link_5");
    this->link6 = _model->GetChildLink("blade_link_6");

    //load aerodynamic parameters
    // this->readParamsFromServer();    
    
    //calculation of constants
    m = this->link0->GetInertial()->GetMass();
    s = (N * c) / (PI * R);                              //rotor solidity
    A = PI * pow(R, 2);                                  //wing area
    Vi_h = -sqrt((m * g) / (2 * N * pho * A * cos(rv))); //induced airflow velocity in hovering case
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

    this->pub_link1_wrench = this->rosNode->advertise<geometry_msgs::WrenchStamped>("/drone/blade_1_wrench", 100);
    this->pub_link2_wrench = this->rosNode->advertise<geometry_msgs::WrenchStamped>("/drone/blade_2_wrench", 100);
    this->pub_link3_wrench = this->rosNode->advertise<geometry_msgs::WrenchStamped>("/drone/blade_3_wrench", 100);
    this->pub_link4_wrench = this->rosNode->advertise<geometry_msgs::WrenchStamped>("/drone/blade_4_wrench", 100);
    this->pub_link5_wrench = this->rosNode->advertise<geometry_msgs::WrenchStamped>("/drone/blade_5_wrench", 100);
    this->pub_link6_wrench = this->rosNode->advertise<geometry_msgs::WrenchStamped>("/drone/blade_6_wrench", 100);

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
      rotor_vel[0] = motor1.update(rotor_vel_cmd[0], dt);
      rotor_vel[1] = motor2.update(rotor_vel_cmd[1], dt);
      rotor_vel[2] = motor3.update(rotor_vel_cmd[2], dt);
      rotor_vel[3] = motor4.update(rotor_vel_cmd[3], dt);
      rotor_vel[4] = motor5.update(rotor_vel_cmd[4], dt);
      rotor_vel[5] = motor6.update(rotor_vel_cmd[5], dt);
    }
    else{
      for(int i=0; i<6; i++)
        rotor_vel[i] = rotor_vel_cmd[i];
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

    //blade1 aero dynamic,Vxx1,Vyy1...airflow velocity in blade local coordinate
    double q1_1, q1_2, q1_3, q1_4, Vxx1, Vyy1, Vzz1, Vxy1, C1, Vi1;
    double CT1, l1, u1, CQ1; //aerodynamic coefficient
    double fh1, a1;          //H-force and its orientation in blade rotate xy plane
    double momentR1;         //roll moment
    q1_1 = msg->pose[2].orientation.x;
    q1_2 = msg->pose[2].orientation.y;
    q1_3 = msg->pose[2].orientation.z;
    q1_4 = msg->pose[2].orientation.w;
    Eigen::Matrix3d T1;
    Eigen::Matrix3d T1_trans;
    T1(0, 0) = pow(q1_1, 2) - pow(q1_2, 2) - pow(q1_3, 2) + pow(q1_4, 2);
    T1(0, 1) = 2 * q1_1 * q1_2 - 2 * q1_3 * q1_4;
    T1(0, 2) = 2 * q1_1 * q1_3 + 2 * q1_2 * q1_4;
    T1(1, 0) = 2 * q1_1 * q1_2 + 2 * q1_3 * q1_4;
    T1(1, 1) = -pow(q1_1, 2) + pow(q1_2, 2) - pow(q1_3, 2) + pow(q1_4, 2);
    T1(1, 2) = 2 * q1_2 * q1_3 - 2 * q1_1 * q1_4;
    T1(2, 0) = 2 * q1_1 * q1_3 - 2 * q1_2 * q1_4;
    T1(2, 1) = 2 * q1_2 * q1_3 + 2 * q1_1 * q1_4;
    T1(2, 2) = -pow(q1_1, 2) - pow(q1_2, 2) + pow(q1_3, 2) + pow(q1_4, 2);
    T1_trans = T1.transpose();
    Eigen::Vector3d V_airflow;
    Eigen::Vector3d V1_local_airflow;
    V_airflow(0) = Vx;
    V_airflow(1) = Vy;
    V_airflow(2) = Vz;
    V1_local_airflow = T1_trans * V_airflow;
    Vxx1 = V1_local_airflow(0);
    Vyy1 = V1_local_airflow(1);
    Vzz1 = V1_local_airflow(2);
    Vxy1 = sqrt(pow(Vxx1, 2) + pow(Vyy1, 2));

    C1 = Vzz1 / Vi_h;
    if (C1 >= 0)
    {
      // Vi1 = -(sqrt(pow((Vzz1 / 2), 2) + pow(Vi_h, 2)) + Vzz1 / 2); //Vi induced airflow velocity
      Vi1 =  - Vzz1/2.0 + sqrt(pow((Vzz1 / 2), 2) + pow(Vi_h, 2));
    }
    else if (C1 >= -2 && C1 < 0)
    {
      Vi1 = -Vi_h * (k0 + k1 * C1 + k2 * pow(C1, 2) + k3 * pow(C1, 3) + k4 * pow(C1, 4));
    }
    else
    {
      Vi1 = -Vzz1 / 2 - sqrt(pow((Vzz1 / 2), 2) - pow(Vi_h, 2));
    }


    l1 = (Vi1 + Vzz1) / (rotor_vel[0] * R); // inflow rate
    u1 = Vxy1 / (rotor_vel[0] * R);
    CT1 = s*a*((pa/3)*(pow(B,3)+3*u1*u1*B/2.0)-l1*B*B/2.0); // Hiller's (4.57)
    force_1 = di_force[0]*0.5* pho * CT1 * A *pow(rotor_vel[0]*R,2);
    CQ1 = ki * l1 * CT1 + 0.25 * s * CD0 * (1 + k * pow(u1, 2)); // Hiller's (4.62)
    moment_1 = 0.5 * pho * pow((rotor_vel[0] * R), 2) * A * R * CQ1 * di_blade_rot[0]; // Hiller's (4.60)
    momentR1 = 0.125 * s * a * pho * R * A * Vxy1 * (((4 / 3) * th0 - thtw) * rotor_vel[0] * R + Vi1 + Vzz1);
    if (Vxx1 >= 0)
    {
      a1 = atan(Vyy1 / Vxx1); //orientation of Vxy in blade coordinate
    }
    else
    {
      a1 = PI + atan(Vyy1 / Vxx1);
    }
    fh1 = 0.25 * s * pho * rotor_vel[0] * R * A * CD0 * Vxy1; //H-force
    fh_x1 = fh1 * cos(a1);                                    //H-force in x direction
    fh_y1 = fh1 * sin(a1);                                    //H-force in y direction
    moment_R1x = momentR1 * cos(a1);                          //roll moment in x direction
    moment_R1y = momentR1 * sin(a1);                          //roll moment in y direction
    //blade2 aerodynamic
    double q2_1, q2_2, q2_3, q2_4, Vxx2, Vyy2, Vzz2, Vxy2, C2, Vi2;
    double CT2, l2, u2, CQ2;
    double fh2, a2;
    double momentR2;
    q2_1 = msg->pose[3].orientation.x;
    q2_2 = msg->pose[3].orientation.y;
    q2_3 = msg->pose[3].orientation.z;
    q2_4 = msg->pose[3].orientation.w;
    Eigen::Matrix3d T2;
    Eigen::Matrix3d T2_trans;
    T2(0, 0) = pow(q2_1, 2) - pow(q2_2, 2) - pow(q2_3, 2) + pow(q2_4, 2);
    T2(0, 1) = 2 * q2_1 * q2_2 - 2 * q2_3 * q2_4;
    T2(0, 2) = 2 * q2_1 * q2_3 + 2 * q2_2 * q2_4;
    T2(1, 0) = 2 * q2_1 * q2_2 + 2 * q2_3 * q2_4;
    T2(1, 1) = -pow(q2_1, 2) + pow(q2_2, 2) - pow(q2_3, 2) + pow(q2_4, 2);
    T2(1, 2) = 2 * q2_2 * q2_3 - 2 * q2_1 * q2_4;
    T2(2, 0) = 2 * q2_1 * q2_3 - 2 * q2_2 * q2_4;
    T2(2, 1) = 2 * q2_2 * q2_3 + 2 * q2_1 * q2_4;
    T2(2, 2) = -pow(q2_1, 2) - pow(q2_2, 2) + pow(q2_3, 2) + pow(q2_4, 2);
    T2_trans = T2.transpose();
    Eigen::Vector3d V2_local_airflow;
    V2_local_airflow = T2_trans * V_airflow;
    Vxx2 = V2_local_airflow(0);
    Vyy2 = V2_local_airflow(1);
    Vzz2 = V2_local_airflow(2);
    Vxy2 = sqrt(pow(Vxx2, 2) + pow(Vyy2, 2));
    C2 = Vzz2 / Vi_h;
    if (C2 >= 0)
    {
      // Vi2 = -(sqrt(pow((Vzz2 / 2), 2) + pow(Vi_h, 2)) + Vzz2 / 2);
      Vi2 = -Vzz2 / 2 + sqrt(pow((Vzz2 / 2), 2) + pow(Vi_h, 2)) ;
    }
    else if (C2 >= -2 && C2 < 0)
    {
      Vi2 = -Vi_h * (k0 + k1 * C2 + k2 * pow(C2, 2) + k3 * pow(C2, 3) + k4 * pow(C2, 4));
    }
    else
    {
      // Vi2 = -Vzz2 / 2 + sqrt(pow((Vzz2 / 2), 2) - pow(Vi_h, 2));
      Vi2 = -Vzz2 / 2 - sqrt(pow((Vzz2 / 2), 2) - pow(Vi_h, 2));
    }

    if (Vxx2 >= 0)
    {
      a2 = atan(Vyy2 / Vxx2);
    }
    else
    {
      a2 = PI + atan(Vyy2 / Vxx2);
    }
    fh2 = 0.25 * s * pho * rotor_vel[1] * R * A * CD0 * Vxy2;
    fh_x2 = fh2 * cos(a2);
    fh_y2 = fh2 * sin(a2);

    l2 = (Vi2 + Vzz2) / (rotor_vel[1] * R);
    u2 = Vxy2 / (rotor_vel[1] * R);
    CT2 = s*a*((pa/3)*(pow(B,3)+3*u2*u2*B/2.0)-l2*B*B/2.0);
    force_2 = di_force[1]*0.5* pho * CT2 * A *pow(rotor_vel[1]*R,2);
    CQ2 = ki * l2 * CT2 + 0.25 * s * CD0 * (1 + k * pow(u2, 2));
    moment_2 = 0.5 * pho * pow((rotor_vel[1] * R), 2) * A * R * CQ2 * di_blade_rot[1]; 
    momentR2 = 0.125 * s * a * pho * R * A * Vxy2 * (((4 / 3) * th0 - thtw) * rotor_vel[1] * R + Vi2 + Vzz2);

    moment_R2x = momentR2 * cos(a2);
    moment_R2y = momentR2 * sin(a2);

    //blade3 aerodynamic
    double q3_1, q3_2, q3_3, q3_4, Vxx3, Vyy3, Vzz3, Vxy3, C3, Vi3;
    double CT3, l3, u3, CQ3;
    double fh3, a3;
    double momentR3;
    q3_1 = msg->pose[4].orientation.x;
    q3_2 = msg->pose[4].orientation.y;
    q3_3 = msg->pose[4].orientation.z;
    q3_4 = msg->pose[4].orientation.w;
    Eigen::Matrix3d T3;
    Eigen::Matrix3d T3_trans;
    T3(0, 0) = pow(q3_1, 2) - pow(q3_2, 2) - pow(q3_3, 2) + pow(q3_4, 2);
    T3(0, 1) = 2 * q3_1 * q3_2 - 2 * q3_3 * q3_4;
    T3(0, 2) = 2 * q3_1 * q3_3 + 2 * q3_2 * q3_4;
    T3(1, 0) = 2 * q3_1 * q3_2 + 2 * q3_3 * q3_4;
    T3(1, 1) = -pow(q3_1, 2) + pow(q3_2, 2) - pow(q3_3, 2) + pow(q3_4, 2);
    T3(1, 2) = 2 * q3_2 * q3_3 - 2 * q3_1 * q3_4;
    T3(2, 0) = 2 * q3_1 * q3_3 - 2 * q3_2 * q3_4;
    T3(2, 1) = 2 * q3_2 * q3_3 + 2 * q3_1 * q3_4;
    T3(2, 2) = -pow(q3_1, 2) - pow(q3_2, 2) + pow(q3_3, 2) + pow(q3_4, 2);
    T3_trans = T3.transpose();
    Eigen::Vector3d V3_local_airflow;
    V3_local_airflow = T3_trans * V_airflow;
    Vxx3 = V3_local_airflow(0);
    Vyy3 = V3_local_airflow(1);
    Vzz3 = V3_local_airflow(2);
    Vxy3 = sqrt(pow(Vxx3, 2) + pow(Vyy3, 2));
    C3 = Vzz3 / Vi_h;
    if (C3 >= 0)
    {
      // Vi3 = -(sqrt(pow((Vzz3 / 2), 2) + pow(Vi_h, 2)) + Vzz3 / 2);
      Vi3 = -Vzz3/2 + sqrt(pow((Vzz3 / 2), 2) + pow(Vi_h, 2));
    }
    else if (C3 >= -2 && C3 < 0)
    {
      Vi3 = -Vi_h * (k0 + k1 * C3 + k2 * pow(C3, 2) + k3 * pow(C3, 3) + k4 * pow(C3, 4));
    }
    else
    {
      // Vi3 = -Vzz3 / 2 + sqrt(pow((Vzz3 / 2), 2) - pow(Vi_h, 2));
      Vi3 = -Vzz3 / 2 - sqrt(pow((Vzz3 / 2), 2) - pow(Vi_h, 2));
    }

    // constrain the rotor spinning vel
    // rotor_vel[2] = clamp(rotor_vel[2], vel_min, vel_max);

    if (Vxx3 >= 0)
    {
      a3 = atan(Vyy3 / Vxx3);
    }
    else
    {
      a3 = PI + atan(Vyy3 / Vxx3);
    }

    fh3 = 0.25 * s * pho * rotor_vel[2] * R * A * CD0 * Vxy3;
    fh_x3 = fh3 * cos(a3);
    fh_y3 = fh3 * sin(a3);

    l3 = (Vi3 + Vzz3) / (rotor_vel[2] * R);
    u3 = Vxy3 / (rotor_vel[2] * R);
    CT3 = s*a*((pa/3)*(pow(B,3)+3*u3*u3*B/2.0)-l3*B*B/2.0);
    force_3 = di_force[2]*0.5* pho * CT3 * A *pow(rotor_vel[2]*R,2);
    CQ3 = ki * l3 * CT3 + 0.25 * s * CD0 * (1 + k * pow(u3, 2));
    moment_3 = 0.5 * pho * pow((rotor_vel[2] * R), 2) * A * R * CQ3 * di_blade_rot[2];
    momentR3 = 0.125 * s * a * pho * R * A * Vxy3 * (((4 / 3) * th0 - thtw) * rotor_vel[2] * R + Vi3 + Vzz3);

    moment_R3x = momentR3 * cos(a3);
    moment_R3y = momentR3 * sin(a3);

    //blade4 aerodynamic
    double q4_1, q4_2, q4_3, q4_4, Vxx4, Vyy4, Vzz4, Vxy4, C4, Vi4;
    double CT4, l4, u4, CQ4;
    double fh4, a4;
    double momentR4;
    q4_1 = msg->pose[5].orientation.x;
    q4_2 = msg->pose[5].orientation.y;
    q4_3 = msg->pose[5].orientation.z;
    q4_4 = msg->pose[5].orientation.w;
    Eigen::Matrix3d T4;
    Eigen::Matrix3d T4_trans;
    T4(0, 0) = pow(q4_1, 2) - pow(q4_2, 2) - pow(q4_3, 2) + pow(q4_4, 2);
    T4(0, 1) = 2 * q4_1 * q4_2 - 2 * q4_3 * q4_4;
    T4(0, 2) = 2 * q4_1 * q4_3 + 2 * q4_2 * q4_4;
    T4(1, 0) = 2 * q4_1 * q4_2 + 2 * q4_3 * q4_4;
    T4(1, 1) = -pow(q4_1, 2) + pow(q4_2, 2) - pow(q4_3, 2) + pow(q4_4, 2);
    T4(1, 2) = 2 * q4_2 * q4_3 - 2 * q4_1 * q4_4;
    T4(2, 0) = 2 * q4_1 * q4_3 - 2 * q4_2 * q4_4;
    T4(2, 1) = 2 * q4_2 * q4_3 + 2 * q4_1 * q4_4;
    T4(2, 2) = -pow(q4_1, 2) - pow(q4_2, 2) + pow(q4_3, 2) + pow(q4_4, 2);
    T4_trans = T4.transpose();
    Eigen::Vector3d V4_local_airflow;
    V4_local_airflow = T4_trans * V_airflow;
    Vxx4 = V4_local_airflow(0);
    Vyy4 = V4_local_airflow(1);
    Vzz4 = V4_local_airflow(2);
    Vxy4 = sqrt(pow(Vxx4, 2) + pow(Vyy4, 2));
    C4 = Vzz4 / Vi_h;
    if (C4 >= 0)
    {
      // Vi4 = -(sqrt(pow((Vzz4 / 2), 2) + pow(Vi_h, 2)) + Vzz4 / 2);
      Vi4 = -Vzz4/2+sqrt(pow((Vzz4 / 2), 2) + pow(Vi_h, 2));
    }
    else if (C4 >= -2 && C4 < 0)
    {
      Vi4 = -Vi_h * (k0 + k1 * C4 + k2 * pow(C4, 2) + k3 * pow(C4, 3) + k4 * pow(C4, 4));
    }
    else
    {
      // Vi4 = -Vzz4 / 2 + sqrt(pow((Vzz4 / 2), 2) - pow(Vi_h, 2));
      Vi4 = -Vzz4 / 2 - sqrt(pow((Vzz4 / 2), 2) - pow(Vi_h, 2));
    }
    
    l4 = (Vi4 + Vzz4) / (rotor_vel[3] * R);
    u4 = Vxy4 / (rotor_vel[3] * R);
    CT4 = s*a*((pa/3)*(pow(B,3)+3*u4*u4*B/2.0)-l4*B*B/2.0);
    force_4 = di_force[3]*0.5* pho * CT4 * A *pow(rotor_vel[3]*R,2);
    CQ4 = ki * l4 * CT4 + 0.25 * s * CD0 * (1 + k * pow(u4, 2));
    moment_4 = 0.5 * pho * pow((rotor_vel[3] * R), 2) * A * R * CQ4 * di_blade_rot[3];
    momentR4 = 0.125 * s * a * pho * R * A * Vxy4 * (((4 / 3) * th0 - thtw) * rotor_vel[3] * R + Vi4 + Vzz4);
    if (Vxx4 >= 0)
    {
      a4 = atan(Vyy4 / Vxx4);
    }
    else
    {
      a4 = PI + atan(Vyy4 / Vxx4);
    }
    fh4 = 0.25 * s * pho * rotor_vel[3] * R * A * CD0 * Vxy4;
    fh_x4 = fh4 * cos(a4);
    fh_y4 = fh4 * sin(a4);

    moment_R4x = momentR4 * cos(a4);
    moment_R4y = momentR4 * sin(a4);

    //blade5 aerodynamic
    double q5_1, q5_2, q5_3, q5_4, Vxx5, Vyy5, Vzz5, Vxy5, C5, Vi5;
    double CT5, l5, u5, CQ5;
    double fh5, a5;
    double momentR5;
    q5_1 = msg->pose[6].orientation.x;
    q5_2 = msg->pose[6].orientation.y;
    q5_3 = msg->pose[6].orientation.z;
    q5_4 = msg->pose[6].orientation.w;
    Eigen::Matrix3d T5;
    Eigen::Matrix3d T5_trans;
    T5(0, 0) = pow(q5_1, 2) - pow(q5_2, 2) - pow(q5_3, 2) + pow(q5_4, 2);
    T5(0, 1) = 2 * q5_1 * q5_2 - 2 * q5_3 * q5_4;
    T5(0, 2) = 2 * q5_1 * q5_3 + 2 * q5_2 * q5_4;
    T5(1, 0) = 2 * q5_1 * q5_2 + 2 * q5_3 * q5_4;
    T5(1, 1) = -pow(q5_1, 2) + pow(q5_2, 2) - pow(q5_3, 2) + pow(q5_4, 2);
    T5(1, 2) = 2 * q5_2 * q5_3 - 2 * q5_1 * q5_4;
    T5(2, 0) = 2 * q5_1 * q5_3 - 2 * q5_2 * q5_4;
    T5(2, 1) = 2 * q5_2 * q5_3 + 2 * q5_1 * q5_4;
    T5(2, 2) = -pow(q5_1, 2) - pow(q5_2, 2) + pow(q5_3, 2) + pow(q5_4, 2);
    T5_trans = T5.transpose();
    Eigen::Vector3d V5_local_airflow;
    V5_local_airflow = T5_trans * V_airflow;
    Vxx5 = V5_local_airflow(0);
    Vyy5 = V5_local_airflow(1);
    Vzz5 = V5_local_airflow(2);
    Vxy5 = sqrt(pow(Vxx5, 2) + pow(Vyy5, 2));
    C5 = Vzz5 / Vi_h;
    if (C5 >= 0)
    {
      // Vi5 = -(sqrt(pow((Vzz5 / 2), 2) + pow(Vi_h, 2)) + Vzz5 / 2);
      Vi5 = -Vzz5/2 + sqrt(pow((Vzz5 / 2), 2) + pow(Vi_h, 2)) ;
    }
    else if (C5 >= -2 && C5 < 0)
    {
      Vi5 = -Vi_h * (k0 + k1 * C5 + k2 * pow(C5, 2) + k3 * pow(C5, 3) + k4 * pow(C5, 4));
    }
    else
    {
      // Vi5 = -Vzz5 / 2 + sqrt(pow((Vzz5 / 2), 2) - pow(Vi_h, 2));
      Vi5 = -Vzz5 / 2 - sqrt(pow((Vzz5 / 2), 2) - pow(Vi_h, 2));
    }
    
    l5 = (Vi5 + Vzz5) / (rotor_vel[4] * R);
    u5 = Vxy5 / (rotor_vel[4] * R);
    CT5 = s*a*((pa/3)*(pow(B,3)+3*u5*u5*B/2.0)-l5*B*B/2.0);
    force_5 = di_force[4]*0.5* pho * CT5 * A *pow(rotor_vel[4]*R,2);
    CQ5 = ki * l5 * CT5 + 0.25 * s * CD0 * (1 + k * pow(u5, 2));
    moment_5 = 0.5 * pho * pow((rotor_vel[4] * R), 2) * A * R * CQ5 * di_blade_rot[4];
    momentR5 = 0.125 * s * a * pho * R * A * Vxy5 * (((4 / 3) * th0 - thtw) * rotor_vel[4] * R + Vi5 + Vzz5);
    if (Vxx5 >= 0)
    {
      a5 = atan(Vyy5 / Vxx5);
    }
    else
    {
      a5 = PI + atan(Vyy5 / Vxx5);
    }
    fh5 = 0.25 * s * pho * rotor_vel[4] * R * A * CD0 * Vxy5;
    fh_x5 = fh5 * cos(a5);
    fh_y5 = fh5 * sin(a5);

    moment_R5x = momentR5 * cos(a5);
    moment_R5y = momentR5 * sin(a5);

    //blade6 aerodynamic
    double q6_1, q6_2, q6_3, q6_4, Vxx6, Vyy6, Vzz6, Vxy6, C6, Vi6;
    double CT6, l6, u6, CQ6;
    double fh6, a6;
    double momentR6;
    q6_1 = msg->pose[7].orientation.x;
    q6_2 = msg->pose[7].orientation.y;
    q6_3 = msg->pose[7].orientation.z;
    q6_4 = msg->pose[7].orientation.w;
    Eigen::Matrix3d T6;
    Eigen::Matrix3d T6_trans;
    T6(0, 0) = pow(q6_1, 2) - pow(q6_2, 2) - pow(q6_3, 2) + pow(q6_4, 2);
    T6(0, 1) = 2 * q6_1 * q6_2 - 2 * q6_3 * q6_4;
    T6(0, 2) = 2 * q6_1 * q6_3 + 2 * q6_2 * q6_4;
    T6(1, 0) = 2 * q6_1 * q6_2 + 2 * q6_3 * q6_4;
    T6(1, 1) = -pow(q6_1, 2) + pow(q6_2, 2) - pow(q6_3, 2) + pow(q6_4, 2);
    T6(1, 2) = 2 * q6_2 * q6_3 - 2 * q6_1 * q6_4;
    T6(2, 0) = 2 * q6_1 * q6_3 - 2 * q6_2 * q6_4;
    T6(2, 1) = 2 * q6_2 * q6_3 + 2 * q6_1 * q6_4;
    T6(2, 2) = -pow(q6_1, 2) - pow(q6_2, 2) + pow(q6_3, 2) + pow(q6_4, 2);
    T6_trans = T6.transpose();
    Eigen::Vector3d V6_local_airflow;
    V6_local_airflow = T6_trans * V_airflow;
    Vxx6 = V6_local_airflow(0);
    Vyy6 = V6_local_airflow(1);
    Vzz6 = V6_local_airflow(2);
    Vxy6 = sqrt(pow(Vxx6, 2) + pow(Vyy6, 2));
    C6 = Vzz6 / Vi_h;
    if (C6 >= 0)
    {
      // Vi6 = -(sqrt(pow((Vzz6 / 2), 2) + pow(Vi_h, 2)) + Vzz6 / 2);
      Vi6 = -Vzz6/2 + sqrt(pow((Vzz6 / 2), 2) + pow(Vi_h, 2));
    }
    else if (C6 >= -2 && C6 < 0)
    {
      Vi6 = -Vi_h * (k0 + k1 * C6 + k2 * pow(C6, 2) + k3 * pow(C6, 3) + k4 * pow(C6, 4));
    }
    else
    {
      // Vi6 = -Vzz6 / 2 + sqrt(pow((Vzz6 / 2), 2) - pow(Vi_h, 2));
      Vi6 = -Vzz6 / 2 - sqrt(pow((Vzz6 / 2), 2) - pow(Vi_h, 2));
    }
    
    l6 = (Vi6 + Vzz6) / (rotor_vel[5] * R);
    u6 = Vxy6 / (rotor_vel[5] * R);
    CT6 = s*a*((pa/3)*(pow(B,3)+3*u6*u6*B/2.0)-l6*B*B/2.0);
    force_6 = di_force[5]*0.5* pho * CT6 * A *pow(rotor_vel[5]*R,2);
    CQ6 = ki * l6 * CT6 + 0.25 * s * CD0 * (1 + k * pow(u6, 2));
    moment_6 = 0.5 * pho * pow((rotor_vel[5] * R), 2) * A * R * CQ6 * di_blade_rot[5];
    momentR6 = 0.125 * s * a * pho * R * A * Vxy6 * (((4 / 3) * th0 - thtw) * rotor_vel[5] * R + Vi6 + Vzz6);
    if (Vxx6 >= 0)
    {
      a6 = atan(Vyy6 / Vxx6);
    }
    else
    {
      a6 = PI + atan(Vyy6 / Vxx6);
    }
    fh6 = 0.25 * s * pho * rotor_vel[5] * R * A * CD0 * Vxy6;
    fh_x6 = fh6 * cos(a6);
    fh_y6 = fh6 * sin(a6);

    moment_R6x = momentR6 * cos(a6);
    moment_R6y = momentR6 * sin(a6);
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

    if(add_wrench_to_drone){
      this->SetForce();
      this->SetTorque();
    }
    this->SetVelocity();

    // save data for csv output
    test_data[0] = rotor_vel[0]* di_vel[0];
    test_data[1] = rotor_vel[1]* di_vel[1];
    test_data[2] = rotor_vel[2]* di_vel[2];
    test_data[3] = rotor_vel[3]* di_vel[3];
    test_data[4] = rotor_vel[4]* di_vel[4];
    test_data[5] = rotor_vel[5]* di_vel[5];
    test_data[6] = moment_1;
    test_data[7] = moment_2;
    test_data[8] = moment_3;
    test_data[9] = moment_4;
    test_data[10] = moment_5;
    test_data[11] = moment_6;

    // test_data[0] = CT1;
    // test_data[1] = CT2;
    // test_data[2] = CT3;
    // test_data[3] = CT4;
    // test_data[4] = CT5;
    // test_data[5] = CT6;
    ros::spinOnce();
  }

public:
  void OnRosWindMsg(const geometry_msgs::Vector3ConstPtr &_wind_msg)
  {
    Vwind_x = _wind_msg->x;
    Vwind_y = _wind_msg->y;
    Vwind_z = _wind_msg->z;
    // Vdrone_x = this->model->GetRelativeLinearVel().x;
    // Vdrone_y = this->model->GetRelativeLinearVel().y;
    // Vdrone_z = this->model->GetRelativeLinearVel().z;
    // Vx = Vwind_x - Vdrone_x;
    // Vy = Vwind_y - Vdrone_y;
    // Vz = Vwind_z - Vdrone_z;
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

    //TODO: di_vel and di_force should change after motor dynamic, not here
    if (bidirectional) //bi-directional rotors
    {
      for (int i = 0; i < 6; i++)
      {
        if (_msg->velocity[i] < 0)
        {
          rotor_vel_cmd[i] = abs(_msg->velocity[i]);
          di_vel[i] = -di_blade_rot[i]; //inverse rotating direction
          di_force[i] = -1;
        }
        else
        {
          rotor_vel_cmd[i] = _msg->velocity[i];
          di_vel[i] = di_blade_rot[i]; //keep default rotating direction
          di_force[i] = 1;
        }
      }
    }
    else //uni-directional rotors
    {
      for (int i = 0; i < 6; i++)
      {
        if (_msg->velocity[i] < 0)
        {
          rotor_vel_cmd[i] = 0; //lower boundary
        }
        else
        {
          rotor_vel_cmd[i] = _msg->velocity[i];
        }
        di_vel[i] = di_blade_rot[i];
        di_force[i] = 1;
      }
    }
    // ROS_INFO_STREAM("aero:"<<_msg->velocity[0]<<","<<_msg->velocity[1]<<","<<_msg->velocity[2]<<","<<_msg->velocity[3]<<","<<_msg->velocity[4]<<","<<_msg->velocity[5]);
    // ROS_INFO_STREAM("aero:"<<rotor_vel[0]<<","<<rotor_vel[1]<<","<<rotor_vel[2]<<","<<rotor_vel[3]<<","<<rotor_vel[4]<<","<<rotor_vel[5]);
    // static double vel_temp = 100;
    // for (int i=0; i<6; i++)
    // {
    //   rotor_vel[i] = vel_temp;
    //   di_vel[i] = di_blade_rot[i];
    //   di_force[i] = 1;
    // }
    // vel_temp += 10;
    // if(vel_temp > 2500)
    //   vel_temp = 2500;
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
  //add force to blade link
public:
  void SetForce()
  {
    // this->link1->AddRelativeForce(math::Vector3(0, 0, 0));
    // this->link2->AddRelativeForce(math::Vector3(0, 0, 0));
    // this->link3->AddRelativeForce(math::Vector3(0, 0, 0));
    // this->link4->AddRelativeForce(math::Vector3(0, 0, 0));
    // this->link5->AddRelativeForce(math::Vector3(0, 0, 0));
    // this->link6->AddRelativeForce(math::Vector3(0, 0, 0));
    this->link1->AddRelativeForce(math::Vector3(fh_x1, fh_y1, force_1/ground_effect_coeff));
    this->link2->AddRelativeForce(math::Vector3(fh_x2, fh_y2, force_2/ground_effect_coeff));
    this->link3->AddRelativeForce(math::Vector3(fh_x3, fh_y3, force_3/ground_effect_coeff));
    this->link4->AddRelativeForce(math::Vector3(fh_x4, fh_y4, force_4/ground_effect_coeff));
    this->link5->AddRelativeForce(math::Vector3(fh_x5, fh_y5, force_5/ground_effect_coeff));
    this->link6->AddRelativeForce(math::Vector3(fh_x6, fh_y6, force_6/ground_effect_coeff));
    // ROS_INFO_STREAM("force:"<<force_1<<","<<force_2<<","<<force_3<<","<<force_4<<","<<force_5<<","<<force_6);
  }
  //add torque to blade link
public:
  void SetTorque()
  {
    // this->link1->AddRelativeTorque(math::Vector3(0, 0, 0));
    // this->link2->AddRelativeTorque(math::Vector3(0, 0, 0));
    // this->link3->AddRelativeTorque(math::Vector3(0, 0, 0));
    // this->link4->AddRelativeTorque(math::Vector3(0, 0, 0));
    // this->link5->AddRelativeTorque(math::Vector3(0, 0, 0));
    // this->link6->AddRelativeTorque(math::Vector3(0, 0, 0));
    this->link1->AddRelativeTorque(math::Vector3(moment_R1x, moment_R1y, moment_1));
    this->link2->AddRelativeTorque(math::Vector3(moment_R2x, moment_R2y, moment_2));
    this->link3->AddRelativeTorque(math::Vector3(moment_R3x, moment_R3y, moment_3));
    this->link4->AddRelativeTorque(math::Vector3(moment_R4x, moment_R4y, moment_4));
    this->link5->AddRelativeTorque(math::Vector3(moment_R5x, moment_R5y, moment_5));
    this->link6->AddRelativeTorque(math::Vector3(moment_R6x, moment_R6y, moment_6));
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
    this->rosNode->param("KV_value", Kv0, Kv0);
    this->rosNode->param("nominal_no_load_voltage", Um0, Um0);
    this->rosNode->param("nominal_no_load_current", Im0, Im0);
    this->rosNode->param("motor_resitance", Rm, Rm);
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

    wrench_msg_tmp.header.frame_id = "motor_link_1";
    wrench_msg_tmp.wrench.force.x = fh_x1;
    wrench_msg_tmp.wrench.force.y = fh_y1;
    wrench_msg_tmp.wrench.force.z = force_1;
    wrench_msg_tmp.wrench.torque.x = moment_R1x;
    wrench_msg_tmp.wrench.torque.y = moment_R1y;
    wrench_msg_tmp.wrench.torque.z = moment_1;
    this->pub_link1_wrench.publish(wrench_msg_tmp);
    wrench_msg_tmp.header.frame_id = "motor_link_2";
    wrench_msg_tmp.wrench.force.x = fh_x2;
    wrench_msg_tmp.wrench.force.y = fh_y2;
    wrench_msg_tmp.wrench.force.z = force_2;
    wrench_msg_tmp.wrench.torque.x = moment_R2x;
    wrench_msg_tmp.wrench.torque.y = moment_R2y;
    wrench_msg_tmp.wrench.torque.z = moment_2;
    this->pub_link2_wrench.publish(wrench_msg_tmp);
    wrench_msg_tmp.header.frame_id = "motor_link_3";
    wrench_msg_tmp.wrench.force.x = fh_x3;
    wrench_msg_tmp.wrench.force.y = fh_y3;
    wrench_msg_tmp.wrench.force.z = force_3;
    wrench_msg_tmp.wrench.torque.x = moment_R3x;
    wrench_msg_tmp.wrench.torque.y = moment_R3y;
    wrench_msg_tmp.wrench.torque.z = moment_3;
    this->pub_link3_wrench.publish(wrench_msg_tmp);
    wrench_msg_tmp.header.frame_id = "motor_link_4";
    wrench_msg_tmp.wrench.force.x = fh_x4;
    wrench_msg_tmp.wrench.force.y = fh_y4;
    wrench_msg_tmp.wrench.force.z = force_4;
    wrench_msg_tmp.wrench.torque.x = moment_R4x;
    wrench_msg_tmp.wrench.torque.y = moment_R4y;
    wrench_msg_tmp.wrench.torque.z = moment_4;
    this->pub_link4_wrench.publish(wrench_msg_tmp);
    wrench_msg_tmp.header.frame_id = "motor_link_5";
    wrench_msg_tmp.wrench.force.x = fh_x5;
    wrench_msg_tmp.wrench.force.y = fh_y5;
    wrench_msg_tmp.wrench.force.z = force_5;
    wrench_msg_tmp.wrench.torque.x = moment_R5x;
    wrench_msg_tmp.wrench.torque.y = moment_R5y;
    wrench_msg_tmp.wrench.torque.z = moment_5;
    this->pub_link5_wrench.publish(wrench_msg_tmp);
    wrench_msg_tmp.header.frame_id = "motor_link_6";
    wrench_msg_tmp.wrench.force.x = fh_x6;
    wrench_msg_tmp.wrench.force.y = fh_y6;
    wrench_msg_tmp.wrench.force.z = force_6;
    wrench_msg_tmp.wrench.torque.x = moment_R6x;
    wrench_msg_tmp.wrench.torque.y = moment_R6y;
    wrench_msg_tmp.wrench.torque.z = moment_6;
    this->pub_link6_wrench.publish(wrench_msg_tmp);
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

private:
  physics::LinkPtr link1;

private:
  physics::LinkPtr link2;

private:
  physics::LinkPtr link3;

private:
  physics::LinkPtr link4;

private:
  physics::LinkPtr link5;

private:
  physics::LinkPtr link6;
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
  ros::Publisher pub_link1_wrench;
  ros::Publisher pub_link2_wrench;
  ros::Publisher pub_link3_wrench;
  ros::Publisher pub_link4_wrench;
  ros::Publisher pub_link5_wrench;
  ros::Publisher pub_link6_wrench;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(PropulsionPlugin)
} // namespace gazebo

#endif /*_PROPULSION_PLUGIN_HH_*/
