#ifndef _CONTROL_PLUGIN_HH_
#define _CONTROL_PLUGIN_HH_

#include <iostream>
#include <math.h>
#include <thread>
#include <fstream>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/common/Time.hh>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/gazebo_client.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <geometry_msgs/Wrench.h>

#include <Eigen/Dense>

#include <flypulator_common_msgs/Vector6dMsg.h>
#include <flypulator_common_msgs/RotorVelStamped.h>


#define pi (M_PI) // TODO: rename pi to PI
#define RAD2DEG (57.29577951)

namespace gazebo
{
/// \brief A plugin to control drone
class ControlPlugin : public ModelPlugin
{
    int N = 6;                     //number of energie
    double c = 0.016;              //blade chord width
    double R = 0.15;               //blade radius
    double a = 5.7;                //2D_lift_curve_slope
    double th0 = 0.7;              //Profile inclination angle
    double thtw = 0.5;             //Inclination change along radius
    double pa = th0 - 0.75 * thtw; //blade pitch angle
    double B = 0.98;               //tip loss factor
    double pho = 1.2;              //air density
    double A = pi * pow(R, 2);     //rotor aero
    double ki = 1.15;              //factor to calculate torque
    double k = 4.65;               //factor to calculate torque
    //parameters to calculate induced velocity Vi
    double k0 = 1.15;
    double k1 = -1.125;
    double k2 = -1.372;
    double k3 = -1.718;
    double k4 = -0.655;
    double CD0 = 0.04;  //Profile_Drag_Coefficient from literatur
    double rv = 0.7854; //rotor_axis_vertical_axis_angle cos(rv)=cos(pitch)*cos(yaw)
    double m;           //drone_masse
    double g = 9.81;    //gravity acceleration constant
    double s;           //rotor solidity
    double Vwind_x = 1e-20;     //wind velocity in global x
    double Vwind_y = 1e-20;     //wind velocity in global y
    double Vwind_z = 1e-20;     //wind velocity in global z
    double Vdrone_x;    //drone velocity in global x
    double Vdrone_y;    //drone velocity in global y
    double Vdrone_z;    //drone velocity in global z
    double Vx = 1e-20;          //air velocity in global x
    double Vy = 1e-20;          //air velocity in global y
    double Vz = 1e-20;          //air velocity in global z
    double Vi_h;        //induced velocity for the hovering case
                        //calculated blade rotate velocity with a small original value to avoid Gazebo crash
    double vel_1 = 0;
    double vel_2 = 0;
    double vel_3 = 0;
    double vel_4 = 0;
    double vel_5 = 0;
    double vel_6 = 0;
    double force_x, force_y, force_z, torque_x, torque_y, torque_z; //input wrench
    double ratio1 = 0;
    double ratio2 = 0;
    double ratio3 = 0;
    double ratio4 = 0;
    double ratio5 = 0;
    double ratio6 = 0;
    //real rotate direction
    int di_vel1;
    int di_vel2;
    int di_vel3;
    int di_vel4;
    int di_vel5;
    int di_vel6;
    //input control signal in velocity DOF 6
    double Wx_input = 0; // target roll in deg.
    double Wy_input = 0; // target pitch
    double Wz_input = 0; // target yaw
    // target position
    // if no joystick, then use these inputs as default
    double Vx_input = 0; 
    double Vy_input = 0;
    double Vz_input = 1.5;
    double _Kp, _Ki, _Kd, _pid_max, _pid_min, dt;

    //PID controller parameters
    //PID for line velocity
    double linear_p = 3.0;     //The proportional gain
    double linear_i = 0.4;     //The integral gain
    double linear_d = 0.011;     //The derivative gain
    double linear_imax = 50.0; //The integral upper limit
    //PID for angular velocity
    double angular_p = 5.00;
    double angular_i = 0.0;
    double angular_d = 0.01;
    double angular_imax = 50.0;
    double _maxForce = 1000;  //Output max value
    double _maxTorque = 1000; //Output max value
                              //transformation matrix from global coordinate to body coordinate
    Eigen::Matrix3d T_trans;
    /// \brief Constructor
  public:
    ControlPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
  public:
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        if (_model->GetJointCount() == 0)
        {
            std::cerr << "Invalid joint count, control plugin not loaded\n";
            return;
        }

        // Store the model pointer for convenience.
        this->model = _model;
        // Get the first joint. We are making an assumption about the model
        // having six joints that is the rotational joint.
        this->joint1 = _model->GetJoint("blade_joint1");
        this->joint2 = _model->GetJoint("blade_joint2");
        this->joint3 = _model->GetJoint("blade_joint3");
        this->joint4 = _model->GetJoint("blade_joint4");
        this->joint5 = _model->GetJoint("blade_joint5");
        this->joint6 = _model->GetJoint("blade_joint6");
        //get the six blade link
        this->link0 = _model->GetChildLink("base_link");
        this->link1 = _model->GetChildLink("blade_Link1");
        this->link2 = _model->GetChildLink("blade_Link2");
        this->link3 = _model->GetChildLink("blade_Link3");
        this->link4 = _model->GetChildLink("blade_Link4");
        this->link5 = _model->GetChildLink("blade_Link5");
        this->link6 = _model->GetChildLink("blade_Link6");
        //calculation of constants
        m = this->link0->GetInertial()->GetMass();
        s = (N * c) / (pi * R);                              //rotor solidity
        Vi_h = -sqrt((m * g) / (2.0 * N * pho * A * cos(rv))); //induced airflow velocity by hover case
        // Initialize ros, if it has not already bee initialized.
        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "gazebo_control",
                      ros::init_options::NoSigintHandler);
            ROS_WARN_STREAM("Create gazebo_control node");
        }

        // Create our ROS node.
        this->rosNode.reset(new ros::NodeHandle("gazebo_control"));
        ROS_INFO_STREAM("control_plugin get node:"<<this->rosNode->getNamespace());

        // TODO: load parameters from .yaml
        // this->readParamsFromServer(); //read parameters from rosserver,maximal 3 in my computer

        // Add a PID controller for each DoF
        common::PID controller_translation_xy(linear_p, linear_i, linear_d,
                                               linear_imax, -linear_imax, _maxForce, -_maxForce);
        common::PID controller_rotation_xy(angular_p, angular_i, angular_d,
                                            angular_imax, -angular_imax, _maxTorque, -_maxTorque);
        this->controllers.push_back(controller_translation_xy);
        this->controllers.push_back(controller_rotation_xy);
        this->controllers.push_back(controller_translation_xy);
        this->controllers.push_back(controller_rotation_xy);

        common::PID controller_translation_z(linear_p, linear_i, linear_d,
                                               linear_imax, -linear_imax, _maxForce, -_maxForce);
        common::PID controller_rotation_z(angular_p, angular_i, angular_d,
                                            angular_imax, -angular_imax, _maxTorque, -_maxTorque);
        this->controllers.push_back(controller_translation_z);
        this->controllers.push_back(controller_rotation_z);

        // publish rotor command (velocity of each propeller)
        this->pub_cmd = this->rosNode->advertise<flypulator_common_msgs::RotorVelStamped>("/drone/rotor_cmd", 100);

        //Create a wind velocity topic and subscribe to it
        ros::SubscribeOptions s1 =
            ros::SubscribeOptions::create<geometry_msgs::Vector3>(
                "/drone/wind_cmd",100, boost::bind(&ControlPlugin::OnRosWindMsg, this, _1),
                ros::VoidPtr(), &this->rosQueue);
        this->rosSubWind = this->rosNode->subscribe(s1);

        //subscribe to model link states to get position and orientation for coordinate transformation
        ros::SubscribeOptions s2 =
            ros::SubscribeOptions::create<gazebo_msgs::LinkStates>(
                "/gazebo/link_states",100,boost::bind(&ControlPlugin::OnlinkMsg, this, _1),
                ros::VoidPtr(), &this->rosQueue);
        this->rosSubLink = this->rosNode->subscribe(s2);

        //subscribe to control signal,six global velocity for drone
        ros::SubscribeOptions s3 =
            ros::SubscribeOptions::create<geometry_msgs::Wrench>(
                "/drone/vel_cmd",100,boost::bind(&ControlPlugin::OnControlMsg, this, _1),
                ros::VoidPtr(), &this->rosQueue);
        this->rosSubControl = this->rosNode->subscribe(s3);

        ros::SubscribeOptions s4 =
            ros::SubscribeOptions::create<flypulator_common_msgs::Vector6dMsg>(
                "/drone/thrust_moment_ratio",100,boost::bind(&ControlPlugin::OnRosRatioMsg, this, _1),
                ros::VoidPtr(), &this->rosQueue);
        this->rosSubRatio = this->rosNode->subscribe(s4);

        this->rosQueueThread = std::thread(std::bind(&ControlPlugin::QueueThread, this));

        // Spin up the queue helper thread.
        //update world to apply constant force
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&ControlPlugin::OnUpdate, this, _1));

        ROS_INFO_STREAM("ControlPlugin Loaded !");
    }

  public:
    void OnUpdate(const common::UpdateInfo &_info)
    {
        double modelLinearVel_x, modelLinearVel_y, modelLinearVel_z, modelAngularVel_x, modelAngularVel_y, modelAngularVel_z;
        double linearError_x, linearError_y, linearError_z, angularError_x, angularError_y, angularError_z;
        
        //get inertial in body coordinate
        double Jxx = this->link0->GetInertial()->GetIXX();
        double Jyy = this->link0->GetInertial()->GetIYY();
        double Jzz = this->link0->GetInertial()->GetIZZ();

        math::Pose model_pose = this->link0->GetWorldPose();
        math::Vector3 model_pose_trans = model_pose.pos;
        math::Quaternion model_pose_q = model_pose.rot;
        math::Vector3 model_pose_rpy = model_pose_q.GetAsEuler();

        float model_roll = model_pose_rpy.x * RAD2DEG;
        float model_pitch = model_pose_rpy.y * RAD2DEG;
        float model_yaw = model_pose_rpy.z * RAD2DEG;

        //get robot global linear velocity
        modelLinearVel_x = this->link0->GetWorldLinearVel().x;
        modelLinearVel_y = this->link0->GetWorldLinearVel().y;
        modelLinearVel_z = this->link0->GetWorldLinearVel().z;
        //get robot global angular velocity
        modelAngularVel_x = this->link0->GetWorldAngularVel().x;
        modelAngularVel_y = this->link0->GetWorldAngularVel().y;
        modelAngularVel_z = this->link0->GetWorldAngularVel().z;
        //get error for PID controller
        linearError_x = (modelLinearVel_x - (0 - model_pose_trans.x));
        linearError_y = (modelLinearVel_y - (0 - model_pose_trans.y));
        linearError_z = (modelLinearVel_z - (Vz_input - model_pose_trans.z));
        // linearError_z = (modelLinearVel_z - (3.0 - model_pose_trans.z));
        angularError_x = (modelAngularVel_x - (Wx_input-model_roll) );
        angularError_y = (modelAngularVel_y - (Wy_input-model_pitch) );
        angularError_z = (modelAngularVel_z - (Wz_input-model_yaw) );
        
        // Get change in time between updates
        common::Time curTime = _info.simTime;
        dt = (curTime - lastSimTime).Double();
        // ROS_INFO_STREAM("dt:"<<dt);
        double a_x = this->controllers[0].Update(linearError_x, dt);
        double Wa_x = this->controllers[1].Update(angularError_x, dt);
        double a_y = this->controllers[2].Update(linearError_y, dt);
        double Wa_y = this->controllers[3].Update(angularError_y, dt);
        double a_z = this->controllers[4].Update(linearError_z, dt);
        double Wa_z = this->controllers[5].Update(angularError_z, dt);

        Eigen::Vector3d lin_acc_global, lin_acc_body, angle_acc_global, angle_acc_body;
        lin_acc_global(0) = a_x;
        lin_acc_global(1) = a_y;
        lin_acc_global(2) = a_z;
        //transforming acceleration from global coordinate in body coordinate
        lin_acc_body = T_trans * lin_acc_global;
        // lin_acc_body = lin_acc_global;
        angle_acc_global(0) = Wa_x;
        angle_acc_global(1) = Wa_y;
        angle_acc_global(2) = Wa_z;
        // angle_acc_body = T_trans * angle_acc_global;
        angle_acc_body = angle_acc_global;
        //force=masse*acceleration torque=J*Wa
        force_x = m * lin_acc_body(0);
        force_y = m * lin_acc_body(1);
        // force_x = 0;
        // force_y = 0;
        force_z = m * lin_acc_body(2);
        // force_z = -m * 9.81*0.01;
        torque_x = -Jxx * angle_acc_body(0);
        torque_y = -Jyy * angle_acc_body(1);
        torque_z = -Jzz * angle_acc_body(2);
        // torque_x = -Jxx * 0;
        // torque_y = -Jyy * 8;
        // torque_z = -Jzz * 0;
        lastSimTime = curTime;
    }

    //calculate aerodynamic
  public:
    void OnlinkMsg(const gazebo_msgs::LinkStatesConstPtr &msg)
    {
        // ROS_INFO_STREAM(Vx<<","<<Vx<<","<<Vx);
        //blade1 aero dynamic,Vxx1,Vyy1...airflow velocity in blade local coordinate
        double q1_1, q1_2, q1_3, q1_4, Vxx1, Vyy1, Vzz1, Vxy1, C1, Vi1;
        double CT1, l1, u1, CQ1; //aerodynamic coefficient
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
            Vi1 = -(sqrt(pow((Vzz1 / 2), 2) + pow(Vi_h, 2)) + Vzz1 / 2); //Vi induced airflow velocity
        }
        else if (C1 >= -2 && C1 < 0)
        {
            Vi1 = Vi_h * (k0 + k1 * C1 + k2 * pow(C1, 2) + k3 * pow(C1, 3) + k4 * pow(C1, 4));
        }
        else
        {
            Vi1 = -Vzz1 / 2 + sqrt(pow((Vzz1 / 2), 2) - pow(Vi_h, 2));
        }

        //blade2 aerodynamic
        double q2_1, q2_2, q2_3, q2_4, Vxx2, Vyy2, Vzz2, Vxy2, C2, Vi2;
        double CT2, l2, u2, CQ2;
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
            Vi2 = -(sqrt(pow((Vzz2 / 2), 2) + pow(Vi_h, 2)) + Vzz2 / 2);
        }
        else if (C2 >= -2 && C2 < 0)
        {
            Vi2 = -Vi_h * (k0 + k1 * C2 + k2 * pow(C2, 2) + k3 * pow(C2, 3) + k4 * pow(C2, 4));
        }
        else
        {
            Vi2 = -Vzz2 / 2 + sqrt(pow((Vzz2 / 2), 2) - pow(Vi_h, 2));
        }

        //blade3 aerodynamic
        double q3_1, q3_2, q3_3, q3_4, Vxx3, Vyy3, Vzz3, Vxy3, C3, Vi3;
        double CT3, l3, u3, CQ3;
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
            Vi3 = -(sqrt(pow((Vzz3 / 2), 2) + pow(Vi_h, 2)) + Vzz3 / 2);
        }
        else if (C3 >= -2 && C3 < 0)
        {
            Vi3 = -Vi_h * (k0 + k1 * C3 + k2 * pow(C3, 2) + k3 * pow(C3, 3) + k4 * pow(C3, 4));
        }
        else
        {
            Vi3 = -Vzz3 / 2 + sqrt(pow((Vzz3 / 2), 2) - pow(Vi_h, 2));
        }

        //blade4 aerodynamic
        double q4_1, q4_2, q4_3, q4_4, Vxx4, Vyy4, Vzz4, Vxy4, C4, Vi4;
        double CT4, l4, u4, CQ4;
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
            Vi4 = -(sqrt(pow((Vzz4 / 2), 2) + pow(Vi_h, 2)) + Vzz4 / 2);
        }
        else if (C4 >= -2 && C4 < 0)
        {
            Vi4 = -Vi_h * (k0 + k1 * C4 + k2 * pow(C4, 2) + k3 * pow(C4, 3) + k4 * pow(C4, 4));
        }
        else
        {
            Vi4 = -Vzz4 / 2 + sqrt(pow((Vzz4 / 2), 2) - pow(Vi_h, 2));
        }

        //blade5 aerodynamic
        double q5_1, q5_2, q5_3, q5_4, Vxx5, Vyy5, Vzz5, Vxy5, C5, Vi5;
        double CT5, l5, u5, CQ5;

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
            Vi5 = -(sqrt(pow((Vzz5 / 2), 2) + pow(Vi_h, 2)) + Vzz5 / 2);
        }
        else if (C5 >= -2 && C5 < 0)
        {
            Vi5 = -Vi_h * (k0 + k1 * C5 + k2 * pow(C5, 2) + k3 * pow(C5, 3) + k4 * pow(C5, 4));
        }
        else
        {
            Vi5 = -Vzz5 / 2 + sqrt(pow((Vzz5 / 2), 2) - pow(Vi_h, 2));
        }
        //blade6 aerodynamic
        double q6_1, q6_2, q6_3, q6_4, Vxx6, Vyy6, Vzz6, Vxy6, C6, Vi6;
        double CT6, l6, u6, CQ6;
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
            Vi6 = -(sqrt(pow((Vzz6 / 2), 2) + pow(Vi_h, 2)) + Vzz6 / 2);
        }
        else if (C6 >= -2 && C6 < 0)
        {
            Vi6 = -Vi_h * (k0 + k1 * C6 + k2 * pow(C6, 2) + k3 * pow(C6, 3) + k4 * pow(C6, 4));
        }
        else
        {
            Vi6 = -Vzz6 / 2 + sqrt(pow((Vzz6 / 2), 2) - pow(Vi_h, 2));
        }

        //Algorithmics of Geometry trensformation
        double q_1, q_2, q_3, q_4;
        q_1 = msg->pose[1].orientation.x;
        q_2 = msg->pose[1].orientation.y;
        q_3 = msg->pose[1].orientation.z;
        q_4 = msg->pose[1].orientation.w;
        Eigen::MatrixXd T(3, 3);
        T(0, 0) = pow(q_1, 2) - pow(q_2, 2) - pow(q_3, 2) + pow(q_4, 2);
        T(0, 1) = 2 * q_1 * q_2 - 2 * q_3 * q_4;
        T(0, 2) = 2 * q_1 * q_3 + 2 * q_2 * q_4;
        T(1, 0) = 2 * q_1 * q_2 + 2 * q_3 * q_4;
        T(1, 1) = -pow(q_1, 2) + pow(q_2, 2) - pow(q_3, 2) + pow(q_4, 2);
        T(1, 2) = 2 * q_2 * q_3 - 2 * q_1 * q_4;
        T(2, 0) = 2 * q_1 * q_3 - 2 * q_2 * q_4;
        T(2, 1) = 2 * q_2 * q_3 + 2 * q_1 * q_4;
        T(2, 2) = -pow(q_1, 2) - pow(q_2, 2) + pow(q_3, 2) + pow(q_4, 2);
        T_trans = T.transpose();
        Eigen::Vector3d r1, r2, r3, r4, r5, r6, coc0, coc1, coc2, coc3, coc4, coc5, coc6, ui1, ui2, ui3, ui4, ui5, ui6;
        //get the position of center oc mass
        coc0(0) = this->link0->GetWorldCoGPose().pos.x;
        coc0(1) = this->link0->GetWorldCoGPose().pos.y;
        coc0(2) = this->link0->GetWorldCoGPose().pos.z;
        coc1(0) = this->link1->GetWorldCoGPose().pos.x;
        coc1(1) = this->link1->GetWorldCoGPose().pos.y;
        coc1(2) = this->link1->GetWorldCoGPose().pos.z;
        coc2(0) = this->link2->GetWorldCoGPose().pos.x;
        coc2(1) = this->link2->GetWorldCoGPose().pos.y;
        coc2(2) = this->link2->GetWorldCoGPose().pos.z;
        coc3(0) = this->link3->GetWorldCoGPose().pos.x;
        coc3(1) = this->link3->GetWorldCoGPose().pos.y;
        coc3(2) = this->link3->GetWorldCoGPose().pos.z;
        coc4(0) = this->link4->GetWorldCoGPose().pos.x;
        coc4(1) = this->link4->GetWorldCoGPose().pos.y;
        coc4(2) = this->link4->GetWorldCoGPose().pos.z;
        coc5(0) = this->link5->GetWorldCoGPose().pos.x;
        coc5(1) = this->link5->GetWorldCoGPose().pos.y;
        coc5(2) = this->link5->GetWorldCoGPose().pos.z;
        coc6(0) = this->link6->GetWorldCoGPose().pos.x;
        coc6(1) = this->link6->GetWorldCoGPose().pos.y;
        coc6(2) = this->link6->GetWorldCoGPose().pos.z;
        //distance vector in global coordinate
        r1 = -coc1 + coc0;
        r2 = -coc2 + coc0;
        r3 = -coc3 + coc0;
        r4 = -coc4 + coc0;
        r5 = -coc5 + coc0;
        r6 = -coc6 + coc0;
        //distance vector in body coordinate
        Eigen::Vector3d r1_B, r2_B, r3_B, r4_B, r5_B, r6_B;
        r1_B = T_trans * r1;
        r2_B = T_trans * r2;
        r3_B = T_trans * r3;
        r4_B = T_trans * r4;
        r5_B = T_trans * r5;
        r6_B = T_trans * r6;
        //unit force vector from local coordinate to global coordinate
        Eigen::Vector3d V_local_force_vector(0, 0, 1);
        ui1 = T1 * V_local_force_vector;
        ui2 = T2 * V_local_force_vector;
        ui3 = T3 * V_local_force_vector;
        ui4 = T4 * V_local_force_vector;
        ui5 = T5 * V_local_force_vector;
        ui6 = T6 * V_local_force_vector;
        //Transformation from global frame to body frame
        Eigen::Vector3d ui1_B, ui2_B, ui3_B, ui4_B, ui5_B, ui6_B;
        ui1_B = T_trans * ui1;
        ui2_B = T_trans * ui2;
        ui3_B = T_trans * ui3;
        ui4_B = T_trans * ui4;
        ui5_B = T_trans * ui5;
        ui6_B = T_trans * ui6;
        Eigen::Vector3d to1, to2, to3, to4, to5, to6, t1, t2, t3, t4, t5, t6;
        //calculate torque with cross product
        to1 = r1_B.cross(ui1_B);
        to2 = r2_B.cross(ui2_B);
        to3 = r3_B.cross(ui3_B);
        to4 = r4_B.cross(ui4_B);
        to5 = r5_B.cross(ui5_B);
        to5 = r6_B.cross(ui6_B);
        //calculate total torque
        t1 = to1 + ratio1 * ui1_B;
        t2 = to2 + ratio2 * ui2_B;
        t3 = to3 + ratio3 * ui3_B;
        t4 = to4 + ratio4 * ui4_B;
        t5 = to5 + ratio5 * ui5_B;
        t6 = to6 + ratio6 * ui6_B;
        Eigen::MatrixXd M(6, 6);
        Eigen::MatrixXd M_trans(6, 6);
        Eigen::MatrixXd M1_inv(6, 6);
        Eigen::MatrixXd M_pseudo(6, 6);
        M << t1, t2, t3, t4, t5, t6, ui1_B, ui2_B, ui3_B, ui4_B, ui5_B, ui6_B;

        Eigen::VectorXd Thrust(6);
        Eigen::VectorXd Thrust_ist(6);
        Eigen::VectorXd Input(6);
        Input(0) = torque_x;
        Input(1) = torque_y;
        Input(2) = torque_z;
        Input(3) = force_x;
        Input(4) = force_y;
        Input(5) = force_z;
        M_trans = M.transpose();
        M1_inv = (M_trans * M).inverse();
        M_pseudo = M1_inv * M_trans;
        Thrust = M_pseudo * Input;
        //std::cout<<M<<std::endl;
        //Thrust=M.colPivHouseholderQr().solve(Input);
        // ROS_INFO_STREAM("Thrust:"<<Thrust);

        if (Thrust(0) < 0)
        {
            Thrust_ist(0) = abs(Thrust(0));
            di_vel1 = -1;
        }
        else
        {
            Thrust_ist(0) = Thrust(0);
            di_vel1 = 1;
        }

        if (Thrust(1) < 0)
        {
            Thrust_ist(1) = abs(Thrust(1));
            di_vel2 = -1;
        }
        else
        {
            Thrust_ist(1) = Thrust(1);
            di_vel2 = 1;
        }

        if (Thrust(2) < 0)
        {
            Thrust_ist(2) = abs(Thrust(2));
            di_vel3 = -1;
        }
        else
        {
            Thrust_ist(2) = Thrust(2);
            di_vel3 = 1;
        }

        if (Thrust(3) < 0)
        {
            Thrust_ist(3) = abs(Thrust(3));
            di_vel4 = -1;
        }
        else
        {
            Thrust_ist(3) = Thrust(3);
            di_vel4 = 1;
        }

        if (Thrust(4) < 0)
        {
            Thrust_ist(4) = abs(Thrust(4));
            di_vel5 = -1;
        }
        else
        {
            Thrust_ist(4) = Thrust(4);
            di_vel5 = 1;
        }

        if (Thrust(5) < 0)
        {
            Thrust_ist(5) = abs(Thrust(5));
            di_vel6 = -1;
        }
        else
        {
            Thrust_ist(5) = Thrust(5);
            di_vel6 = 1;
        }

        //std::cout<<Thrust_ist<<std::endl;
        double a0; //Polynom Profile_Drag_Coefficient y=a0x^2+b0x+c0
        a0 = 0.5 * pho * s * a * A * pa * pow(B, 3) * pow(R, 2) / 3;
        double b1, c1, b2, c2, b3, c3, b4, c4, b5, c5, b6, c6;
        b1 = (Vi1 + Vzz1) * pow(B, 2) * R * pho * s * a * A / 4;
        c1 = pho * s * a * A * pa * B * pow(Vxy1, 2) / 4;
        b2 = (Vi2 + Vzz2) * pow(B, 2) * R * pho * s * a * A / 4;
        c2 = pho * s * a * A * pa * B * pow(Vxy2, 2) / 4;
        b3 = (Vi3 + Vzz3) * pow(B, 2) * R * pho * s * a * A / 4;
        c3 = pho * s * a * A * pa * B * pow(Vxy3, 2) / 4;
        b4 = (Vi4 + Vzz4) * pow(B, 2) * R * pho * s * a * A / 4;
        c4 = pho * s * a * A * pa * B * pow(Vxy4, 2) / 4;
        b5 = (Vi5 + Vzz5) * pow(B, 2) * R * pho * s * a * A / 4;
        c5 = pho * s * a * A * pa * B * pow(Vxy5, 2) / 4;
        b6 = (Vi6 + Vzz6) * pow(B, 2) * R * pho * s * a * A / 4;
        c6 = pho * s * a * A * pa * B * pow(Vxy6, 2) / 4;
        //calculate velocity from thrust
        if (pow(b1, 2) - 4 * a0 * (c1 - Thrust_ist(0)) < 0)
        {
            vel_1 = vel_1;
        }
        else
        {
            vel_1 = -b1 / (2 * a0) + sqrt(pow(b1, 2) - 4 * a0 * (c1 - Thrust_ist(0))) / (2 * a0);
        }

        if (pow(b2, 2) - 4 * a0 * (c2 - Thrust_ist(1)) < 0)
        {
            vel_2 = vel_2;
        }
        else
        {
            vel_2 = -b2 / (2 * a0) + sqrt(pow(b2, 2) - 4 * a0 * (c2 - Thrust_ist(1))) / (2 * a0);
        }

        if (pow(b3, 2) - 4 * a0 * (c3 - Thrust_ist(2)) < 0)
        {
            vel_3 = vel_3;
        }
        else
        {
            vel_3 = -b3 / (2 * a0) + sqrt(pow(b3, 2) - 4 * a0 * (c3 - Thrust_ist(2))) / (2 * a0);
        }

        if (pow(b4, 2) - 4 * a0 * (c4 - Thrust_ist(3)) < 0)
        {
            vel_4 = vel_4;
        }
        else
        {
            vel_4 = -b4 / (2 * a0) + sqrt(pow(b4, 2) - 4 * a0 * (c4 - Thrust_ist(3))) / (2 * a0);
        }

        if (pow(b5, 2) - 4 * a0 * (c5 - Thrust_ist(4)) < 0)
        {
            vel_5 = vel_5;
        }
        else
        {
            vel_5 = -b5 / (2 * a0) + sqrt(pow(b5, 2) - 4 * a0 * (c5 - Thrust_ist(4))) / (2 * a0);
        }

        if (pow(b6, 2) - 4 * a0 * (c6 - Thrust_ist(5)) < 0)
        {
            vel_6 = vel_6;
        }
        else
        {
            vel_6 = -b6 / (2 * a0) + sqrt(pow(b6, 2) - 4 * a0 * (c6 - Thrust_ist(5))) / (2 * a0);
        }

        flypulator_common_msgs::RotorVelStamped _rotor_cmd;

        _rotor_cmd.header.stamp = ros::Time::now();
        _rotor_cmd.name.resize(6);
        _rotor_cmd.velocity.resize(6);

        _rotor_cmd.name[0] = "motor1";  
        _rotor_cmd.velocity[0] = vel_1 * di_vel1;
        _rotor_cmd.name[1] = "motor2";  
        _rotor_cmd.velocity[1] = vel_2 * di_vel2;
        _rotor_cmd.name[2] = "motor3";  
        _rotor_cmd.velocity[2] = vel_3 * di_vel3;
        _rotor_cmd.name[3] = "motor4";  
        _rotor_cmd.velocity[3] = vel_4 * di_vel4;
        _rotor_cmd.name[4] = "motor5";  
        _rotor_cmd.velocity[4] = vel_5 * di_vel5;
        _rotor_cmd.name[5] = "motor6";  
        _rotor_cmd.velocity[5] = vel_6 * di_vel6;
        this->pub_cmd.publish(_rotor_cmd);
        ros::spinOnce();
    }

  public:
    void OnRosRatioMsg(const flypulator_common_msgs::Vector6dMsgConstPtr &_thrust_moment_ratio)
    {
        ratio1 = _thrust_moment_ratio->x1;
        ratio2 = _thrust_moment_ratio->x2;
        ratio3 = _thrust_moment_ratio->x3;
        ratio4 = _thrust_moment_ratio->x4;
        ratio5 = _thrust_moment_ratio->x5;
        ratio6 = _thrust_moment_ratio->x6;
    }

  public:
    void OnRosWindMsg(const geometry_msgs::Vector3ConstPtr &_wind_msg)
    {
        Vwind_x = _wind_msg->x;
        Vwind_y = _wind_msg->y;
        Vwind_z = _wind_msg->z;
        Vdrone_x = this->model->GetRelativeLinearVel().x;
        Vdrone_y = this->model->GetRelativeLinearVel().y;
        Vdrone_z = this->model->GetRelativeLinearVel().z;
        Vx = Vwind_x - Vdrone_x;
        Vy = Vwind_y - Vdrone_y;
        Vz = Vwind_z - Vdrone_z;
    }

  public:
    void OnControlMsg(const geometry_msgs::WrenchConstPtr &_msg)
    {
        Wx_input = _msg->torque.x;
        Wy_input = _msg->torque.y;
        Wz_input = _msg->torque.z;
        Vx_input = _msg->force.x;
        Vy_input = _msg->force.y;
        Vz_input = _msg->force.z;
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
        {
            return -1;
        }
        else if (num > 0)
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }

  public:
    void setPID(const double &Kp, const double &Ki, const double &Kd, const double &pid_max, const double &pid_min)
    {
        _Kp = Kp;
        _Ki = Ki;
        _Kd = Kd;
        _pid_max = pid_max;
        _pid_min = pid_min;
    }

  public:
    double PIDController(const double &_error)
    {
        double _pre_error;
        // Proportional term
        double Pout = _Kp * _error;

        // Integral term
        double _integral;
        _integral += _error * dt;
        double Iout = _Ki * _integral;

        // Derivative term
        double derivative = (_error - _pre_error) / dt;
        double Dout = _Kd * derivative;

        // Calculate total output
        double output = Pout + Iout + Dout;

        // Restrict to max/min
        if (output > _pid_max)
            output = _pid_max;
        else if (output < _pid_min)
            output = _pid_min;

        // Save error to previous error
        _pre_error = _error;

        return output;
    }

    //load parameter from gazebo server
  private:
    void readParamsFromServer()
    {
        this->rosNode->param("rotor_number", N, N);
        this->rosNode->param("blade_chord_width", c, c);
        this->rosNode->param("blade_radius", R, R); //in this computer only three parameters can be read
                                                    /*this->rosNode->param("2D_lift_curve_slope", a, a);
 this->rosNode->param("profile_inclination_angle", th0, th0);
 this->rosNode->param("radial_inclination_change", thtw, thtw);
 this->rosNode->param("TLF", B, B);
 this->rosNode->param("air_density", pho, pho);
 this->rosNode->param("Profile_Drag_Coefficient", CD0, CD0);
 this->rosNode->param("rotor_axis_vertical_axis_angle", rv, rv);*/
    }
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

  private:
    ros::Subscriber rosSubRatio;
    //private: ros::Publisher  rosPub;
    /// \brief A ROS callbackqueue that helps process messages
  private:
    ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
  private:
    std::thread rosQueueThread;
    /// \brief A PID controller for the joint.
  private:
    common::PID pid;

  private:
    std::vector<common::PID> controllers;

  private:
    common::Time lastSimTime;

  private:
    ros::Publisher pub_cmd;

    //private: gazebo::sensors::RaySensorPtr ray_sensor;
};

// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
GZ_REGISTER_MODEL_PLUGIN(ControlPlugin)
} // namespace gazebo

#endif /*_CONTROL_PLUGIN_HH_*/
