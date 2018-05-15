#include "ros/ros.h"

#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"

// include message structs
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"

#include <eigen3/Eigen/Dense>
#include <math.h> //sin,cos functions, M_PI

struct PoseVelocityAcceleration {
    Eigen::Vector3f p;
    Eigen::Quaternionf q;
    Eigen::Vector3f p_dot;
    Eigen::Vector3f omega;
    Eigen::Vector3f p_ddot;
    Eigen::Vector3f omega_dot;
};

struct ForceTorque {
    Eigen::Vector3f f; // force
    Eigen::Vector3f tau; // torque
};

class SlidingModeController {
    public: 
        SlidingModeController(float drone_parameter){
            m_drone_parameter = drone_parameter;
        };
        void computeControlForceAndTorque(const PoseVelocityAcceleration& x_des, const PoseVelocityAcceleration& x_current, ForceTorque& control_force_and_torque){
            //compute force and torque
            // provide output through pass by reference
            //control_force = ..;
            //control_torque = ...;
        };

    private:
        float integratorValues;
        float slidingSurface;
        float drone_parameter;
        void integrate(){};

};

class BaseController {
    public:
        BaseController(){
            readDroneParameterFromServer();
            SlidingModeController m_sliding_mode_controller (m_drone_parameter);
        };
        void computeControlOutput(const PoseVelocityAcceleration& x_des, const PoseVelocityAcceleration& x_current, float spinningRates[6]){
            
            m_sliding_mode_controller.computeControlForceAndTorque(x_des, x_current, m_control_force_and_torque);

            mapControlForceTorqueToPropellerRates(spinningRates);
        };
       
    private:
        void readDroneParameterFromServer(){};
        void mapControlForceTorqueToPropellerRates(float spinningRates[6]){};
        float m_drone_parameter;
        ForceTorque m_control_force_and_torque;
        SlidingModeController m_sliding_mode_controller;
};




BaseController* g_drone_controller_p;
PoseVelocityAcceleration g_currentPose;
PoseVelocityAcceleration g_desiredPose;

// receive trajectory message
void trajectoryMessageCallback(const trajectory_msgs::MultiDOFJointTrajectoryPoint::ConstPtr& msg){
    geometry_msgs::Transform transform = msg->transforms[0];
    geometry_msgs::Twist velocity = msg->velocities[0];
    geometry_msgs::Twist acceleration = msg->accelerations[0];

    geometry_msgs::Vector3 p_des_msg = transform.translation;
    geometry_msgs::Quaternion q_des_msg = transform.rotation;
    geometry_msgs::Vector3 p_dot_des_msg = velocity.linear;
    geometry_msgs::Vector3 omega_des_msg = velocity.angular;
    geometry_msgs::Vector3 p_ddot_des_msg = acceleration.linear;
    geometry_msgs::Vector3 omega_dot_des_msg = acceleration.angular;

    Eigen::Vector3f p_des (p_des_msg.x, p_des_msg.y, p_des_msg.z);
    Eigen::Quaternionf q_des (q_des_msg.w, q_des_msg.x, q_des_msg.y, q_des_msg.z);

    Eigen::Vector3f p_dot_des (p_dot_des_msg.x, p_dot_des_msg.y, p_dot_des_msg.z);
    Eigen::Vector3f omega_des (omega_des_msg.x, omega_des_msg.y, omega_des_msg.z);

    Eigen::Vector3f p_ddot_des (p_ddot_des_msg.x, p_ddot_des_msg.y, p_ddot_des_msg.z);
    Eigen::Vector3f omega_dot_des (omega_dot_des_msg.x, omega_dot_des_msg.y, omega_dot_des_msg.z);

    ros::Duration duration = msg->time_from_start;

    ROS_DEBUG("Received trajectory message: x_des = [%f, %f, %f], q_des = [%f, %f, %f, %f]", p_des.x(), p_des.y(), p_des.z(), q_des.w(), q_des.x(), q_des.y(), q_des.z());
    ROS_DEBUG("    Time from start: %f s", duration.toSec());

    // update global variable for desired pose
    g_desiredPose.p = p_des;
    g_desiredPose.q = q_des;
    g_desiredPose.p_dot = p_dot_des;
    g_desiredPose.omega = omega_des;
    g_desiredPose.p_ddot = p_ddot_des;
    g_desiredPose.omega_dot = omega_dot_des;

}

void computeControlOutputAndPublish(){
    float spinningRates[6]; 
    g_drone_controller_p->computeControlOutput(g_desiredPose, g_currentPose, spinningRates);
}


int main(int argc, char **argv)
{
    // initialize node
    ros::init(argc, argv, "sliding_mode_controller");

    ros::NodeHandle n;

    // suscribe to trajectory messages
    ros::Subscriber sub = n.subscribe("trajectory", 1000, trajectoryMessageCallback);

    // create controller
    //BaseController m_drone_controller();
    //g_drone_controller_p = & m_drone_controller;

    ros::spin();

    return 0;
}