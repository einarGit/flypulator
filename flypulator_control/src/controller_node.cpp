#include "ros/ros.h"
#include "flypulator_control/controller_interface.h" // performs all necessary includes
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"

ControllerInterface* g_drone_controller_p;
PoseVelocityAcceleration g_currentPose;
PoseVelocityAcceleration g_desiredPose;
int g_rotor_vel_message_counter = 0;
ros::Publisher* g_rotor_cmd_pub;
Eigen::Matrix<float,6,1> g_spinningRates;

void computeControlOutputAndPublish(){
    
    // compute spinning rates
    ROS_DEBUG("Compute Control Output..");
    g_drone_controller_p->computeControlOutput(g_desiredPose, g_currentPose, g_spinningRates);
    ROS_DEBUG("Control Output computed! Prepare rotor cmd message...");
    // build message
    flypulator_common_msgs::RotorVelStamped msg;
    msg.header.stamp = ros::Time::now();
    for (int i = 0; i<6;i++){
        msg.velocity.push_back(g_spinningRates(i,0));
        msg.name.push_back(std::string("blade_joint") + std::to_string(i)); 
    }
    ROS_DEBUG("Send rotor cmd message..");
    g_rotor_cmd_pub->publish(msg);
}

// encode a trajectory_msgs::MultiDOFJointTrajectoryPoint message to PoseVelocityAcceleration object
void encodeTrajectoryMsg(const trajectory_msgs::MultiDOFJointTrajectoryPoint::ConstPtr& msg, PoseVelocityAcceleration& pose_dest){
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

    // update global variable for desired pose
    pose_dest.p = p_des;
    pose_dest.q = q_des;
    pose_dest.p_dot = p_dot_des;
    pose_dest.omega = omega_des;
    pose_dest.p_ddot = p_ddot_des;
    pose_dest.omega_dot = omega_dot_des;
}


// receive trajectory message
void trajectoryMessageCallback(const trajectory_msgs::MultiDOFJointTrajectoryPoint::ConstPtr& msg){
    // encode trajectory message to PoseVelocityAcceleration object
    encodeTrajectoryMsg(msg, g_desiredPose);

    ros::Duration duration = msg->time_from_start;

    ROS_DEBUG("Received trajectory message: x_des = [%f, %f, %f], q_des = [%f, %f, %f, %f]", g_desiredPose.p.x(), g_desiredPose.p.y(), g_desiredPose.p.z(),
        g_desiredPose.q.w(), g_desiredPose.q.x(), g_desiredPose.q.y(), g_desiredPose.q.z());
    ROS_DEBUG("    Time from start: %f s", duration.toSec());
}


// receive state estimation message
void stateMessageCallback(const trajectory_msgs::MultiDOFJointTrajectoryPoint::ConstPtr& msg){
    // Tencode state message to PoseVelocityAcceleration object
    encodeTrajectoryMsg(msg, g_currentPose);

    ros::Duration duration = msg->time_from_start;

    ROS_DEBUG("Received state message: x_des = [%f, %f, %f], q_des = [%f, %f, %f, %f]", g_currentPose.p.x(), g_currentPose.p.y(), g_currentPose.p.z(), 
        g_currentPose.q.w(), g_currentPose.q.x(), g_currentPose.q.y(), g_currentPose.q.z());
    ROS_DEBUG("    Time from start: %f s", duration.toSec());

    // compute control output to updated state information
    computeControlOutputAndPublish();
}


int main(int argc, char **argv)
{
    // initialize node
    ros::init(argc, argv, "controller");

    ros::NodeHandle n;

    // suscribe to trajectory messages
    ros::Subscriber sub = n.subscribe("trajectory", 1000, trajectoryMessageCallback);

    // suscribe to state estimation messages
    ros::Subscriber sub_2 = n.subscribe("state_estimation", 1000, stateMessageCallback);

    // ready to publish rotor command messages
    ros::Publisher rotor_cmd_pub = n.advertise<flypulator_common_msgs::RotorVelStamped>("rotor_cmd", 1000);
    g_rotor_cmd_pub = &rotor_cmd_pub;

    // create controller
    ControllerInterface m_drone_controller;
    g_drone_controller_p = &m_drone_controller;

    // Set up a dynamic reconfigure server following
    // https://github.com/UCSD-E4E/stingray-auv/wiki/Writing-publisher-subscriber-with-dynamic-reconfigure-and-parameter-server-(C----)
    dynamic_reconfigure::Server<flypulator_control::control_parameterConfig> dr_srv;
    dynamic_reconfigure::Server<flypulator_control::control_parameterConfig>::CallbackType cb;
    cb = boost::bind(&BaseController::configCallback, g_drone_controller_p->getControllerReference() , _1, _2); //set callback of controller object
    dr_srv.setCallback(cb);

    // set inital quaternions (default initialization zero)
    g_desiredPose.q = Eigen::Quaternionf (1,0,0,0);
    g_currentPose.q = Eigen::Quaternionf (1,0,0,0);

    ros::spin();

    return 0;
}