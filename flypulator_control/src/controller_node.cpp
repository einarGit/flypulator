#include "ros/ros.h"
#include "flypulator_control/controller_interface.h" // performs all necessary includes

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

    computeControlOutputAndPublish();

}

// receive state estimation message
void stateMessageCallback(const sensor_msgs::MultiDOFJointState::ConstPtr& msg){
    // TODO: receive message to g_currentPose and call computeControlOutputAndPublish here instead above
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