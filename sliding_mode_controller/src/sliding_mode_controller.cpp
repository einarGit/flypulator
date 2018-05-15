#include "ros/ros.h"

#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"

// include message structs
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"

#include <eigen3/Eigen/Dense>
#include <math.h> //sin,cos functions, M_PI

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


}




int main(int argc, char **argv)
{
    // initialize node
    ros::init(argc, argv, "sliding_mode_controller");

    ros::NodeHandle n;

    // suscribe to trajectory messages
    ros::Subscriber sub = n.subscribe("trajectory", 1000, trajectoryMessageCallback);



    ros::spin();

    return 0;
}