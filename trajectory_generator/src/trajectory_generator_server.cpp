#include "ros/ros.h"
#include "trajectory_generator/linear_trajectory.h"
//add additional service headers here

// include message structs
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"

#include <eigen3/Eigen/Dense> //TODO Why does <Eigen/Dense> not work??


ros::Publisher trajectory_publisher; //message publisher for output trajectory, needs to be global to be visible to create<..>Trajectory functions

// convert 2 messages of Vector3 type to 6D float array
void convertTo6DArray(geometry_msgs::Vector3 x1, geometry_msgs::Vector3 x2, float destination[]){
    destination[0] = x1.x;
    destination[1] = x1.y;
    destination[2] = x1.z;
    destination[3] = x2.x;
    destination[4] = x2.y;
    destination[5] = x2.z;
}

bool createLinearTrajectory(trajectory_generator::linear_trajectory::Request &req,
                            trajectory_generator::linear_trajectory::Response &res){
    //TODO: type check?
    // geometry_msgs::Vector3 x_start = req.x_start;
    // geometry_msgs::Vector3 x_end = req.x_end;
    // geometry_msgs::Vector3 rpy_start = req.rpy_start;
    // geometry_msgs::Vector3 rpy_end = req.rpy_end;
    Eigen::Vector3f x_start (req.x_start.x, req.x_start.y, req.x_start.z);

    float pose_start[6];
    convertTo6DArray(req.x_start, req.rpy_start, pose_start);
    float pose_end[6];
    convertTo6DArray(req.x_end, req.rpy_end, pose_end);

    float pose_current[6];

    float duration = req.delta_t;

    geometry_msgs::Vector3 x_current;
    geometry_msgs::Vector3 x_dot_current;
    geometry_msgs::Vector3 x_ddot_current;

    float vel_current[6];
    for (int i = 0; i<6 ; i++){
            vel_current[i] = (pose_end[i] - pose_start[i])/duration;
    }

    float t_start = 0;
    float t = t_start;
    while (it < 10){
        for (int i = 0; i<6 ; i++){
            pose_current[i] = vel_current[i] / duration * t + pose_start[i];
        }
        x_current = { pose_current[0], pose_current[1], pose_current[2] };
        x_dot_current = { vel_current[0], vel_current[1], vel_current[2] };
        x_ddot_current = {0,0,0}

        trajectory_msgs::MultiDOFJointTrajectoryPoint msg = generateTrajectoryMessage();
        trajectory_publisher.publish(msg);
    }



    res.finished = true;

    //ROS_INFO("request: x_start = %2.2f, x_end = %2.2f, duration = %2.2f", x_start,x_end,duration);




    ros::spinOnce(); // not necessary but good measure
    return true;
}

// create trajectory message 
trajectory_msgs::MultiDOFJointTrajectoryPoint generateTrajectoryMessage(geometry_msgs::Vector3& x, geometry_msgs::Vector3& x_dot, geometry_msgs::Vector3& x_dotdot, 
                                                                        geometry_msgs::Quaternion& q, geometry_msgs::Vector3& omega, geometry_msgs::Vector3& omega_dot){
    //TODO confirm valid input
    //TODO Maybe pass reference to output to avoid copying of msg_trajectory

    geometry_msgs::Transform msg_transform; //Pose
    msg_transform.translation = x;
    msg_transform.rotation = q;

    geometry_msgs::Twist msg_velocities; //Velocities
    msg_velocities.linear = x_dot;
    msg_velocities.angular = omega;

    geometry_msgs::Twist msg_accelerations; //Accelerations
    msg_accelerations.linear = x_dotdot;
    msg_accelerations.angular = omega_dot;

    trajectory_msgs::MultiDOFJointTrajectoryPoint msg_trajectory;
    msg_trajectory.transforms.push_back(msg_transform); //in C++ Ros Message Arrays are implemented as std::vector
    msg_trajectory.velocities.push_back(msg_velocities);
    msg_trajectory.accelerations.push_back(msg_accelerations);

    return msg_trajectory;
}



int main(int argc, char **argv)
{
    ros::init(argc,argv,"trajectory_generator"); //pass node name (!)
    ros::NodeHandle n;

    //register service
    ros::ServiceServer serviceLinTraj = n.advertiseService("linear_trajectory",createLinearTrajectory);
    ROS_INFO("Service linear_trajectory ready");

    //register publisher for output message
    trajectory_publisher = n.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("trajectory",1000);
    //trajectory_publisher = n.advertise<std_msgs::String>("trajectory",1000);
    

    ros::spin(); //keep server alive and watch for requests

    return 0;
}
