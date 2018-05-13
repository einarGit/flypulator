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
#include <math.h> //sin,cos functions, M_PI


ros::Publisher trajectory_publisher; //message publisher for output trajectory, needs to be global to be visible to create<..>Trajectory functions

// convert 2 messages of Vector3 type to 6D float array
void convertTo6DArray(geometry_msgs::Vector3 x1, geometry_msgs::Vector3 x2, float destination[]){
    destination[0] = x1.x;
    destination[1] = x1.y;
    destination[2] = x1.z;
    destination[3] = x2.x * M_PI / 180.0f;
    destination[4] = x2.y * M_PI / 180.0f;
    destination[5] = x2.z * M_PI / 180.0f;
}

// convert Euler angles to quaternions using roll-pitch-yaw sequence
Eigen::Quaternionf euler2Quaternion(const float roll, const float pitch, const float yaw){
    // following https://stackoverflow.com/questions/21412169/creating-a-rotation-matrix-with-pitch-yaw-roll-using-eigen/21414609
    Eigen::AngleAxisf rollAngle (roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle (pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle (yaw, Eigen::Vector3f::UnitZ());

    Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
    return q; 
}

Eigen::Matrix3f euler2Rot_dot(const float roll, const float roll_dot, const float pitch, const float pitch_dot, const float yaw, const float yaw_dot){
    Eigen::Matrix3f R_psi ;
    R_psi <<  - sin(yaw), -cos(yaw), 0, 
                cos(yaw), -sin(yaw),0,
                0,              0,                 0;
    R_psi = yaw_dot * R_psi;

    Eigen::Matrix3f R_theta;
    R_theta << -sin(pitch), 0, cos(pitch),
                0         , 0,             0,
                -cos(pitch), 0, -sin(pitch);
    R_theta = pitch_dot * R_theta;

    Eigen::Matrix3f R_phi;
    R_phi << 0,         0,         0,
            0, -sin(roll), -cos(roll),
            0, cos(roll), -sin(roll);
    R_phi = roll_dot * R_phi;

    Eigen::Matrix3f R_dot = R_psi * R_theta * R_phi; // roll- pitch - yaw sequence, following Janschek lecture RS02_SS16_Vorwaertskinematik.pdf
    return R_dot;
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


bool createLinearTrajectory(trajectory_generator::linear_trajectory::Request &req,
                            trajectory_generator::linear_trajectory::Response &res){
    //TODO: type check?
    //Eigen::Vector3f x_start (req.x_start.x, req.x_start.y, req.x_start.z);

    float pose_start[6];
    convertTo6DArray(req.x_start, req.rpy_start, pose_start);
    float pose_end[6];
    convertTo6DArray(req.x_end, req.rpy_end, pose_end);

    float pose_current[6];

    float duration = req.delta_t;

    // create current value structs
    geometry_msgs::Vector3 x_current;
    geometry_msgs::Vector3 x_dot_current;
    geometry_msgs::Vector3 x_ddot_current;
    geometry_msgs::Quaternion q_current_formsg;
    geometry_msgs::Vector3 omega_current_formsg;
    geometry_msgs::Vector3 omega_dot_current_formsg;

    Eigen::Quaternionf q_current;
    Eigen::Matrix3f rotMatrix_current;
    Eigen::Vector3f omega_current;
    Eigen::Matrix3f rotMatrix_dot_current;
    Eigen::Vector3f omega_dot_current;
    Eigen::Matrix3f rotMatrix_ddot_current;


    // calculate constant linear velocity
    float vel_current[6];
    for (int i = 0; i<6 ; i++){
            vel_current[i] = (pose_end[i] - pose_start[i])/duration;
    }

    float t_start = 0.0f;
    float t = t_start;
    while ( t < t_start + duration ){
        for (int i = 0; i<6 ; i++){
            pose_current[i] = vel_current[i] * t + pose_start[i];
        }
        //set linear velocities in geometry_msgs::Vector3 structs
        geometry_msgs::Vector3 x_current;
        x_current.x = pose_current[0];
        x_current.y = pose_current[1];
        x_current.z = pose_current[2];
        x_dot_current.x = vel_current[0];
        x_dot_current.y = vel_current[1];
        x_dot_current.z = vel_current[2];
        x_ddot_current.x = 0;
        x_ddot_current.y = 0;
        x_ddot_current.z = 0;

        //calculate quaternion from euler angles
        q_current = euler2Quaternion(pose_current[3], pose_current[4], pose_current[5]);
        rotMatrix_current = q_current.toRotationMatrix(); //calculate rotation matrix
        //calculate first derivative of rotation matrix
        rotMatrix_dot_current = euler2Rot_dot(pose_current[3], vel_current[3], pose_current[4], vel_current[4], pose_current[5], vel_current[5]);
        Eigen::Matrix3f tmp_matrix = rotMatrix_current.transpose() * rotMatrix_dot_current;
        
        omega_dot_current = Eigen::Vector3f(0,0,0);

        q_current_formsg.w = q_current.w();
        q_current_formsg.x = q_current.x();
        q_current_formsg.y = q_current.y();
        q_current_formsg.z = q_current.z();

        omega_current_formsg.x = tmp_matrix.coeff(3,2);//omega_current.x();
        omega_current_formsg.y = tmp_matrix.coeff(1,3);//omega_current.y();
        omega_current_formsg.z = tmp_matrix.coeff(2,1);//omega_current.z();

        omega_dot_current_formsg.x = omega_dot_current.x();
        omega_dot_current_formsg.y = omega_dot_current.y();
        omega_dot_current_formsg.z = omega_dot_current.z();

        trajectory_msgs::MultiDOFJointTrajectoryPoint msg = generateTrajectoryMessage(x_current, x_dot_current, x_ddot_current, 
                                                                                    q_current_formsg, omega_current_formsg, omega_dot_current_formsg);
        trajectory_publisher.publish(msg);
        t = t + 0.1;
    }



    res.finished = true;

    //ROS_INFO("request: x_start = %2.2f, x_end = %2.2f, duration = %2.2f", x_start,x_end,duration);




    ros::spinOnce(); // not necessary but good measure
    return true;
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
