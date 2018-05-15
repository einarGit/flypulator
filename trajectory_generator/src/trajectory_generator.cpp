#include "trajectory_generator/trajectory_generator.h"

// create Trajectory and send it periodically
bool TrajectoryGenerator::createAndSendTrajectory(const geometry_msgs::Vector3& x_start, const geometry_msgs::Vector3& x_end, 
                                                  const geometry_msgs::Vector3& rpy_start, const geometry_msgs::Vector3& rpy_end, 
                                                  const float duration, const TrajTypes::Type traj_type){
    //TODO: check if trajectory is feasible? (vel and acc to high, duration to low)
    // save input values in 6D array
    float pose_start[6];
    convertTo6DArray(x_start, rpy_start, pose_start);
    float pose_end[6];
    convertTo6DArray(x_end, rpy_end, pose_end);
   //float duration = req.delta_t; // trajectory duration time

    float pose_current[6]; // array for 6D pose
    float vel_current[6]; // array for 6D velocities
    float acc_current[6]; // array for 6D accelerations
    // create math objects for quaternion and omega calculations using Eigen library
    Eigen::Quaternionf q_current;
    Eigen::Vector3f omega_current;
    Eigen::Vector3f omega_dot_current;

    float a[6][6]; // polynomial coefficients for trajectory, first dimension: axis, second dimension: coefficient (can be more than 6 for higher order polynoms)

    // calculate polynomial coeffiencts for linear and polynomial trajectory
    switch (traj_type){
        
        case TrajTypes::Linear:
            // calculate constant linear velocity and acceleration (=0)
            for (int dim = 0; dim<6; dim++){
                a[dim][0] = pose_start[dim];
                a[dim][1] = (pose_end[dim] - pose_start[dim])/duration;
                a[dim][2] = 0;
                a[dim][3] = 0;
                a[dim][4] = 0;
                a[dim][5] = 0;
            }
            ROS_INFO("Start linear trajectory..");
            break;

        case TrajTypes::Polynomial:
            // calculate polynomial coefficients
            for (int dim = 0; dim<6; dim++){
                a[dim][0] = pose_start[dim];
                a[dim][1] = 0.0f;
                a[dim][2] = 0.0f;
                a[dim][3] = 10*(pose_end[dim] - pose_start[dim])/pow(duration,3);
                a[dim][4] = -15*(pose_end[dim] - pose_start[dim])/pow(duration,4);
                a[dim][5] = 6*(pose_end[dim] - pose_start[dim])/pow(duration,5);
            }
            ROS_INFO("Start polynomial trajectory..");
            break;

        default:
            ROS_ERROR("Polynom type not well defined! Must follow enumeration");
    }
    
    const ros::Time t_start = ros::Time::now(); // Take ros time - sync with "use_sim_time" parameter over /clock topic
    const ros::Duration trajDuration (duration);

    ros::Time t (t_start.toSec()); // running time variable

    ROS_DEBUG("Start Time: %f", t.toSec());

    ros::Rate r(10); //TODO: take Frequency of state estimation

    // start continous message publishing 
    while ( t <= t_start + trajDuration ){
        // calculate trajectory for position, velocity, acceleration, and euler angles trajectory and its derivatives (not equal to omega and omega_dot!!)
        if (traj_type == TrajTypes::Linear || traj_type == TrajTypes::Polynomial)
        {
            // linear trajectory is also a polynomial trajectory with different coeffiencts (set before)
            float dt = (float) (t.toSec() - t_start.toSec());
            for (int dim = 0; dim < 6; dim++){
                pose_current[dim] = a[dim][0] + a[dim][1]*dt + a[dim][2]*pow(dt,2) + a[dim][3]*pow(dt,3) + a[dim][4]*pow(dt,4) + a[dim][5]*pow(dt,5);
                vel_current[dim] = a[dim][1] + 2*a[dim][2]*dt + 3*a[dim][3]*pow(dt,2) + 4*a[dim][4]*pow(dt,3) + 5*a[dim][5]*pow(dt,4);
                acc_current[dim] = 2*a[dim][2] + 6*a[dim][3]*dt + 12*a[dim][4]*pow(dt,2) + 20*a[dim][5]*pow(dt,3);
            }
        }
        // add other trajectory types here
       
        // calculate rotational trajectory from euler angles trajectory
        // calculate quaternion from euler angles
        euler2Quaternion(pose_current[3], pose_current[4], pose_current[5], q_current);
        // calculate omega from euler angles and their derivative
        calculateOmega(pose_current[3], vel_current[3], pose_current[4], vel_current[4], pose_current[5], vel_current[5], omega_current);
        // calculate omega_dot from euler angles and their derivatives
        calculateOmegaDot(pose_current[3], vel_current[3], acc_current[3],
                          pose_current[4], vel_current[4], acc_current[4],
                          pose_current[5], vel_current[5], acc_current[5], omega_dot_current);

        // Concatenate informations to trajectory message
        trajectory_msgs::MultiDOFJointTrajectoryPoint msg = generateTrajectoryMessage(pose_current, vel_current, acc_current, 
                                                                                    q_current, omega_current, omega_dot_current, ros::Duration(t-t_start));
        this->trajectory_publisher.publish(msg); //publish message to topic /trajectory

        ros::spinOnce(); // not necessary (?) but good measure
        r.sleep(); // keep frequency, so sleep until next timestep
        t = ros::Time::now(); //update time
        ROS_DEBUG("Current Time: %f", t.toSec());
    }

    ROS_INFO("trajectory finished!");

    return true;
}

// convert 2 messages of Vector3 type to 6D float array
void TrajectoryGenerator::convertTo6DArray(const geometry_msgs::Vector3& x1, const geometry_msgs::Vector3& x2, float destination[]){
    destination[0] = x1.x;
    destination[1] = x1.y;
    destination[2] = x1.z;
    destination[3] = x2.x * M_PI / 180.0f; // convert to rad!
    destination[4] = x2.y * M_PI / 180.0f; // convert to rad!
    destination[5] = x2.z * M_PI / 180.0f; // convert to rad!
}

// convert Euler angles to quaternions using roll-pitch-yaw sequence
void TrajectoryGenerator::euler2Quaternion(const float roll, const float pitch, const float yaw, Eigen::Quaternionf& q){
    // following https://stackoverflow.com/questions/21412169/creating-a-rotation-matrix-with-pitch-yaw-roll-using-eigen/21414609
    Eigen::AngleAxisf rollAngle (roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle (pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle (yaw, Eigen::Vector3f::UnitZ());

    q = yawAngle * pitchAngle * rollAngle;
}

// calculate angular velocity from euler angles and its derivatives, following Fje94 p.42
void TrajectoryGenerator::calculateOmega(const float roll, const float roll_dot, const float pitch, const float pitch_dot, 
                                            const float yaw, const float yaw_dot, Eigen::Vector3f& omega){
    omega.x() = roll_dot - yaw_dot*sin(pitch);
    omega.y() = pitch_dot*cos(roll) + yaw_dot*sin(roll)*cos(pitch);
    omega.z() = -pitch_dot*sin(roll) + yaw_dot*cos(roll)*cos(pitch);
}

// calculate angular acceleration from euler angles and its derivatives, following Fje94 p.42 derivated
void TrajectoryGenerator::calculateOmegaDot(const float roll, const float roll_dot, const float roll_ddot,
                                  const float pitch, const float pitch_dot, const float pitch_ddot,
                                  const float yaw, const float yaw_dot, const float yaw_ddot, Eigen::Vector3f& omega_dot){
    omega_dot.x() = roll_ddot - (yaw_ddot*sin(pitch) + yaw_dot*pitch_dot*cos(pitch));
    omega_dot.y() = pitch_ddot*cos(roll) - pitch_dot*roll_dot*sin(roll) + yaw_ddot*sin(roll)*cos(pitch) + 
                    yaw_dot*(roll_dot*cos(roll)*cos(pitch) - pitch_dot*sin(roll)*sin(pitch));
    omega_dot.z() = -(pitch_ddot*sin(roll) + roll_dot*pitch_dot*cos(roll)) + yaw_ddot*cos(roll)*cos(pitch) - 
                    yaw_dot*(roll_dot*sin(roll)*cos(pitch) + pitch_dot*cos(roll)*sin(pitch));
}

// create trajectory message 
trajectory_msgs::MultiDOFJointTrajectoryPoint TrajectoryGenerator::generateTrajectoryMessage(const float p[6], const float p_dot[6], const float p_ddot[6], 
                                                                        const Eigen::Quaternionf& q, const Eigen::Vector3f& omega, const Eigen::Vector3f& omega_dot, 
                                                                        const ros::Duration& time_from_start){
    //TODO Maybe pass reference to output to avoid copying of msg_trajectory

    //set linear position, velocity and acceleration in geometry_msgs::Vector3 structs
    geometry_msgs::Vector3 p_msg;   
    p_msg.x = p[0];
    p_msg.y = p[1];
    p_msg.z = p[2];

    geometry_msgs::Vector3 p_dot_msg;
    p_dot_msg.x = p_dot[0];
    p_dot_msg.y = p_dot[1];
    p_dot_msg.z = p_dot[2];

    geometry_msgs::Vector3 p_ddot_msg;
    p_ddot_msg.x = p_ddot[0];
    p_ddot_msg.y = p_ddot[1];
    p_ddot_msg.z = p_ddot[2];

    geometry_msgs::Quaternion q_msg; // set quaternion struct
    q_msg.w = q.w();
    q_msg.x = q.x();
    q_msg.y = q.y();
    q_msg.z = q.z();

    geometry_msgs::Vector3 omega_msg; // set omega struct
    omega_msg.x = omega.x();
    omega_msg.y = omega.y();
    omega_msg.z = omega.z();

    geometry_msgs::Vector3 omega_dot_msg; // set omega_dot struct
    omega_dot_msg.x = omega_dot.x();
    omega_dot_msg.y = omega_dot.y();
    omega_dot_msg.z = omega_dot.z();

    geometry_msgs::Transform msg_transform; //Pose
    msg_transform.translation = p_msg;
    msg_transform.rotation = q_msg;

    geometry_msgs::Twist msg_velocities; //Velocities
    msg_velocities.linear = p_dot_msg;
    msg_velocities.angular = omega_msg;

    geometry_msgs::Twist msg_accelerations; //Accelerations
    msg_accelerations.linear = p_ddot_msg;
    msg_accelerations.angular = omega_dot_msg;

    trajectory_msgs::MultiDOFJointTrajectoryPoint msg_trajectory;
    msg_trajectory.transforms.push_back(msg_transform); //in C++ Ros Message Arrays are implemented as std::vector
    msg_trajectory.velocities.push_back(msg_velocities);
    msg_trajectory.accelerations.push_back(msg_accelerations);
    msg_trajectory.time_from_start = time_from_start;

    return msg_trajectory;
}