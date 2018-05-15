#include "ros/ros.h"
#include "trajectory_generator/linear_trajectory.h"
#include "trajectory_generator/polynomial_trajectory.h"
#include "trajectory_generator/trajectory_generator.h"
//add additional service headers here

//global variable for trajectory generator class object
TrajectoryGenerator* g_generator_p;

// create polynomial trajectory from request and give response
bool createPolynomialTrajectory(trajectory_generator::polynomial_trajectory::Request &req,
                            trajectory_generator::polynomial_trajectory::Response &res){
    res.finished = g_generator_p->createAndSendTrajectory(req.x_start, req.x_end, req.rpy_start, req.rpy_end, req.delta_t, trajectory_types::Polynomial);
    return true;
}

// create linear trajectory from request and give response
bool createLinearTrajectory(trajectory_generator::linear_trajectory::Request &req,
                            trajectory_generator::linear_trajectory::Response &res){
    res.finished = g_generator_p->createAndSendTrajectory(req.x_start, req.x_end, req.rpy_start, req.rpy_end, req.delta_t, trajectory_types::Linear);
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"trajectory_generator"); //pass node name (!)
    ros::NodeHandle n;
    ros::Publisher trajectory_publisher;

    //register services
    ros::ServiceServer serviceLinTraj = n.advertiseService("linear_trajectory",createLinearTrajectory);
    ROS_INFO("Service linear_trajectory ready");
    ros::ServiceServer servicePolTraj = n.advertiseService("polynomial_trajectory",createPolynomialTrajectory);
    ROS_INFO("Service polynomial_trajectory ready");

    //register publisher for output message
    trajectory_publisher = n.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("trajectory",1000);
    
    // create trajectory generator class object
    TrajectoryGenerator m_generator (trajectory_publisher);
    g_generator_p = &m_generator; //set pointer for global variable

    ros::spin(); //keep server alive and watch for requests

    return 0;
}
