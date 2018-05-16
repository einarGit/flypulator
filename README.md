#Trajectory Generator and Controller for Flypulator Project

## Start
Use the following command to start trajectory generator node:
` rosrun trajectory_generator trajetory_generator_server ` 

and to start sliding mode controller node:
` rosrun sliding_mode_controller sliding_mode_controller `

## User Interface
trajectory_generator can be applied via service call. It provides (atm) two services:
    linear_trajectory
    polynomial_trajectory

They can be called by the following command (for instance):

` rosservice call /linear_trajectory '{x_start: [0,0,0], x_end:  [5,5,-1], rpy_start: [-9,0,23], rpy_end: [5,85,360], delta_t: 10}'`

## Structure

The trajectory generator publishes [MultiDOFJointTrajectoryPoint Messages](http://docs.ros.org/jade/api/trajectory_msgs/html/msg/MultiDOFJointTrajectoryPoint.html) to the topic 
"/trajectory". The sliding mode controller listens to this topic.