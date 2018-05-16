# Trajectory Generator and Controller for Flypulator Project

## Build
The content of this repository must be cloned into the `/src` folder of a catkin workspace. For example, an existing workspace in `~/catkin_ws` needs the following (in new Terminal):

```
cd ~/catkin_ws/src/
git clone https://github.com/danneboom/flypulator_tracking_control
cd ..
catkin_make
```
Dont forget sourcing the setup.bash file:
```
source ~/catkin_ws/devel/setup.bash
```
To do sourcing permanently, edit the .bashrc file with `gedit ~/.bashrc` and add the source command from above (`source ~/catkin_ws/devel/setup.bash`). Note that you have to start a new terminal to apply the changes. You can check if it has worked by trying to locate your package using `rospack find sliding_mode_controller`.

So by now, the packages are located at `~/catkin_ws/src/flypulator_tracking_control/trajectory_generator` and `~/catkin_ws/src/flypulator_tracking_control/sliding_mode_controller`.

## Start
Use the following command to start trajectory generator node:

` rosrun trajectory_generator trajetory_generator_server ` 

and to start sliding mode controller node:

` rosrun sliding_mode_controller sliding_mode_controller `

Remember that `roscore` must be running!

## User Interface
trajectory_generator can be applied via service call. It provides (atm) two services:
    linear_trajectory
    polynomial_trajectory

They can be called by the following command (for instance):

` rosservice call /linear_trajectory '{x_start: [0,0,0], x_end:  [5,5,-1], rpy_start: [-9,0,23], rpy_end: [5,85,360], delta_t: 10}'`

## Structure

The trajectory generator publishes [MultiDOFJointTrajectoryPoint Messages](http://docs.ros.org/jade/api/trajectory_msgs/html/msg/MultiDOFJointTrajectoryPoint.html) to the topic 
"/trajectory". The sliding mode controller listens to this topic.