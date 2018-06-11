PS: merge the develop branch to master only after validation.

# Flypulator Project

## Prerequisites

### System requirements
From [gazebosim.org](http://gazebosim.org/tutorials?cat=guided_b&tut=guided_b1):
 - A dedicated GPU,
   - Nvidia cards tend to work well in Ubuntu
 - A CPU that is at least an Intel I5, or equivalent,
 - At least 500MB of free disk space
 - Ubuntu Trusty or later installed.

### Install ROS and Gazebo
To install ROS Kinetic, follow the instructions of [Ros Wiki](http://wiki.ros.org/kinetic/Installation). It is highly recommended to use Ubuntu 16.04 64Bit. Choose to install full version of ROS (for Ubuntu, [Step 1.4: sudo apt-get install ros-kinetic-desktop-full](http://wiki.ros.org/kinetic/Installation/Ubuntu#Installation-1)). Gazebo 7 is already included in this package.

### Build
The content of this repository must be cloned into the `/src` folder of a catkin workspace ([how to create an empty workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)). For an existing workspace in `~/catkin_ws` that requires the following steps (in a new terminal):

```
cd ~/catkin_ws/src/
git clone https://github.com/FLYing-maniPULATOR/flypulator.git
cd ..
catkin_make
```
Dont forget sourcing the setup.bash file:
```
source ~/catkin_ws/devel/setup.bash
```
To do sourcing permanently, edit the .bashrc file with `gedit ~/.bashrc` and add the source command from above (`source ~/catkin_ws/devel/setup.bash`). *Note that you have to start a new terminal to apply the changes*. You can check if it has worked by trying to locate a package using `rospack find flypulator_control`.

## Gazebo simulation

### Launch:  
` roslaunch flypulator gazebo.launch ` 
includes aerodynamics, fake sensor plugin & parameters.

### Check tf in rviz: 
roslaunch flypulator_description display.launch

## Trajectory Generator and Controller
The controller package (flypulator_control) provides a sliding mode controller for a fully actuated hexarotor. By now, the packages are located at `~/catkin_ws/src/flypulator/flypulator_traj_generator` and `~/catkin_ws/src/flypulator/flypulator_control`.

### Start
The *rosrun* command works as follows: `rosrun <package> <node>`
Use the following command to start trajectory generator node:

` rosrun flypulator_traj_generator trajetory_generator_server ` 

and to start controller node:

` rosrun flypulator_control controller_node `

Remember that `roscore` must be running!

To start both packages, controller and trajectory generator, and load the .yaml file including the model parameters, a launch file is provided in `~/catkin_ws/src/flypulator/flypulator/launch` named `controller.launch`, which can be launched using the following command:

` roslaunch flypulator controller.launch `

### User Interface
The trajectory generator can be applied via service call. It provides (atm) two services:
    linear_trajectory
    polynomial_trajectory

They can be called by the following command (for instance):

` rosservice call /linear_trajectory '{x_start: [0,0,0], x_end:  [5,5,-1], rpy_start: [-9,0,23], rpy_end: [5,85,360], delta_t: 10}'`

The model parameters are defined in the file `drone_parameter.yaml` located at `~/catkin_ws/src/flypulator/flypulator_description/param/`. They have to be load to ROS parameter server either using the provided launchfile `controller.launch` or using the following command: 

` rosparam load flypulator_ws/src/flypulator/flypulator_description/param/drone_parameter.yaml `

The control parameters can be changed via runtime using the [dynamic_reconfigure package](http://wiki.ros.org/dynamic_reconfigure). The default values are defined in the file `control_parameter.cfg` located at `~/catkin_ws/src/flypulator/flypulator_control/cfg/`. To start the dynamic reconfigure GUI, use the following command:

` rosrun rqt_reconfigure rqt_reconfigure `

The new values are passed to the controller, which also can be watched in console output. Note that the parameters are for an ISM controller; to support additional controllers, they need to implement the BaseController interface/abstract class and the file `control_parameter.cfg` needs to be adapted as well as the ControllerInterface class, where the new controller type has to be registered.

The debug level can be changed in a GUI by running

` rosrun rqt_logger_level rqt_logger_level ` or simply ` rqt_logger_level`


### Structure

The trajectory generator publishes [MultiDOFJointTrajectoryPoint Messages](http://docs.ros.org/jade/api/trajectory_msgs/html/msg/MultiDOFJointTrajectoryPoint.html) to the topic 
"/trajectory". The controller suscribes to this topic, and to state estimation messages (*UavStateStamped.msg* from `flypulator_common_msgs`). Every time the controller gets a state estimation message, the control output is calculated using the latest available desired pose. The calculated spinning velocities are published on topic `rotor_cmd` using a *RotorVelStamped* - message from package `flypulator_common_msgs`.
For further information, consider the corresponding diploma thesis, chapter 6.
The motors feedforward control requires the sampling time of the state estimation as parameter, which must be on ros parameter server as `state_estimation/sampling_time`. This is included in launchfile `gazebo.launch`.

### Code

The code style follows the [ROS C++ Style Guide](http://wiki.ros.org/CppStyleGuide). Hence, class member variables have a underscore suffix (e.g. `variable1_`). Global variables have a leading `g_` prefix (e.g. `g_variable2`). For performance reasons, functions which are called frequently do not return values, but get a reference on the output passed as argument, so the result can be stored in this reference. This is a commonly used principle in C++.

## Known Issues

 - Sometimes gazebo fails to start. Just exit and start again
 - If `catkin_make` does not succeed because some header files are missing, just run it twice.
 - Sometimes the drone model is not loaded correctly and there are errors in console. Try log out and log in (Linux) or delete `/build` and `/devel` folder in `~/catkin_ws/` and rerun `catkin_make`.
