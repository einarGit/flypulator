#ifndef CONTROLLER_INTERFACE_H
#define CONTROLLER_INTERFACE_H

#include "ros/ros.h"

#include <eigen3/Eigen/Dense>

// include controller classes
#include "base_controller.h"
#include "flypulator_control/sliding_mode_controller.h"

//dynamic reconfigure includes
#include <dynamic_reconfigure/server.h>
#include <flypulator_control/control_parameterConfig.h>

// include message structs
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "flypulator_common_msgs/RotorVelStamped.h"


class ControllerInterface {
    public:
        ControllerInterface(); // constructor implemented in .cpp file

        // compute spinning rates from current and desired pose
        void computeControlOutput(const PoseVelocityAcceleration& x_des, const PoseVelocityAcceleration& x_current, Eigen::Matrix<float,6,1>& spinning_rates);

        // return a reference to controller for dynamic reconfigure
        BaseController* getControllerReference(){
            return controller_;
        }
       
    private:
        // read uav parameter from ros parameter server
        void readDroneParameterFromServer();
        // map control forces and torques to propeller spinning rates
        void mapControlForceTorqueInputToPropellerRates(const PoseVelocityAcceleration& x_current, Eigen::Matrix<float,6,1>& spinning_rates);
        // compute mapping matrix from spinning rates to forces/torques
        void computeMappingMatrix();

        // map of drone parameters 
        std::map<std::string,double> drone_parameter_;
        // controller type
        std::string controller_type_;
        // 6D- Vector of control force and torque (output of controller class)
        Eigen::Matrix<float,6,1> control_force_and_torque_;
        // pointer to controller;
        BaseController* controller_;
        // mapping matrix
        Eigen::Matrix<float, 6,6> map_matrix_;
        // matrix containing R_BtoI matrix to convert body forces to inertal frame 
        Eigen::Matrix<float, 6, 6> convert_force_part_to_b_;
        // inverse of mapping matrix
        Eigen::Matrix<float, 6, 6> map_matrix_inverse_b_;
};

#endif // CONTROLLER_INTERFACE_H