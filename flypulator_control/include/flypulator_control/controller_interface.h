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
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"
#include "flypulator_common_msgs/RotorVelStamped.h"
#include "sensor_msgs/MultiDOFJointState.h"


class ControllerInterface {
    public:
        ControllerInterface();

        void computeControlOutput(const PoseVelocityAcceleration& x_des, const PoseVelocityAcceleration& x_current, Eigen::Matrix<float,6,1>& spinningRates);

        BaseController* getControllerReference(){
            return controller_;
        }
       
    private:
        void readDroneParameterFromServer();
        void mapControlForceTorqueInputToPropellerRates(const PoseVelocityAcceleration& x_current, Eigen::Matrix<float,6,1>& spinningRates);
        void computeMappingMatrix();

        std::map<std::string,double> drone_parameter_;
        std::string controller_type_;
        Eigen::Matrix<float,6,1> controlForceAndTorque_;
        BaseController* controller_;
        Eigen::Matrix<float, 6,6> map_matrix_;
        Eigen::Matrix<float, 6, 6> convert_force_part_to_b_;
        Eigen::Matrix<float, 6, 6> map_matrix_inverse_b_;
};

#endif // CONTROLLER_INTERFACE_H