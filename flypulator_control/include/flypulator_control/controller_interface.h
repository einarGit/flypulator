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

        void computeControlOutput(const PoseVelocityAcceleration& x_des, const PoseVelocityAcceleration& x_current, float spinningRates[6]);

        BaseController* getControllerReference(){
            return controller_;
        }
       
    private:
        void readDroneParameterFromServer();
        void mapControlForceTorqueInputToPropellerRates(const PoseVelocityAcceleration& x_current, float spinningRates[6]);

        std::map<std::string,double> drone_parameter_;
        std::string controller_type_;
        ForceTorqueInput controlForceAndTorque_;
        BaseController* controller_;
};

#endif // CONTROLLER_INTERFACE_H