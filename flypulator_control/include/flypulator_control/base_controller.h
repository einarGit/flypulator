#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H

#include <flypulator_control/control_parameterConfig.h>
#include <eigen3/Eigen/Dense>

struct PoseVelocityAcceleration {
    Eigen::Vector3f p;
    Eigen::Quaternionf q;
    Eigen::Vector3f p_dot;
    Eigen::Vector3f omega;
    Eigen::Vector3f p_ddot;
    Eigen::Vector3f omega_dot;
    void printToROSINFO(){
        ROS_INFO("pose: \n \t x \t\t = [%f, %f, %f], \n \t x_dot \t\t = [%f, %f, %f], \n \t x_ddot \t = [%f, %f, %f], \n \t q \t\t = [%f, %f, %f, %f], \n \t omega \t\t = [%f, %f, %f], \n \t omega_dot \t = [%f, %f, %f]", 
            p.x(), p.y(), p.z(), p_dot.x(), p_dot.y(), p_dot.z(), p_ddot.x(), p_ddot.y(), p_ddot.z(), 
            q.w(), q.x(), q.y(), q.z(), omega.x(), omega.y(), omega.z(), omega_dot.x(), omega_dot.y(), omega_dot.z());
    }
};

// Abstract class, no objects from this class allowed, following http://cpp.nope.bz/pure_virtual.html
// Superclass for all controller types
class BaseController {
    public: 
        virtual ~BaseController(){}; 
        // compute Control Force and Torque
        virtual void computeControlForceTorqueInput(const PoseVelocityAcceleration& x_des, const PoseVelocityAcceleration& x_current, Eigen::Matrix<float,6,1>& control_force_and_torque) = 0;
        // callback for dynamic reconfigure, sets dynamic parameters (controller gains)
        virtual void configCallback(flypulator_control::control_parameterConfig& config, uint32_t level) = 0;
};

#endif // BASE_CONTROLLER_H