#ifndef SLIDING_MODE_CONTROLLER_H
#define SLIDING_MODE_CONTROLLER_H

#include "flypulator_control/base_controller.h"
#include <eigen3/Eigen/Dense>

class SlidingModeController : public BaseController {
    public: 
        SlidingModeController(){};
        SlidingModeController(const float mass, const Eigen::Matrix3f inertia, const float gravity){
            // set static parameter
            mass_ = mass;
            inertia_ = inertia;
            inertia_inv_ = inertia.inverse();
            gravity_ = Eigen::Vector3f (0,0,gravity);
            integral_T_ = Eigen::Vector3f (0,0,0);
            integral_R_ = Eigen::Vector4f (0,0,0,0);
        };
        // compute Control Force and Torque
        void computeControlForceTorqueInput(const PoseVelocityAcceleration& x_des, const PoseVelocityAcceleration& x_current, Eigen::Matrix<float,6,1>& controlForceAndTorque);

        // callback for dynamic reconfigure, sets dynamic parameters (controller gains)
        void configCallback(flypulator_control::control_parameterConfig& config, uint32_t level);

    private:
        float mass_;
        Eigen::Matrix3f inertia_;
        Eigen::Matrix3f inertia_inv_;
        Eigen::Vector3f gravity_;
        //translational variables
        float lambda_T_;
        Eigen::Matrix3f K_T_;
        Eigen::Matrix3f K_T_I_;
        Eigen::Vector3f z_1_T_;
        Eigen::Vector3f z_2_T_;
        Eigen::Vector3f s_T_;
        Eigen::Vector3f s_T_I_;
        Eigen::Vector3f integral_T_;
        Eigen::Vector3f u_T_;
        Eigen::Vector3f u_T_I_;
        // rotational variables
        float lambda_R_;
        float eta_;
        float eta_d_;
        float eta_err_;
        float eta_dot_err_;
        Eigen::Matrix4f K_R_;
        Eigen::Vector3f eps_;
        Eigen::Vector3f eps_d_;
        Eigen::Vector3f eps_err_;
        Eigen::Vector3f eps_dot_err_;

        Eigen::Vector4f z_1_R_;
        Eigen::Vector3f omega_err_;
        Eigen::Matrix<float,4,3> matrix_g_transposed_;
        Eigen::Vector4f z_2_R_;
        Eigen::Matrix<float,4,3> matrix_g_dot_transposed_;
        Eigen::Vector4f s_R_;
        Eigen::Vector3f u_R_;

        Eigen::Vector4f integral_R_;
        Eigen::Matrix3f K_R_I_;
        Eigen::Vector4f s_R_I_;
        Eigen::Vector3f u_R_I_;

        // time variables
        ros::Time t_last_;
        ros::Time t_current_;
        ros::Duration t_delta_;

        // sign function without zero (sgn(x) <0 or sgn(x) > 0, there is NO x such that sgn(x) = 0)
        float sgn(float x){if (x>= 0.0f) {return 1.0f;} else {return -1.0f;}};

};



#endif //SLIDING_MODE_CONTROLLER_H
