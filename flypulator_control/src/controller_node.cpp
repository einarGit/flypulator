#include "ros/ros.h"

#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"

// include message structs
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"

#include <eigen3/Eigen/Dense>
#include <math.h> //sin,cos functions, M_PI

//dynamic reconfigure includes
#include <dynamic_reconfigure/server.h>
#include <flypulator_control/control_parameterConfig.h>

struct PoseVelocityAcceleration {
    Eigen::Vector3f p;
    Eigen::Quaternionf q;
    Eigen::Vector3f p_dot;
    Eigen::Vector3f omega;
    Eigen::Vector3f p_ddot;
    Eigen::Vector3f omega_dot;
};

struct ForceTorqueInput {
    Eigen::Vector3f u_T; // translational force input
    Eigen::Vector3f u_R; // rotational torque input
};

class SlidingModeController {
    public: 
        SlidingModeController(){};
        SlidingModeController(float drone_parameter){
            drone_parameter_ = drone_parameter;
            integral_T_ = Eigen::Vector3f (0,0,0);
            integral_R_ = Eigen::Vector4f (0,0,0,0);
        };
        void computeControlForceTorqueInput(const PoseVelocityAcceleration& x_des, const PoseVelocityAcceleration& x_current, ForceTorqueInput& controlForceAndTorque){
            //compute force and torque
            //translational control
            z_1_T_ = x_current.p - x_des.p;
            z_2_T_ = x_current.p_dot - x_des.p_dot;

            // update sliding surface
            s_T_ = z_2_T_ + lambda_T_ * z_1_T_;

            // calculate translational output, take elementwise arctan with .array() and convert back with .matrix()
            u_T_ = - lambda_T_ * z_2_T_ + gravity_ + x_des.p_ddot - 2.0f/M_PI * K_T_ * (atan(s_T_.array())).matrix(); 
            
            // integral sliding mode: suppose zero intial conditions (?)
            t_current_ = ros::Time::now(); // get current time
            t_delta_ = t_current_ - t_last_; // calculate time difference for integral action
            t_last_ = t_current_;
            integral_T_ = integral_T_ + 2.0f /M_PI * K_T_ * (atan(s_T_.array())).matrix() * t_delta_.toSec(); 
            s_T_I_ = s_T_ + integral_T_;
            u_T_I_ = - 2/M_PI * K_T_I_ * (atan(s_T_I_.array())).matrix(); 

            // provide output through pass by reference
            controlForceAndTorque.u_T = (u_T_ + u_T_I_) * mass_; // convert to force input by multiplying with mass (f=m*a)
        
            // Rotational controller
            // calculate error quaternion
            eta_ = x_current.q.w();
            eta_d_ = x_des.q.w();
            eps_ = x_current.q.vec();
            eps_d_ = x_des.q.vec();

            eta_err_ = eta_d_ * eta_ + eps_d_.dot(eps_); //transposed eps_d_ times eps_ is equal to dot product
            eps_err_ = eta_d_*eps_ - eta_ * eps_d_ - eps_d_.cross(eps_); // skew symmetric matrix times vector is equal to cross product

            // calculate z1
            z_1_R_(0) = 1 - std::abs(eta_err_); // std::abs is overloaded by math.h such that abs(float) works
            z_1_R_(1) = eps_err_.x();
            z_1_R_(2) = eps_err_.y();
            z_1_R_(3) = eps_err_.z();
            
            // calculate error omega
            omega_err_ = x_current.omega - x_des.omega;

            // calculate matrix GT (T.. transposed, means 4x3)
            matrix_g_transposed_.row(0) << sgn(eta_err_)*eps_err_.x(), sgn(eta_err_)*eps_err_.y(), sgn(eta_err_)*eps_err_.z();
            matrix_g_transposed_.block(1,0,3,3) << eta_err_, -eps_err_.z(), eps_err_.y(),
                                                  eps_err_.z(), eta_err_, -eps_err_.x(),
                                                  -eps_err_.y(), eps_err_.x(), eta_err_;
            // calculate z2
            z_2_R_ = 0.5f * matrix_g_transposed_ * omega_err_;

            // calculate first derivative of error quaternion 
            eta_dot_err_ = -0.5f * (eps_err_).dot(omega_err_);
            eps_dot_err_ = matrix_g_transposed_.block(1,0,3,3) * omega_err_;

            // calculate matrix G_dot_T (T.. transposed, means 4x3)
            matrix_g_dot_transposed_.row(0) << sgn(eta_err_)*eps_dot_err_.x(), sgn(eta_err_)*eps_dot_err_.y(), sgn(eta_err_)*eps_dot_err_.z();
            matrix_g_dot_transposed_.block(1,0,3,3) << eta_dot_err_, -eps_dot_err_.z(), eps_dot_err_.y(),
                                                  eps_dot_err_.z(), eta_dot_err_, -eps_dot_err_.x(),
                                                  -eps_dot_err_.y(), eps_dot_err_.x(), eta_dot_err_;
            
            // calculate sliding surface s
            s_R_ = z_2_R_ + lambda_R_ * z_1_R_;

            // calculate output
            u_R_ = - inertia_*matrix_g_transposed_.transpose()* 
                    ( 2*lambda_R_*z_2_R_ + matrix_g_dot_transposed_*omega_err_ + 2.0f * K_R_ * 2.0f/M_PI * (atan(s_R_.array())).matrix() ) + 
                    inertia_*x_des.omega_dot + x_current.omega.cross(inertia_*x_current.omega);
                    
            // integral sliding mode: calculate integral value
            integral_R_ = integral_R_ + 2.0f /M_PI * K_R_ * (atan(s_R_.array())).matrix() * t_delta_.toSec(); 
            // calculate integral sliding surface
            s_R_I_ = s_R_ + integral_R_;
            // calculate integral output
            u_R_I_ = - K_R_I_ * 2.0f / M_PI * ( atan( ( 0.5f*inertia_inv_.transpose()*matrix_g_transposed_.transpose() * s_R_I_ ).array())).matrix();
            
            // output is the sum of both rotational outputs (with and without integral action)
            controlForceAndTorque.u_R = u_R_ + u_R_I_; // already torque dimension
        };

    private:
        float drone_parameter_;
        float mass_;
        Eigen::Matrix3f inertia_;
        Eigen::Matrix3f inertia_inv_;
        //translational variables
        float lambda_T_;
        Eigen::Vector3f gravity_;
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

class BaseController {
    public:
        BaseController(){
            readDroneParameterFromServer();
            SlidingModeController controller (drone_parameter_);
            sliding_mode_controller_ = controller; // test if it works!?
        };
        void computeControlOutput(const PoseVelocityAcceleration& x_des, const PoseVelocityAcceleration& x_current, float spinningRates[6]){
            
            sliding_mode_controller_.computeControlForceTorqueInput(x_des, x_current, controlForceAndTorque_);

            mapControlForceTorqueInputToPropellerRates(spinningRates);
        };
       
    private:
        void readDroneParameterFromServer(){};
        void mapControlForceTorqueInputToPropellerRates(float spinningRates[6]){};
        float drone_parameter_;
        ForceTorqueInput controlForceAndTorque_;
        SlidingModeController sliding_mode_controller_;
};




BaseController* g_drone_controller_p;
PoseVelocityAcceleration g_currentPose;
PoseVelocityAcceleration g_desiredPose;

// receive trajectory message
void trajectoryMessageCallback(const trajectory_msgs::MultiDOFJointTrajectoryPoint::ConstPtr& msg){
    geometry_msgs::Transform transform = msg->transforms[0];
    geometry_msgs::Twist velocity = msg->velocities[0];
    geometry_msgs::Twist acceleration = msg->accelerations[0];

    geometry_msgs::Vector3 p_des_msg = transform.translation;
    geometry_msgs::Quaternion q_des_msg = transform.rotation;
    geometry_msgs::Vector3 p_dot_des_msg = velocity.linear;
    geometry_msgs::Vector3 omega_des_msg = velocity.angular;
    geometry_msgs::Vector3 p_ddot_des_msg = acceleration.linear;
    geometry_msgs::Vector3 omega_dot_des_msg = acceleration.angular;

    Eigen::Vector3f p_des (p_des_msg.x, p_des_msg.y, p_des_msg.z);
    Eigen::Quaternionf q_des (q_des_msg.w, q_des_msg.x, q_des_msg.y, q_des_msg.z);

    Eigen::Vector3f p_dot_des (p_dot_des_msg.x, p_dot_des_msg.y, p_dot_des_msg.z);
    Eigen::Vector3f omega_des (omega_des_msg.x, omega_des_msg.y, omega_des_msg.z);

    Eigen::Vector3f p_ddot_des (p_ddot_des_msg.x, p_ddot_des_msg.y, p_ddot_des_msg.z);
    Eigen::Vector3f omega_dot_des (omega_dot_des_msg.x, omega_dot_des_msg.y, omega_dot_des_msg.z);

    ros::Duration duration = msg->time_from_start;

    ROS_DEBUG("Received trajectory message: x_des = [%f, %f, %f], q_des = [%f, %f, %f, %f]", p_des.x(), p_des.y(), p_des.z(), q_des.w(), q_des.x(), q_des.y(), q_des.z());
    ROS_DEBUG("    Time from start: %f s", duration.toSec());

    // update global variable for desired pose
    g_desiredPose.p = p_des;
    g_desiredPose.q = q_des;
    g_desiredPose.p_dot = p_dot_des;
    g_desiredPose.omega = omega_des;
    g_desiredPose.p_ddot = p_ddot_des;
    g_desiredPose.omega_dot = omega_dot_des;

}

void computeControlOutputAndPublish(){
    float spinningRates[6]; 
    g_drone_controller_p->computeControlOutput(g_desiredPose, g_currentPose, spinningRates);
}


int main(int argc, char **argv)
{
    // initialize node
    ros::init(argc, argv, "controller");

    ros::NodeHandle n;

    // suscribe to trajectory messages
    ros::Subscriber sub = n.subscribe("trajectory", 1000, trajectoryMessageCallback);

    // create controller
    BaseController m_drone_controller;
    g_drone_controller_p = &m_drone_controller;

    //Eigen::Vector3f test;
    Eigen::Quaternionf test1 (0.2,0.7,0.3,0.1);
    Eigen::Vector3f test = test1.vec().transpose();
    Eigen::Vector3f test2 (1,2,3);
    ROS_INFO("[%f,%f,%f]", test.x(), test.y(), test.z());
    float test3 = -test.dot(test2);
    ROS_INFO("%f", test3);


    // Set up a dynamic reconfigure server following
    // https://github.com/UCSD-E4E/stingray-auv/wiki/Writing-publisher-subscriber-with-dynamic-reconfigure-and-parameter-server-(C----)
    dynamic_reconfigure::Server<flypulator_control::control_parameterConfig> dr_srv;
    dynamic_reconfigure::Server<flypulator_control::control_parameterConfig>::CallbackType cb;
    //cb = boost::bind(&NodeExample::configCallback, node_example, _1, _2);
    //dr_srv.setCallback(cb);

    ros::spin();

    return 0;
}