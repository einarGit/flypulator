#include "flypulator_control/controller_interface.h"

// constructor
ControllerInterface::ControllerInterface(){

    // read drone parameters from ros parameter server
    readDroneParameterFromServer();

    // provide mass, inertia and gravity for controller
    float mass = (float) (drone_parameter_["mass"]);
    Eigen::Matrix3f inertia;
    inertia << (float) drone_parameter_["i_xx"], 0 ,0,
                0, (float) drone_parameter_["i_yy"], 0,
                0, 0, (float) drone_parameter_["i_zz"];

    float gravity = (float) drone_parameter_["gravity"];

    // precompute mapping matrix M 
    computeMappingMatrix();
    convert_force_part_to_b_.block(3,3,3,3) << 1,0,0,
                                                0,1,0,
                                                0,0,1;

    // create controller object depending on desired controller type (in controller_type_, read from parameter in readDroneParameterFromServer())
    if (controller_type_.compare("ism") == 0) // controller type ISM, create object of ism class
    {
        controller_ = new SlidingModeController(mass, inertia, gravity); // use new, otherwise object is destroyed after this function and pointer is a dead pointer
        // see also 
        // https://stackoverflow.com/questions/6337294/creating-an-object-with-or-without-new?utm_medium=organic&utm_source=google_rich_qa&utm_campaign=google_rich_qa
    } // add other controller types here with if comparisons
    else{
        // Should not come here
        ROS_ERROR("Unknown controller type in ControllerInterface class constructor, that should not happen!");
    }
};

// compute the control output from desired and current pose and save to spinningRates[6]
void ControllerInterface::computeControlOutput(const PoseVelocityAcceleration& x_des, const PoseVelocityAcceleration& x_current, Eigen::Matrix<float,6,1>& spinningRates){
    // call controller to compute Force and Torque output
    controller_->computeControlForceTorqueInput(x_des, x_current, controlForceAndTorque_);
    // map control force and torque to spinning velocities of the propellers resp. rotors
    mapControlForceTorqueInputToPropellerRates(x_current, spinningRates);
};

void ControllerInterface::readDroneParameterFromServer(){
    // try to load parameter from ros parameter server
    if (ros::param::get("/uav", drone_parameter_)){
        ROS_DEBUG("Drone parameter load successfully from parameter server");
    } 
    else {   // use default parameter values
        ROS_WARN("no drone parameter available, use default values...");
        drone_parameter_["mass"] = 10;
        drone_parameter_["i_xx"] = 0.4;
        drone_parameter_["i_yy"] = 0.4;
        drone_parameter_["i_zz"] = 0.8;
        drone_parameter_["alpha"] = 0.7854;
        drone_parameter_["beta"] = 0;
        drone_parameter_["delta_h"] = 0;
        drone_parameter_["length"] = 0.5;
        drone_parameter_["gravity"] = 9.81;
        drone_parameter_["k"] = 0.000056;
        drone_parameter_["b"] = 0.0000011;
    }
    
    // get controller type and ensure valid type ("ism" or ...)
    if (ros::param::get("/controller/type",controller_type_)){
        ROS_INFO("Controller type = %s",controller_type_.c_str());
        if ( ! (controller_type_.compare("ism") == 0 )  ) // add new controller types here with || !controller_type_.equals(<newControllerTypeAsString>)
        {
            ROS_WARN("Controller type from parameter server not known! Taking ism as default"); // add "or <newControllerType>" for new controller type
            controller_type_ = "ism";
        }
    }
    else{
        ROS_WARN("Controller type could not be read from parameter server, taking ism as default");
        controller_type_ = "ism";
    }

};

// map control force and torques to propeller spinning rates
void ControllerInterface::mapControlForceTorqueInputToPropellerRates(const PoseVelocityAcceleration& x_current, Eigen::Matrix<float,6,1>& spinningRates){
    ROS_DEBUG("map control forces and torques to propeller rates...");

    // [force, torqe] = ^B M * omega_spin
    // convert forces to body frame
    convert_force_part_to_b_.block(0,0,3,3) = x_current.q.toRotationMatrix();
    // calculate inverse mapping matrix
    map_matrix_inverse_b_ = (convert_force_part_to_b_ * map_matrix_).inverse();
    // calculate square of spinning rates
    spinningRates = map_matrix_inverse_b_ * controlForceAndTorque_;
    // calculate spinning rates with correct sign
    for (int i = 0; i<6; i++){
        if (spinningRates(i,0)>=0){
            spinningRates(i,0) = sqrt(spinningRates(i,0));
        }
        else{
            spinningRates(i,0) = - sqrt(-spinningRates(i,0));
        }
    }
};

// computes mapping matrix of spinning rates to forces/torques
void ControllerInterface::computeMappingMatrix(){
    // compute thrust directions
    float alpha;
    float beta;
    float gamma;
    Eigen::Vector3f e_r; // thrust direction
    Eigen::Vector3f mom; // drag + thrust torque
    Eigen::Vector3f r_ti; // vector from COM to i-th rotor
    float k = (float) drone_parameter_["k"];
    float b = (float) drone_parameter_["b"];
    float l = (float) drone_parameter_["length"];
    float dh = (float) drone_parameter_["delta_h"];
    // compute matrix
    for (int i = 0; i<6; i++){
        alpha = (float) drone_parameter_["alpha"] * pow(-1,i);
        beta = (float) drone_parameter_["beta"] * pow(-1,i);;
        gamma = ((float) i) * M_PI / 3.0f;
        // compute thrust direction
        e_r = Eigen::Vector3f (cos(alpha) * sin(beta) * cos(gamma) + sin(alpha)*sin(gamma),
                                  cos(alpha) * sin(beta) * sin(gamma) - sin(alpha)*cos(gamma),
                                  cos(alpha) * cos(beta));
        // compute r_ti vector;
        r_ti << l * cos(gamma), l*sin(gamma), dh ;
        // compute thrust and drag torque
        mom = k * r_ti.cross(e_r) + b * pow(-1,i+1) * e_r; 
        // save to class variable map_matrix
        map_matrix_.block(0,i,3,1) = k * e_r;   
        map_matrix_.block(3,i,3,1) = mom;
    }
}