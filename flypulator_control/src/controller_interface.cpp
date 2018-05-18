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
void ControllerInterface::computeControlOutput(const PoseVelocityAcceleration& x_des, const PoseVelocityAcceleration& x_current, float spinningRates[6]){
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
// TODO implement
void ControllerInterface::mapControlForceTorqueInputToPropellerRates(const PoseVelocityAcceleration& x_current, float spinningRates[6]){
    ROS_DEBUG("map control forces and torques to propellor rates...");
    for (int i = 0; i<6; i++){
        spinningRates[i] = 100 + (float) i;
    }
};