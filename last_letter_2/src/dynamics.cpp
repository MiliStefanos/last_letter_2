Dynamics::Dynamics(Model *parent)
{
    model = parent;
    Factory factory;

    int num_wings, num_motors, i;
    //Read the number of airfoils
    if (!ros::param::getCached("nWings", num_wings)) { ROS_FATAL("Invalid parameters for wings_number in param server!"); ros::shutdown(); }
    //Read the number of motors
    if (!ros::param::getCached("nMotors", num_motors)) { ROS_FATAL("Invalid parameters for motor_number in param server!"); ros::shutdown(); }

    // Create two lists
    // One for aerodynamic class of each airfoil and one for propulsion class for each motors
    for (i = 0; i < num_wings; i++)
    {
        listOfAerodynamics.push_back(factory.buildAerodynamics(model, i));
    }
    for (i = 0; i < num_motors; i++)
    {
        listOfPropulsion.push_back(factory.buildPropulsion(model, i));
    }
}

//  Calculate the new Aerodynamic forces for each airfoil
void Dynamics::calcAero()
{
    std::list<Aerodynamics *>::iterator it = listOfAerodynamics.begin();
    for (it = listOfAerodynamics.begin(); it != listOfAerodynamics.end(); ++it)
    {
        (*it)->calculationCycle();
    }
}

//Calculate the new Aerodynamic forces for each motor
void Dynamics::calcProp()
{
    std::list<Propulsion *>::iterator it = listOfPropulsion.begin();
    for (it = listOfPropulsion.begin(); it != listOfPropulsion.end(); ++it)
    {
        (*it)->calculationCycle();
    }
}