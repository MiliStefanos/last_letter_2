Aerodynamics * Factory::buildAerodynamics(Model *parent)
{
    printf("buildAerodynamics done\n");
    int id = 1;
    char paramMsg[50];
    sprintf(paramMsg, "airfoil%i/aerodynamicsType", id);
    if (!ros::param::getCached(paramMsg, id))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    switch (id){
        case 0: return new NoAerodynamics(parent); 
        case 1: return new StdLinearAero(parent);
        // case 2:
    }

    return new StdLinearAero(parent);
}

Propulsion * Factory::buildPropulsion(Model *parent)
{
    printf("build Propulsion done\n");
    int id = 1;
    char paramMsg[50];
    sprintf(paramMsg, "motor%i/motorType", id);
    if (!ros::param::getCached(paramMsg, id))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }
    switch (id){
        case 0: return new NoEngine(parent); 
        case 1: return new BeardEngine(parent);
        // case 2:
    }
}
