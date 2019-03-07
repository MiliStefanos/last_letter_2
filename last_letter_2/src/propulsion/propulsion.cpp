#include "noEngine.cpp"
#include "beardEngine.cpp"
#include "electricEngine.cpp"

Propulsion::Propulsion(Model *parent)
{
    model=parent;
}

void Propulsion::calcWrench()
{
    rotateWind();
    calcThrust();
    calcTorque();
}

void Propulsion::rotateWind()
{
    tf::Transform body_to_prop;
    tf::Quaternion tempQuat;
    // Construct transformation from body axes to mount frame
    body_to_prop.setOrigin(tf::Vector3(0,0,0));     //set the tranlsation
    tempQuat.setRPY(0,0,0);                 //set the rotation
    body_to_prop.setRotation(tempQuat);
    
    // Transform the relative wind from body axes to propeller axes
    tf::Vector3 bodyWind(model->u_r, model->v_r, model->w_r);
    tf::Vector3 tempVect;
    tempVect= body_to_prop*bodyWind;

    relativeWind.x = tempVect.getX();
    relativeWind.y = tempVect.getY();
    relativeWind.z = tempVect.getZ();

    normalWind= relativeWind.x;

    if (!std::isfinite(normalWind)) {ROS_FATAL("propulsion.cpp: NaN value in normalWind"); ros::shutdown();}
    if (std::fabs(normalWind)>1e+160) {ROS_FATAL("propulsion.cpp/rotateWind: normalWind over 1e+160"); ros::shutdown();}

}