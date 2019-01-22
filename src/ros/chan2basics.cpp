#include <ros/ros.h>
#include <cstdlib>
#include <plane_simple/joystick_input.h>
#include <plane_simple/InputSignals.h>



ros::Publisher pub;

int chanAileron, chanElevator, chanRudder, chanThrottle;



int id=1;
double deltaa_max, deltae_max, deltar_max;

plane_simple::InputSignals InputSignals;

void chan2input(plane_simple::joystick_input msg)
{
	//Convert PPM to radians
	InputSignals.roll= deltaa_max * (double)(msg.value[chanAileron]-1500)/500;
	InputSignals.pitch= deltae_max * (double)(msg.value[chanElevator]-1500)/500;
	InputSignals.yaw= deltar_max * (double)(msg.value[chanRudder]-1500)/500;
	InputSignals.throttle = (double)(msg.value[chanThrottle]-1000)/1000;

	pub.publish(InputSignals);
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "chan2rpy");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("rawPWM",1,chan2input);
	pub = n.advertise<plane_simple::InputSignals>("plane_simple/InputSignals",1);

	char paramMsg[50];
	sprintf(paramMsg, "airfoil%i/deltaa_max", id);
	ros::param::getCached(paramMsg, deltaa_max);
	sprintf(paramMsg, "airfoil%i/deltae_max", id);
	ros::param::getCached(paramMsg, deltae_max);
	sprintf(paramMsg, "airfoil%i/deltar_max", id);
	ros::param::getCached(paramMsg, deltar_max);


	sprintf(paramMsg, "airfoil%i/chanAileron", id);
	ros::param::getCached(paramMsg, chanAileron);
	sprintf(paramMsg, "airfoil%i/chanElevator", id);
	ros::param::getCached(paramMsg, chanElevator);
	sprintf(paramMsg, "airfoil%i/chanRudder", id);
	ros::param::getCached(paramMsg, chanRudder);
	sprintf(paramMsg, "airfoil%i/chanThrottle", id);
	ros::param::getCached(paramMsg, chanThrottle);

	// Enter spin
	while (ros::ok())
	{
		ros::spin();
	}

	return 0;
}
