#include <ros/ros.h>
#include <cstdlib>
#include <last_letter_2/joystick_input.h>
#include <last_letter_2/InputSignals.h>
#include <last_letter_2/Control_signals.h>


ros::Publisher pub;
ros::Publisher control_signals_pub;

int chanAileron, chanElevator, chanRudder, chanThrottle;

int id = 1;
double deltaa_max, deltae_max, deltar_max;

last_letter_2::InputSignals InputSignals;
last_letter_2::Control_signals control_signals;

void chan2input(last_letter_2::joystick_input msg)
{
	//Convert PPM to radians
	InputSignals.roll = deltaa_max * (double)(msg.value[chanAileron] - 1500) / 500;
	InputSignals.pitch = deltae_max * (double)(msg.value[chanElevator] - 1500) / 500;
	InputSignals.yaw = deltar_max * (double)(msg.value[chanRudder] - 1500) / 500;
	InputSignals.throttle = (double)(msg.value[chanThrottle] - 1000) / 1000;

	control_signals.delta_a = deltaa_max * (double)(msg.value[chanAileron] - 1500) / 500;
	control_signals.delta_e = deltae_max * (double)(msg.value[chanElevator] - 1500) / 500;
	control_signals.delta_r = deltar_max * (double)(msg.value[chanRudder] - 1500) / 500;
	control_signals.delta_t = (double)(msg.value[chanThrottle] - 1000) / 1000;

	pub.publish(InputSignals);
	control_signals_pub.publish(control_signals);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "chan2rpy");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("rawPWM", 1, chan2input);
	pub = n.advertise<last_letter_2::InputSignals>("last_letter_2/InputSignals", 1);
	control_signals_pub = n.advertise<last_letter_2::Control_signals>("last_letter_2/Joy_signals", 1);


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
