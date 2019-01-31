#include <ros/ros.h>
#include <last_letter_2/airdata_srv.h>
#include <last_letter_2/Airdata.h>


bool calc_airdata(last_letter_2::airdata_srv::Request& req, last_letter_2::airdata_srv::Response& res )
{
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "environment");
    ros::NodeHandle nh;
	ros::ServiceServer envir_server=nh.advertiseService("last_letter_2/srv/airdata", calc_airdata);

	while(ros::ok())
	{
		ros::spin();
	}

    ros::shutdown();

    return 0;
}
