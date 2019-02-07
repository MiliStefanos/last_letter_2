#include <ros/ros.h>
#include <last_letter_2/get_control_signals_srv.h>
#include <last_letter_2/control_signals.h>
#include <string>

float delta_a = 0, delta_e = 0, delta_r = 0, delta_t = 0;
bool contr_signals_ready = false;
bool synch_sim_contr;

void getControllerSignals(const last_letter_2::control_signals::ConstPtr &msg)
{   

    delta_a = msg->delta_a; //store the control signals from controller
    delta_e = msg->delta_e;
    delta_r = msg->delta_r;
    delta_t = msg->delta_t;
    if (synch_sim_contr)
        contr_signals_ready = true;
}

bool sendControlSignals(last_letter_2::get_control_signals_srv::Request &req,
                        last_letter_2::get_control_signals_srv::Response &res)
{

    if (synch_sim_contr)
    {
        while (!contr_signals_ready && ros::ok())
        {
        } // whait controller to finish calculations and send data
        contr_signals_ready = false;
    }
    res.signals.delta_a = delta_a; //prepare to send back to simulator the controller's signals
    res.signals.delta_e = delta_e;
    res.signals.delta_r = delta_r;
    res.signals.delta_t = delta_t;
   
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_mediator");
    printf("controller_node: just start\n");

    ros::NodeHandle nh;
    ros::Subscriber controller_outputs = nh.subscribe("last_letter_2/controller_output_signals", 1000, getControllerSignals);
    ros::ServiceServer control_signals_server = nh.advertiseService("last_letter_2/control_signals", sendControlSignals);

    char paramMsg[50];
    sprintf(paramMsg, "simulation/synch_sim_contr");
    if (!ros::param::getCached(paramMsg, synch_sim_contr))
    {
        ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
        ros::shutdown();
    }

    while (ros::ok())
    {
        ros::MultiThreadedSpinner spinner(2);  //need of 2 threads. controller callback and sendControlSingals server
        spinner.spin();
    }

    ros::shutdown();

    return 0;
}
