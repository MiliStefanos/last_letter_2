#include <ros/ros.h>
#include "last_letter_2/input_signals.h"
#include <last_letter_2/model_states.h>
#include <last_letter_2/control_signals.h>

class Controller
{
  public:
    ros::NodeHandle nh;
    last_letter_2::control_signals control_input_signals;
    last_letter_2::control_signals control_output_signals;
    last_letter_2::model_states model_states;

    ros::Publisher controller_output;
    ros::Subscriber sub_joystick;
    ros::Subscriber sub_model_states;
    // ros::Subscriber sub_sensor_data;


    Controller() //constructor
    {
        ROS_INFO("Starting constructor");

        //subscribers
        sub_joystick = nh.subscribe("last_letter_2/Joy_signals", 1, &Controller::get_joystick_signals, this);
        sub_model_states = nh.subscribe("last_letter_2/model_states", 1, &Controller::get_model_states, this);
        // sub_sensor_data = nh.subscribe("last_letter_2/sensor_data", 1, &Controller::get_sensor_data, this);

        //publishers
        controller_output = nh.advertise<last_letter_2::control_signals>("last_letter_2/controller_output_signals", 1000);

        ROS_INFO("Done with constructor");
    }

    void get_joystick_signals(const last_letter_2::control_signals::ConstPtr &msg)
    {
        // printf("ros get joy=%li\n", clock());
        control_input_signals.delta_a = msg->delta_a;
        control_input_signals.delta_e = msg->delta_e;
        control_input_signals.delta_r = msg->delta_r;
        control_input_signals.delta_t = msg->delta_t;
    }

    void get_model_states(const last_letter_2::model_states &msg)
    {
        //get Rotation, Linear Vel, Angular Vel
        model_states.roll = msg.roll;
        model_states.pitch = msg.pitch;
        model_states.yaw = msg.yaw;
        model_states.u = msg.u;
        model_states.v = msg.v;
        model_states.w = msg.w;
        model_states.p = msg.p;
        model_states.q = msg.q;
        model_states.r = msg.r;
        // printf("roll=%f pitch=%f    yaw=%f  u=%f    v=%f    w=%f\n",roll,pitch,yaw,u,v,w);
        make_controller_step();
    }

    // void get_sensor_data()
    // {

    // }

    void make_controller_step()
    {
        calcOutputs();
        publish_outputs();
    }

    void calcOutputs()
    {
        // calculate new outputs
    }

    void publish_outputs()
    {
        control_output_signals.delta_a = control_input_signals.delta_a;
        control_output_signals.delta_e = control_input_signals.delta_e;
        control_output_signals.delta_r = control_input_signals.delta_r;
        control_output_signals.delta_t = control_input_signals.delta_t;

        //    publish controller outputs
        controller_output.publish(control_output_signals);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller");

    Controller controller;
    while (ros::ok())
    {
        ros::spin();
    }
    ros::shutdown();

    return 0;
}
