// A node that convert channels signals, to input signals for the model,
// based on what kind of model is loaded (plane, multirotor etc)

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <cstdlib>
#include <last_letter_2_msgs/joystick_input.h>
#include <last_letter_2_msgs/model_inputs.h>
#include <last_letter_2_msgs/model_states.h>
#include <last_letter_2_msgs/get_control_inputs_srv.h>

class Controller
{
private:
  ros::Publisher pub;
  ros::Subscriber sub_chan;
  ros::Subscriber sub_mod_st;
  ros::NodeHandle n;

  ros::ServiceServer get_control_inputs_service;

  int mixerid;
  int num_wings;
  int num_motors;
  int i;
  last_letter_2_msgs::joystick_input channels;
  last_letter_2_msgs::model_inputs model_inputs;
  last_letter_2_msgs::model_states model_states;

  float roll, pitch, yaw, thrust;
  float prev_thrust, prev_roll,prev_pitch, prev_yaw;

  //Create essencial variables, based on param server values.
  int aileron[4], elevator[4], rudder[4], motor_enable[4];
  int chanAileron[4], chanElevator[4], chanRudder[4], chanMotor[4];
  float wing_input_x[4], wing_input_y[4], wing_input_z[4], motor_input[4];
  float deltax_max[4], deltay_max[4], deltaz_max[4];

public:
  Controller()
  {
      //Subscribtions
      sub_chan = n.subscribe("last_letter_2/rawPWM", 1, &Controller::chan2signal, this, ros::TransportHints().tcpNoDelay());
      sub_mod_st = n.subscribe("last_letter_2/gazebo/model_states", 1, &Controller::storeStates, this, ros::TransportHints().tcpNoDelay());

      pub = n.advertise<last_letter_2_msgs::model_inputs>("last_letter_2/model_inputs", 1);

      //Servers
      get_control_inputs_service = n.advertiseService("last_letter_2/get_control_inputs_srv", &Controller::return_control_inputs, this);

      // Read the mixer type
      if (!ros::param::getCached("HID/mixerid", mixerid)) { ROS_INFO("No mixing function selected"); mixerid = 0;}
      //Read the number of airfoils
      if (!ros::param::getCached("airfoil/nWings", num_wings)) { ROS_FATAL("Invalid parameters for wings_number in param server!"); ros::shutdown();}
      //Read the number of motors
      if (!ros::param::getCached("motor/nMotors", num_motors)) { ROS_FATAL("Invalid parameters for motor_number in param server!"); ros::shutdown();}

      char paramMsg[50];

      //Load basic characteristics for each airfoil
      for (i = 0; i < num_wings; ++i)
      {
          sprintf(paramMsg, "airfoil%i/aileron", i + 1);
          if (!ros::param::getCached(paramMsg, aileron[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
          sprintf(paramMsg, "airfoil%i/elevator", i + 1);
          if (!ros::param::getCached(paramMsg, elevator[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
          sprintf(paramMsg, "airfoil%i/rudder", i + 1);
          if (!ros::param::getCached(paramMsg, rudder[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
          sprintf(paramMsg, "airfoil%i/chanAileron", i + 1);
          if (!ros::param::getCached(paramMsg, chanAileron[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
          sprintf(paramMsg, "airfoil%i/chanElevator", i + 1);
          if (!ros::param::getCached(paramMsg, chanElevator[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
          sprintf(paramMsg, "airfoil%i/chanRudder", i + 1);
          if (!ros::param::getCached(paramMsg, chanRudder[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
          sprintf(paramMsg, "airfoil%i/deltax_max", i + 1);
          if (!ros::param::getCached(paramMsg, deltax_max[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
          sprintf(paramMsg, "airfoil%i/deltay_max", i + 1);
          if (!ros::param::getCached(paramMsg, deltay_max[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
          sprintf(paramMsg, "airfoil%i/deltaz_max", i + 1);
          if (!ros::param::getCached(paramMsg, deltaz_max[i]))     { ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
      }

      //load basic characteristics for each motor
      for (i = 0; i < num_motors; ++i)
      {
          sprintf(paramMsg, "motor%i/enable", i + 1);
          if (!ros::param::getCached(paramMsg, motor_enable[i])) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
          sprintf(paramMsg, "motor%i/chanMotor", i + 1);
          if (!ros::param::getCached(paramMsg, chanMotor[i])) {ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg); ros::shutdown(); }
      }

      prev_thrust=prev_roll=prev_pitch=prev_yaw=0;
    }

    bool gluInvertMatrix( double* m,double* invOut)
    {
        double inv[16], det;
        int i;

        inv[0] = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15] + m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13] * m[7] * m[10];
        inv[4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15] - m[8] * m[7] * m[14] - m[12] * m[6] * m[11] + m[12] * m[7] * m[10];
        inv[8] = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15] + m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[9];
        inv[12] = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14] - m[8] * m[6] * m[13] - m[12] * m[5] * m[10] + m[12] * m[6] * m[9];
        inv[1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2] * m[15] - m[9] * m[3] * m[14] - m[13] * m[2] * m[11] + m[13] * m[3] * m[10];
        inv[5] = m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15] + m[8] * m[3] * m[14] + m[12] * m[2] * m[11] - m[12] * m[3] * m[10];
        inv[9] = -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15] - m[8] * m[3] * m[13] - m[12] * m[1] * m[11] + m[12] * m[3] * m[9];
        inv[13] = m[0] * m[9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14] + m[8] * m[2] * m[13] + m[12] * m[1] * m[10] - m[12] * m[2] * m[9];
        inv[2] = m[1] * m[6] * m[15] - m[1] * m[7] * m[14] - m[5] * m[2] * m[15] + m[5] * m[3] * m[14] + m[13] * m[2] * m[7] - m[13] * m[3] * m[6];
        inv[6] = -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] + m[4] * m[2] * m[15] - m[4] * m[3] * m[14] - m[12] * m[2] * m[7] + m[12] * m[3] * m[6];
        inv[10] = m[0] * m[5] * m[15] - m[0] * m[7] * m[13] - m[4] * m[1] * m[15] + m[4] * m[3] * m[13] + m[12] * m[1] * m[7] - m[12] * m[3] * m[5];
        inv[14] = -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] + m[4] * m[1] * m[14] - m[4] * m[2] * m[13] - m[12] * m[1] * m[6] + m[12] * m[2] * m[5];
        inv[3] = -m[1] * m[6] * m[11] + m[1] * m[7] * m[10] + m[5] * m[2] * m[11] - m[5] * m[3] * m[10] - m[9] * m[2] * m[7] + m[9] * m[3] * m[6];
        inv[7] = m[0] * m[6] * m[11] - m[0] * m[7] * m[10] - m[4] * m[2] * m[11] + m[4] * m[3] * m[10] + m[8] * m[2] * m[7] - m[8] * m[3] * m[6];
        inv[11] = -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] + m[4] * m[1] * m[11] - m[4] * m[3] * m[9] - m[8] * m[1] * m[7] + m[8] * m[3] * m[5];
        inv[15] = m[0] * m[5] * m[10] - m[0] * m[6] * m[9] - m[4] * m[1] * m[10] + m[4] * m[2] * m[9] + m[8] * m[1] * m[6] - m[8] * m[2] * m[5];

        det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

        if (det == 0)
            return false;

        det = 1.0 / det;

        for (i = 0; i < 16; i++)
            invOut[i] = inv[i] * det;

        return true;
    }

    //Store new joystick changes
    void chan2signal(last_letter_2_msgs::joystick_input msg)
    {
        channels = msg;
        //Choose which channel covnertion, based on model type
        switch (mixerid)
        {
        case 0: // No mixing applied
            break;
        case 1: // Airplane mixing
            // Airfoil inputs
            for (i = 0; i < num_wings; i++)
            {
                model_inputs.wing_input_x[i] = aileron[i] * (deltax_max[i] * (channels.value[chanAileron[i]] - 1500) / 500);
                model_inputs.wing_input_y[i] = elevator[i] * (deltay_max[i] * (channels.value[chanElevator[i]] - 1500) / 500);
                model_inputs.wing_input_z[i] = rudder[i] * (deltaz_max[i] * (channels.value[chanRudder[i]] - 1500) / 500);
            }
            // Motor inputs
            for (i = 0; i < num_motors; i++)
            {
                model_inputs.motor_input[i] = motor_enable[i] * ((float)(channels.value[chanMotor[i]] - 1000) / 1000);
            }
            break;
        case 2: // Quadrotor mixing
                // Motor inputs

            thrust = (float)channels.value[2] / 1000 - 1;
            roll = (float)channels.value[0] / 1000 - 1.5;
            pitch = (float)channels.value[1] / 1000 - 1.5;
            yaw = (float)channels.value[3] / 1000 - 1.5;
            // prev_thrust = thrust;
            // prev_roll = roll;
            // prev_pitch = pitch;
            // prev_yaw = yaw;
            
            break;

        default:
            ROS_FATAL("Invalid parameter for -/HID/mixerid- in param server!");
            ros::shutdown();
            break;
        }
        model_inputs.header.stamp = ros::Time::now();
    }

    void storeStates(const last_letter_2_msgs::model_states::ConstPtr& msg)
    {
        model_states.base_link_states = msg->base_link_states;
        model_states.airfoil_states = msg->airfoil_states;
        model_states.motor_states = msg->motor_states;
    }

    bool return_control_inputs(last_letter_2_msgs::get_control_inputs_srv::Request &req,
                               last_letter_2_msgs::get_control_inputs_srv::Response &res)
    {
        float k1 = 0.25;
        float l = 10;
        float k2 = 0.2;
        double M[16] = {k1, k1, k1, k1, 0, -l * k1, 0, l * k1, l * k1, 0, -l * k1, 0, -k2, k2, -k2, k2};
        double invM[16];
        switch (mixerid)
        {
        case 0: // No mixing applied
            break;
        case 1: // Airplane mixing
            if (channels.value[0] == 1500 && channels.value[1] == 1500 && channels.value[3] == 1500)
            {
                res.airfoil_inputs[0].x = -1 * model_states.base_link_states.roll;
                res.airfoil_inputs[0].y = 5 * model_states.base_link_states.pitch;
                res.airfoil_inputs[0].z = model_inputs.wing_input_z[0];
            }
            else
            {
                for (i = 0; i < num_wings; i++)
                {
                    res.airfoil_inputs[i].x = model_inputs.wing_input_x[i];
                    res.airfoil_inputs[i].y = model_inputs.wing_input_y[i];
                    res.airfoil_inputs[i].z = model_inputs.wing_input_z[i];
                }
            }
            // Motor inputs
            for (i = 0; i < num_motors; i++)
            {
                res.motor_input[i] = model_inputs.motor_input[i];
            }
            break;

        case 2: // Quadrotor mixing
            // Motor inputs
            if ((channels.value[0] <1530 && channels.value[0] >1470) && (channels.value[1] <1530 && channels.value[1] >1470) && (channels.value[3] <1530 && channels.value[3] >1470))  //no signal of joystick
            {
                roll = -2 * model_states.base_link_states.roll - 0.5*model_states.base_link_states.p;
                pitch = 2 * model_states.base_link_states.pitch +0.5*model_states.base_link_states.q;
                // thrust = 0.3*(15-model_states.base_link_states.z-model_states.base_link_states.w);
                thrust = (float)channels.value[2] / 1000 - 1;
                yaw = (float)channels.value[3] / 1000 - 1.5;
            }
            else
            {
                thrust = (float)channels.value[2] / 1000 - 1;
                roll = (float)channels.value[0] / 1000 - 1.5;
                pitch = (float)channels.value[1] / 1000 - 1.5;
                yaw = (float)channels.value[3] / 1000 - 1.5;
                prev_thrust = thrust;
                prev_roll = roll;
                prev_pitch = pitch;
                prev_yaw = yaw;
            }
            gluInvertMatrix(M, invM);
            // std::cout<< invM[0]<<" "<< invM[1]<<" "<< invM[2]<<" "<< invM[3]<<" "<<std::endl;
            // std::cout<< invM[4]<<" "<< invM[5]<<" "<< invM[6]<<" "<< invM[7]<<" "<<std::endl;
            // std::cout<< invM[8]<<" "<< invM[9]<<" "<< invM[10]<<" "<< invM[11]<<" "<<std::endl;
            // std::cout<< invM[12]<<" "<< invM[13]<<" "<< invM[14]<<" "<< invM[15]<<" "<<std::endl<<std::endl;
            
            std::cout << "motor inputs:";
            for (i = 0; i < num_motors; i++)
            {
                model_inputs.motor_input[i] = invM[i * 4] * thrust + invM[i * 4 + 1] * roll + invM[i * 4 + 2] * pitch + invM[i * 4 + 3] * yaw;
                res.motor_input[i] = model_inputs.motor_input[i];
                // if (model_inputs.motor_input[i] > 1)
                //     model_inputs.motor_input[i] = 1;
                // if (model_inputs.motor_input[i] < 0)
                //     model_inputs.motor_input[i] = 0;
                std::cout << model_inputs.motor_input[i] << " ";
            }
            std::cout << std::endl;

            break;
        default:
            ROS_FATAL("Invalid parameter for -/HID/mixerid- in param server!");
            ros::shutdown();
            break;
        }
        return true;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_node");

    Controller controller;

    //Build a thread to spin for callbacks
    ros::AsyncSpinner spinner(1); // Use 1 threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}