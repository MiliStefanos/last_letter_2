#include <ros/ros.h>
#include <string>
// #include <class_config.hpp>
#include <rosgraph_msgs/Clock.h>
#include <last_letter_2/model_states_srv.h>
#include <last_letter_2/control_signals_srv.h>
#include <last_letter_2/airdata_srv.h>
#include <last_letter_2/Airdata.h>
#include <last_letter_2/Control_signals.h>
#include <last_letter_2/Model_states.h>
#include <last_letter_2/Aero_wrenches.h>
#include <last_letter_2/Prop_wrenches.h>
#include <geometry_msgs/Point.h>
#include <last_letter_2/planeForces.h>
#include <last_letter_2/apply_wrench.h>
#include <std_srvs/Empty.h>
#include <math.h>
#include <ctime>

class Aerodynamics;
class Propulsion;
class Model;
class Master;