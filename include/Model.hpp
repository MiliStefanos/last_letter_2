#include <last_letter_libs.hpp>

class Model
{
  public:
	last_letter_2::Model_states model_states;
	last_letter_2::Airdata airdata;
	last_letter_2::Control_signals control_signals;
	last_letter_2::planeForces planeForces;
	
	ros::NodeHandle nh;
    ros::ServiceClient states_client;
	ros::ServiceClient control_signals_client;
	ros::ServiceClient airdata_client;
	ros::ServiceClient apply_wrench_client;
	ros::ServiceClient sim_step_client;
	ros::ServiceClient pauseGazebo;

	ros::Publisher signals_publisher;
	last_letter_2::model_states_srv states_srv;
	last_letter_2::control_signals_srv signals_srv;
	last_letter_2::airdata_srv air_data;
	last_letter_2::apply_wrench apply_wrench_srv;

	Aerodynamics aerodynamics;
	Propulsion propulsion;
	// Airdata airdata;
	Model();
    void model_step();
	void get_states();
	void get_control_signals();
	void get_airdata();
	void calc_wrenches();
	void apply_wrenches();
	void simulation_step();

};