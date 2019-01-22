#include <ros/ros.h>
#include <cstdlib>
#include "last_letter_2/InputSignals.h"
#include <last_letter_2/planeForces.h>
#include <last_letter_2/model_states.h>
#include "last_letter_2/apply_wrench.h"
#include "ros/service.h"
#include <math.h>
#include <rosgraph_msgs/Clock.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <ctime>
#include <std_srvs/Empty.h>
#include <std_msgs/Int8.h>
#include <gazebo_msgs/GetLinkState.h>
#include <gazebo_msgs/LinkState.h>

class force_calculator
{
  public:
	float s_prop = 0.2027;
	float k_motor = 80;
	float k_t_p = 0.0;
	float k_omega = 20.0;
	float c_prop = 1.0;
	float rho = 1.2250;
	float airspeed = 1; /// stable ideal value, need fix
	float qbar, b_2Va;
	float delta_a = 0, delta_e = 0, delta_r = 0, delta_t = 0;
	float roll, pitch, yaw = 0;
	float alpha = 0, beta = 0;
	float p, q, r;
	float l, m, n;
	float u, v, w;
	float c_drag_q, c_drag_deltae, c_drag_p, c_drag_0;
	float c_lift_0, c_lift_deltae, c_lift_q, c_lift_a;
	float b, c, s;
	float c_y_0, c_y_b, c_y_p, c_y_r, c_y_deltaa, c_y_deltar;
	float c_l_0, c_l_p, c_l_b, c_l_r, c_l_deltaa, c_l_deltar;
	float c_n_0, c_n_p, c_n_b, c_n_r, c_n_deltaa, c_n_deltar;
	float c_m_0, c_m_a, c_m_q, c_m_deltae;
	float ca, sa, lift, drag, fy = 0;
	float M = 50;
	float oswald;
	float a0;
	clock_t t1;

	geometry_msgs::Quaternion quat;
	geometry_msgs::Vector3 Va;
	ros::NodeHandle nh;
	bool flagok = false;

	last_letter_2::planeForces planeForces;
	last_letter_2::apply_wrench srv;
	ros::Publisher pub1;
	ros::Publisher pub2;
	ros::ServiceClient wrench_client;
	ros::ServiceClient step_client;
	ros::ServiceClient client3;
	ros::ServiceClient pauseGazebo;
	ros::Subscriber sub_basic_signals;
	ros::Subscriber sub_model_states;
	ros::Subscriber sub3;
	int gazeboFlag;

	force_calculator() //constructor
	{
		ROS_INFO("Starting constructor");

		//subscribers
		sub_basic_signals = nh.subscribe("last_letter_2/InputSignals", 1, &force_calculator::store_rpy_thr, this);
		sub_model_states = nh.subscribe("last_letter_2/model_states", 1, &force_calculator::model_states, this);

		//publishers
		pub1 = nh.advertise<last_letter_2::planeForces>("last_letter_2/planeForces", 1000);

		//services
		//srv for sending wrench to gazebo
		ros::service::waitForService("apply_wrench_srv");
		wrench_client = nh.serviceClient<last_letter_2::apply_wrench>("apply_wrench_srv", true);
		//srv for give step to gazebo
		ros::service::waitForService("step");
		step_client = nh.serviceClient<last_letter_2::apply_wrench>("step", true);
		//srv for pause gazebo
		ros::service::waitForService("/gazebo/pause_physics");
		pauseGazebo = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
		std_srvs::Empty emptySrv;
		pauseGazebo.call(emptySrv); //pause gazebo simulator

		init_param();

		ROS_INFO("Done with constructor");
	}

	void store_rpy_thr(const last_letter_2::InputSignals::ConstPtr &msg)
	{
		// printf("ros get joy=%li\n", clock());
		delta_a = msg->roll;
		delta_e = msg->pitch;
		delta_r = msg->yaw;
		delta_t = msg->throttle;
	}

	void model_states(const last_letter_2::model_states &msg)
	{
		//get Rotation, Linear Vel, Angular Vel
		roll = msg.roll;
		pitch = msg.pitch;
		yaw = msg.yaw;
		u = msg.u;
		v = msg.v;
		w = msg.w;
		p = msg.p;
		q = msg.q;
		r = msg.r;
		// printf("roll=%f	pitch=%f	yaw=%f	u=%f	v=%f	w=%f\n",roll,pitch,yaw,u,v,w);

		calc_forces();
	}

	void calc_forces()
	{
		// airspeed, alpha, beta
		airspeed = sqrt(pow(u, 2) + pow(v, 2) + pow(w, 2));
		alpha = atan2(w, u);
		if (u == 0)
		{
			if (v == 0)
			{
				beta = 0;
			}
			else
			{
				beta = asin(v / abs(v));
			}
		}
		else
		{
			beta = atan2(v, u);
		}

		qbar = 0.5 * rho * pow(airspeed, 2) * s;

		float sigmoid = (1 + exp(-M * (alpha - a0)) + exp(M * (alpha + a0))) / (1 + exp(-M * (alpha - a0))) / (1 + exp(M * (alpha + a0)));
		if (isnan(sigmoid))
			sigmoid = 0;
		float linear = (1.0 - sigmoid) * (c_lift_0 + c_lift_a * alpha);							//Lift at small AoA
		float flatPlate = sigmoid * (2 * copysign(1, alpha) * pow(sin(alpha), 2) * cos(alpha)); //Lift beyond stall
		float c_lift_alpha = linear + flatPlate;

		float AR = pow(b, 2) / s;
		float c_drag_alpha = c_drag_p + pow(c_lift_0 + c_lift_a * alpha, 2) / (M_PI * oswald * AR);

		ca = cos(alpha);
		sa = sin(alpha);

		// force calculation - - - - - - - - - - -expressed to body frame
		if (airspeed == 0)
		{
			drag = 0;
			lift = 0;
			fy = 0;
		}
		else
		{
			drag = qbar * ((-c_drag_alpha * ca + c_lift_alpha * sa) + (-c_drag_q * ca + c_lift_q * sa) * 0.5 / airspeed * c * q + (-c_drag_deltae * ca + c_lift_deltae * sa) * delta_e);
			lift = qbar * ((-c_drag_alpha * sa - c_lift_alpha * ca) + (-c_drag_q * sa - c_lift_q * ca) * 0.5 / airspeed * c * q + (-c_drag_deltae * sa - c_lift_deltae * ca) * delta_e);
			fy = qbar * (c_y_0 + c_y_b * beta + c_y_p * b / 2 / airspeed * p + c_y_r * b / 2 / airspeed * r + c_y_deltaa * delta_a + c_y_deltar * delta_r);
		}
		// torque calculation - - - - - - - - - - -expressed to body frame
		if (airspeed == 0)
		{
			l = 0;
			m = 0;
			n = 0;
		}
		else
		{
			l = qbar * (b * (c_l_0 + c_l_b * beta + c_l_p * b / 2 / airspeed * p + c_l_r * b / 2 / airspeed * r + c_l_deltaa * delta_a + c_l_deltar * delta_r));
			m = qbar * (c * (c_m_0 + c_m_a * alpha + c_m_q * c / 2 / airspeed * q + c_m_deltae * delta_e));
			n = qbar * (b * (c_n_0 + c_n_b * beta + c_n_p * b / 2 / airspeed * p + c_n_r * b / 2 / airspeed * r + c_n_deltaa * delta_a + c_n_deltar * delta_r));
		}
		l += -k_t_p * pow((k_omega * delta_t), 2);

		//propulsion
		float thrust = 1.0 / 2.0 * rho * s_prop * c_prop * (pow(delta_t * k_motor, 2) - pow(airspeed, 2));

		planeForces.thrust = thrust; // y,z values with opposite sign
		planeForces.forces[0] = drag;
		planeForces.forces[1] = -fy;
		planeForces.forces[2] = -lift;
		planeForces.torques[0] = l;
		planeForces.torques[1] = -m;
		planeForces.torques[2] = -n;

		// prepare service to call - - - - - - - - - -
		srv.request.planeForces = planeForces;

		if (wrench_client.isValid())
		{
			if (wrench_client.call(srv))
			{
				// ROS_INFO("succeed service call\n");
			}
			else
			{
				ROS_ERROR("Failed to call service apply_wrench_srv\n");
			}
		}
		else
		{
			ROS_ERROR("Service down, waiting reconnection...");
			wrench_client.waitForExistence();
			//		    connectToClient(); //Why this??
		}

		// if (step_client.isValid())
		// {
		// 	// printf("ros: call gazebo plugin srv to step\n");
		// 	if (step_client.call(srv))
		// 	{
		// 		// printf("persistent connection at %f sec\n",double(clock()-t1)/CLOCKS_PER_SEC);

		// 		// ROS_INFO("succeed service call\n");
		// 	}
		// 	else
		// 	{
		// 		ROS_ERROR("Failed to call service apply_wrench_srv\n");
		// 		//			break;
		// 	}
		// 	// loop_rate.sleep();
		// }
		// else
		// {
		// 	ROS_ERROR("Service down, waiting reconnection...");
		// 	step_client.waitForExistence();
		// 	//		    connectToClient(); //Why this??
		// }
	}

	void init_param()
	{
		int id = 1;
		char paramMsg[50];
		sprintf(paramMsg, "airfoil%i/c_drag_p", id);
		if (!ros::param::getCached(paramMsg, c_drag_p))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/c_drag_deltae", id);
		if (!ros::param::getCached(paramMsg, c_drag_deltae))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/c_drag_q", id);
		if (!ros::param::getCached(paramMsg, c_drag_q))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/c_drag_0", id);
		if (!ros::param::getCached(paramMsg, c_drag_0))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/c_lift_0", id);
		if (!ros::param::getCached(paramMsg, c_lift_0))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/c_lift_deltae", id);
		if (!ros::param::getCached(paramMsg, c_lift_deltae))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/c_lift_q", id);
		if (!ros::param::getCached(paramMsg, c_lift_q))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/c_lift_a", id);
		if (!ros::param::getCached(paramMsg, c_lift_a))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/alpha_stall", id);
		if (!ros::param::getCached(paramMsg, a0))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/s", id);
		if (!ros::param::getCached(paramMsg, s))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/b", id);
		if (!ros::param::getCached(paramMsg, b))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/c", id);
		if (!ros::param::getCached(paramMsg, c))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/c_y_0", id);
		if (!ros::param::getCached(paramMsg, c_y_0))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/c_y_b", id);
		if (!ros::param::getCached(paramMsg, c_y_b))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/c_y_p", id);
		if (!ros::param::getCached(paramMsg, c_y_p))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/c_y_r", id);
		if (!ros::param::getCached(paramMsg, c_y_r))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/c_y_deltaa", id);
		if (!ros::param::getCached(paramMsg, c_y_deltaa))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/c_y_deltar", id);
		if (!ros::param::getCached(paramMsg, c_y_deltar))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/c_l_0", id);
		if (!ros::param::getCached(paramMsg, c_l_0))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/c_l_p", id);
		if (!ros::param::getCached(paramMsg, c_l_p))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/c_l_b", id);
		if (!ros::param::getCached(paramMsg, c_l_b))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/c_l_r", id);
		if (!ros::param::getCached(paramMsg, c_l_r))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/c_l_deltaa", id);
		if (!ros::param::getCached(paramMsg, c_l_deltaa))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/c_l_deltar", id);
		if (!ros::param::getCached(paramMsg, c_l_deltar))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/c_m_0", id);
		if (!ros::param::getCached(paramMsg, c_m_0))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/c_m_a", id);
		if (!ros::param::getCached(paramMsg, c_m_a))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/c_m_q", id);
		if (!ros::param::getCached(paramMsg, c_m_q))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/c_m_deltae", id);
		if (!ros::param::getCached(paramMsg, c_m_deltae))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/c_n_0", id);
		if (!ros::param::getCached(paramMsg, c_n_0))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/c_n_b", id);
		if (!ros::param::getCached(paramMsg, c_n_b))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/c_n_p", id);
		if (!ros::param::getCached(paramMsg, c_n_p))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/c_n_r", id);
		if (!ros::param::getCached(paramMsg, c_n_r))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/c_n_deltaa", id);
		if (!ros::param::getCached(paramMsg, c_n_deltaa))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/c_n_deltar", id);
		if (!ros::param::getCached(paramMsg, c_n_deltar))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
		sprintf(paramMsg, "airfoil%i/oswald", id);
		if (!ros::param::getCached(paramMsg, oswald))
		{
			ROS_FATAL("Invalid parameters for -%s- in param server!", paramMsg);
			ros::shutdown();
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "basics2forces");

	force_calculator plane_model;
	while (ros::ok())
	{
		ros::spin();
	}
	ros::shutdown();

	return 0;
}
