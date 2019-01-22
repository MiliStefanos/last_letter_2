#include <gazebo/gazebo_client.hh> //gazebo version >6
#include <ros/ros.h>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include "ros/callback_queue.h"
#include <plane_simple/model_states.h>
#include <boost/bind.hpp>
#include <plane_simple/apply_wrench.h>
#include <plane_simple/planeForces.h>
#include <rosgraph_msgs/Clock.h>
#include <ctime> // for timer
#include <ros/service.h>
#include <iostream>

namespace gazebo
{
class add_forces_plugin : public ModelPlugin
{
	// Pointer to the model
  private:
	physics::ModelPtr model;

	// Pointer to the update event connection
	event::ConnectionPtr updateConnection; //A class that encapsulates a connection

	///  A node use for ROS transport
	ros::NodeHandle *rosNode;

	///  A ROS subscriber
	//ros::Subscriber rosSub;

	// ROS publisher
	ros::Publisher pub_;
	ros::ServiceServer srv_;

	///  A ROS callbackqueue that helps process messages
	ros::CallbackQueue rosQueue;

	///  A thread the keeps running the rosQueue
	std::thread rosQueueThread;

  public:
	add_forces_plugin() : ModelPlugin() //constructor
	{
	}

	void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) //Called when a Plugin is first created,
	{														  //and after the World has been loaded.Νot be blocking.
		this->model = _model;
		ROS_INFO("add_forces_plugin just started");

		this->rosNode = new ros::NodeHandle; //Create a ros node for transport
		while (!this->rosNode->ok())
		{
			ROS_INFO("Waiting for node to rise");
		}

		// Spin up the queue helper thread.
		this->rosQueueThread =
			std::thread(std::bind(&add_forces_plugin::QueueThread, this));
		//Connect a callback to the world update start signal.
		this->updateConnection = event::Events::ConnectWorldUpdateEnd(std::bind(&add_forces_plugin::OnUpdate, this));
		ros::AdvertiseServiceOptions so = (ros::AdvertiseServiceOptions::create<plane_simple::apply_wrench>("apply_wrench_srv",
																											boost::bind(&add_forces_plugin::add_wrench, this, _1, _2), ros::VoidPtr(), &this->rosQueue));
		this->srv_ = this->rosNode->advertiseService(so);

		// Publish code
		this->pub_ = this->rosNode->advertise<plane_simple::model_states>("plane_simple/model_states", 1000);
	}

	//  ROS helper function that processes messages
	void QueueThread()
	{
		// static const double timeout = 0.01;
		ROS_INFO(" i am in QueueThread now\n");
		while (this->rosNode->ok())
		{
			// this->rosQueue.callAvailable(ros::WallDuration(timeout));
			this->rosQueue.callAvailable();
		}
	}

	bool add_wrench(plane_simple::apply_wrench::Request &req,
			 plane_simple::apply_wrench::Response &res)
	{
		ignition::math::Vector3d force, torque;
		float thrust;

		thrust = req.planeForces.thrust;
		force[0] = thrust;
		force[1] = 0;
		force[2] = 0;
		model->GetLink("airfoil")->AddLinkForce(force);
		model->GetJoint("body_to_arm")->SetVelocity(0, thrust);

		force[0] = req.planeForces.forces[0];
		force[1] = req.planeForces.forces[1];
		force[2] = req.planeForces.forces[2];
		model->GetLink("airfoil")->AddLinkForce(force);

		torque[0] = req.planeForces.torques[0];
		torque[1] = req.planeForces.torques[1];
		torque[2] = req.planeForces.torques[2];
		model->GetLink("airfoil")->AddRelativeTorque(torque);

		return true;
	}

	void OnUpdate()
	{
		ignition::math::Vector3d relLinVel;
		relLinVel = model->GetLink("airfoil")->RelativeLinearVel();
		ignition::math::Vector3d rotation;
		rotation = model->GetLink("airfoil")->WorldPose().Rot().Euler();
		ignition::math::Vector3d relAngVel;
		relAngVel = model->GetLink("airfoil")->RelativeAngularVel();

		plane_simple::model_states model_states;
		model_states.header.stamp = ros::Time::now();
		model_states.roll = rotation[0];
		model_states.pitch = -rotation[1];
		model_states.yaw = -rotation[2];
		model_states.u = relLinVel[0];
		model_states.v = -relLinVel[1];
		model_states.w = -relLinVel[2];
		model_states.p = relAngVel[0];
		model_states.q = -relAngVel[1];
		model_states.r = -relAngVel[2];
		pub_.publish(model_states);
	}
};
// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(add_forces_plugin)
} // namespace gazebo
