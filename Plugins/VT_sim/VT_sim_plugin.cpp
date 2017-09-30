#ifndef _VT_SIM_PLUGIN_HH_
#define _VT_SIM_PLUGIN_HH_

#include <iostream>
#include <vector>
#include <thread>
#include <string>
#include <sstream>
#include <algorithm>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/math/gzmath.hh>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include <ros/console.h>
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

#include "getJoints.hh"

// #include <sdf/sdf.hh>

bool invalidChar (char c){
	return !((c>=47 && c <=57) || (c>=65 && c <=90) || (c>=97 && c <=122) );
}

void validate_str(std::string & str){
	std::replace_if( str.begin(), str.end(), invalidChar, '_' );
}

namespace gazebo {

	/// \brief A plugin to control a VT_sim.
	class VT_simPlugin : public ModelPlugin {

	/// \brief Constructor
public:
	VT_simPlugin() {
	}

	/// \brief The load function is called by Gazebo when the plugin is
	/// inserted into simulation
	/// \param[in] _model A pointer to the model that this plugin is
	/// attached to.
	/// \param[in] _sdf A pointer to the plugin's SDF element.
public:
	virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
		// error check
		if (!_model || !_sdf) {
			gzerr << "No visual or SDF element specified. Plugin won't load." << std::endl;
			return;
		}

		// this->SetPIDControler();
		//
		// this->ROSCommunication();

		// this->updateConnection = event::Events::ConnectWorldUpdateEnd(
		// 	boost::bind(&VT_simPlugin::OnUpdate, this));
	}

private:
	// void SetPIDControler() {
	// 	this->jController.reset(new physics::JointController(this->model));
	// 	if (this->all) {
	// 		this->vel_pid = common::PID(this->vel_pid_gains[0], this->vel_pid_gains[1], this->vel_pid_gains[2]);
	// 		this->pos_pid = common::PID(this->pos_pid_gains[0], this->pos_pid_gains[1], this->pos_pid_gains[2]);
	//
	// 		for (int i = 0; i < this->n_joints; i++) {
	// 			std::string name = this->joints_vector[i]->GetScopedName();
	// 			gzmsg << name << std::endl;
	//
	// 			unsigned int type = this->joints_vector[i]->GetType();
	// 			if (type == REVOLUTE || type == PRISMATIC) {
	// 				this->jController->AddJoint(model->GetJoint(name));
	//
	// 				this->jController->SetVelocityPID(name, this->vel_pid);
	// 				this->jController->SetVelocityTarget(name, 0);
	//
	// 				this->jController->SetPositionPID(name, this->pos_pid);
	// 				this->jController->SetPositionTarget(name, 0);
	// 			}
	// 		}
	// 		std::cout << "------------" << std::endl;
	// 	} else {
	// 		for (int i = 0; i < this->n_joints; i++) {
	// 			this->jController->AddJoint(this->joints[i].joint);
	//
	// 			this->jController->SetVelocityPID(this->joints[i].name, this->vel_pid);
	// 			this->jController->SetVelocityTarget(this->joints[i].name, 0);
	//
	// 			this->jController->SetPositionPID(this->joints[i].name, this->pos_pid);
	// 			this->jController->SetPositionTarget(this->joints[i].name, 0);
	//
	// 		}
	// 	}
	//
	// }

	/// \brief Set the velocity of the VT_sim
	/// \param[in] _vel New target velocity
// public:void SetVelocity(const double &_vel, int joint_ID){
// 		// Set the joint's target velocity.
// 		this->jController->SetVelocityTarget(
// 			this->joints_vector[joint_ID]->GetScopedName(), _vel);
//
// 	}

// private:
// 	void ROSCommunication() {
// 		// Initialize ros, if it has not already bee initialized.
// 		if (!ros::isInitialized()) {
// 			int argc = 0;
// 			char **argv = NULL;
// 			ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
// 		}
//
// 		// Create joint ROS nodes
// 		this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
//
// 		ROS_INFO_STREAM("Creating ROS topics:" << std::endl);
//
// 		for (int i = 0; i < this->n_joints; ++i) {
// 			// Create a topic name
// 			unsigned int type = this->joints_vector[i]->GetType();
// 			if (type == REVOLUTE || type == PRISMATIC) {
// 				std::string topicName = "/" + this->model->GetName() + "/joint_vel_" + this->joints_vector[i]->GetName();
// 				validate_str(topicName);
// 				ROS_INFO_STREAM(topicName);
//
// 				// Create a named topic, and subscribe to it.
// 				ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>( topicName, 1,
// 				    boost::bind(&VT_simPlugin::OnRosMsg, this, _1, i), ros::VoidPtr(), &this->rosQueue);
//
// 				this->rosSub_vector.push_back(this->rosNode->subscribe(so));
// 			}
// 		}
// 		std::cout << "------------" << std::endl;
//
// 		// Spin up the queue helper thread.
// 		this->rosQueueThread = std::thread(std::bind(&VT_simPlugin::QueueThread, this));
// 	}

	/// \brief Handle an incoming message from ROS
	/// \param[in] _msg A float value that is used to set the velocity
	/// of the VT_sim.
// public:
// 	void OnRosMsg(const std_msgs::Float32ConstPtr &_msg, const int joint_ID){
// 		this->SetVelocity(_msg->data, joint_ID);
// 	}
//
// 	/// \brief ROS helper function that processes messages
// private:
// 	void QueueThread() {
// 		static const double timeout = 0.01;
// 		while (this->rosNode->ok()) {
// 			this->rosQueue.callAvailable(ros::WallDuration(timeout));
// 		}
// 	}
//
// public:
// 	void OnUpdate() {
// 		this->jController->Update();
// 	}





	/// \brief A PID controller for the joint.
	private: common::PID vel_pid, pos_pid;


	/// \brief A node use for ROS transport
	private: std::unique_ptr<ros::NodeHandle> rosNode;

	/// \brief A ROS subscriber
	private: std::vector<ros::Subscriber> rosSub_vector;

	/// \brief A ROS callbackqueue that helps process messages
	private: ros::CallbackQueue rosQueue;

	/// \brief A thread the keeps running the rosQueue
	private: std::thread rosQueueThread;

	private: std::unique_ptr<physics::JointController> jController;

	// events
	private: event::ConnectionPtr updateConnection;


	};

	// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
	GZ_REGISTER_MODEL_PLUGIN(VT_simPlugin)
}
#endif
