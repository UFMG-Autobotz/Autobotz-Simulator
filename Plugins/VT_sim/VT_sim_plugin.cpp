#ifndef _VT_SIM_PLUGIN_HH_
#define _VT_SIM_PLUGIN_HH_

#include <iostream>
#include <vector>
#include <thread>
#include <string>
#include <algorithm>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include <ros/console.h>
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

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
	public: VT_simPlugin() {
	}

	/// \brief The load function is called by Gazebo when the plugin is
	/// inserted into simulation
	/// \param[in] _model A pointer to the model that this plugin is
	/// attached to.
	/// \param[in] _sdf A pointer to the plugin's SDF element.
	public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
		// error check
		if (!_model || !_sdf) {
			gzerr << "No visual or SDF element specified. Plugin won't load." << std::endl;
			return;
		}

		// Store the model and sdf pointers for convenience.
		this->model = _model;
		this->sdf = _sdf;


		this->ReadVariables();
		this->GetJoints();
		this->SetPIDControler();

		// set initial velocity to zero
		for (int i = 0; i < this->n_joints; ++i) {
			this->SetVelocity(0, i);
		}

		this->ROSCommunication();

		this->updateConnection = event::Events::ConnectWorldUpdateEnd(
			boost::bind(&VT_simPlugin::OnUpdate, this));
	}

	private: void ReadVariables() {
		this->kp = 1;
		if (this->sdf->HasElement("pid_kp")) {
			this->kp = this->sdf->Get<double>("pid_kp");
		}

		this->ki = 0;
		if (this->sdf->HasElement("pid_ki")) {
			this->ki = this->sdf->Get<double>("pid_ki");
		}

		this->kd = 0;
		if (this->sdf->HasElement("pid_kd")) {
			this->kd = this->sdf->Get<double>("pid_kd");
		}

	}

	private: void GetJoints() {
		this->n_joints = this->model->GetJointCount();
		std::cout << "------------" << std::endl;
		gzmsg << this->n_joints << " Joints found" << std::endl;
		std::cout << "------------" << std::endl;

		if (this->n_joints == 0) {
			gzerr << "Invalid joint count, VT_sim plugin not loaded" << std::endl;
			return;
		}

		this->joints_vector = this->model->GetJoints();
		
		for (int i = 0; i < this->joints_vector.size(); i++)
			gzmsg << this->joints_vector[i]->GetType() << std::endl;

	}

	private: void SetPIDControler() {
		this->pid = common::PID(this->kp, this->ki, this->kd);

		for (int i = 0; i < this->n_joints; ++i) {
			gzmsg << this->joints_vector[i]->GetScopedName() << std::endl;
			this->model->GetJointController()->SetVelocityPID(this->joints_vector[i]->GetScopedName(), this->pid);
		}
		std::cout << "------------" << std::endl;
	}

	/// \brief Set the velocity of the VT_sim
	/// \param[in] _vel New target velocity
	public: void SetVelocity(const double &_vel, int joint_ID){
		// Set the joint's target velocity.
		this->model->GetJointController()->SetVelocityTarget(
			this->joints_vector[joint_ID]->GetScopedName(), _vel);

	}

	private: void ROSCommunication() {
		// Initialize ros, if it has not already bee initialized.
		if (!ros::isInitialized()) {
			int argc = 0;
			char **argv = NULL;
			ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
		}

		// Create joint ROS nodes
		this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

		ROS_INFO_STREAM("Creating ROS topics:" << std::endl);

		for (int i = 0; i < this->n_joints; ++i) {
			// Create a topic name
			std::string topicName = "/" + this->model->GetName() + "/joint_vel_" + this->joints_vector[i]->GetName();
			validate_str(topicName);
			ROS_INFO_STREAM(topicName);

			// Create a named topic, and subscribe to it.
			ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>( topicName, 1,
			    boost::bind(&VT_simPlugin::OnRosMsg, this, _1, i), ros::VoidPtr(), &this->rosQueue);

			this->rosSub_vector.push_back(this->rosNode->subscribe(so));
		}
		std::cout << "------------" << std::endl;

		// Spin up the queue helper thread.
		this->rosQueueThread = std::thread(std::bind(&VT_simPlugin::QueueThread, this));
	}

	/// \brief Handle an incoming message from ROS
	/// \param[in] _msg A float value that is used to set the velocity
	/// of the VT_sim.
	public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg, const int joint_ID){
		this->SetVelocity(_msg->data, joint_ID);
	}

	/// \brief ROS helper function that processes messages
	private: void QueueThread() {
		static const double timeout = 0.01;
		while (this->rosNode->ok()) {
			this->rosQueue.callAvailable(ros::WallDuration(timeout));
		}
	}

	public: void OnUpdate() {
		this->model->GetJointController()->Update();
	}

	/// \brief Pointer to the model.
	private: physics::ModelPtr model;

	/// \brief Pointer to the sdf.
	private: sdf::ElementPtr sdf;

	/// \brief Pointer to the joint.
	private: std::vector<physics::JointPtr> joints_vector;

	private: int n_joints;

	/// \brief A PID controller for the joint.
	private: common::PID pid;

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

	// pid gains
	private: double kp, ki, kd;
	};

	// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
	GZ_REGISTER_MODEL_PLUGIN(VT_simPlugin)
}
#endif
