#ifndef _PID_CONTROL_PLUGIN_HH_
#define _PID_CONTROL_PLUGIN_HH_

#include "Data.hpp"

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include <ros/console.h>
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <thread>


namespace gazebo {

	class PIDControlPlugin : public ModelPlugin {

public:
	// Constructor
	PIDControlPlugin() {
		// Initialize ROS
		if (!ros::isInitialized()) {
			int argc = 0;
			char **argv = NULL;
			ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
		}

		// Create ROS node
		this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
	}

	/*-------------------*/

	virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
		// error check
		if (!_model || !_sdf) {
			gzerr << "No visual or SDF element specified. Plugin won't load." << std::endl;
			return;
		}

		this->joint_data.reset(new Data(_model, _sdf));
		this->joint_data->ReadVariables();
		this->joint_data->ShowJoints();

		this->SetPIDControler(_model);

		// Spin up the queue helper thread.
		this->rosQueueThread = std::thread(std::bind(&PIDControlPlugin::QueueThread, this));

		this->updateConnection = event::Events::ConnectWorldUpdateEnd(
			boost::bind(&PIDControlPlugin::OnUpdate, this));
	}

	/*-------------------*/

	void SetPIDControler(physics::ModelPtr _model) {
		this->jController.reset(new physics::JointController(_model));  // create joint controller

		int jointCount = this->joint_data->GetJointCount();  // get number of valid joints
		joint_param *currentJoint;

		std::cout << std::endl << "------------------------" << std::endl;
		ROS_INFO_STREAM("Subscribing to ROS topics:");
		std::cout << "------------------------" << std::endl;

		for (int i = 0; i < jointCount; i++) {
			// add current joint to de controller
			currentJoint = this->joint_data->GetJoint(i);
			if(currentJoint->valid) {
				this->jController->AddJoint(currentJoint->joint);

				if (currentJoint->velocity) {
					// set velocity PID and set initial velocity to zero
					this->jController->SetVelocityPID(currentJoint->name, currentJoint->vel_pid);
					this->jController->SetVelocityTarget(currentJoint->name, 0);

					// create subscriber
					ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>(currentJoint->veltopic, 100,
							boost::bind(&PIDControlPlugin::OnRosMsg, this, _1, i), ros::VoidPtr(), &this->rosQueue);
					this->rosSub_vector.push_back(this->rosNode->subscribe(so));

					ROS_INFO_STREAM(currentJoint->veltopic);
				}

				if (currentJoint->position) {
					// set position PID and set initial position to zero
					this->jController->SetPositionPID(currentJoint->name, currentJoint->pos_pid);
					this->jController->SetPositionTarget(currentJoint->name, 0);

					// create subscriber
					ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>(currentJoint->postopic, 100,
							boost::bind(&PIDControlPlugin::OnRosMsg, this, _1, i), ros::VoidPtr(), &this->rosQueue);
					this->rosSub_vector.push_back(this->rosNode->subscribe(so));

					ROS_INFO_STREAM(currentJoint->postopic);
				}
			}
		}
		std::cout << "------------------------" << std::endl << std::endl;
	}

	/*-------------------*/

	// Set the joint's target velocity
	void SetVelocity(const double &_vel, int joint_ID){
		this->jController->SetVelocityTarget(
			this->joint_data->GetJoint(joint_ID)->name, _vel);
	}

	/*-------------------*/

	// Set the joint's target position
	void SetPosition(const double &_pos, int joint_ID){

			this->jController->SetPositionTarget(
				this->joint_data->GetJoint(joint_ID)->name, _pos);
	}

	/*-------------------*/

	// / \brief Handle an incoming message from ROS
	// / \param[in] _msg A float value that is used to set the velocity
	// / of the VT_sim.
	void OnRosMsg(const std_msgs::Float32ConstPtr &_msg, const int joint_ID){
		this->SetVelocity(_msg->data, joint_ID);
	}

	/*-------------------*/

	/// \brief ROS helper function that processes messages
	void QueueThread() {
		static const double timeout = 0.01;
		while (this->rosNode->ok()) {
			this->rosQueue.callAvailable(ros::WallDuration(timeout));
		}
	}

	/*-------------------*/

	void OnUpdate() {
		this->jController->Update();
	}

private:
	std::unique_ptr<Data> joint_data;
	std::unique_ptr<ros::NodeHandle> rosNode;	/// \brief A node use for ROS transport
	std::vector<ros::Subscriber> rosSub_vector;  /// \brief A ROS subscriber
	ros::CallbackQueue rosQueue; /// \brief A ROS callbackqueue that helps process messages
	std::thread rosQueueThread;/// \brief A thread the keeps running the rosQueue
	std::unique_ptr<physics::JointController> jController;
	event::ConnectionPtr updateConnection; // events

	};

	// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
	GZ_REGISTER_MODEL_PLUGIN(PIDControlPlugin)
}
#endif
