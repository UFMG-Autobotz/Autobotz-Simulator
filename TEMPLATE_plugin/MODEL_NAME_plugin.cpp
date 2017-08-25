#ifndef _MODEL_NAME_PLUGIN_HH_
#define _MODEL_NAME_PLUGIN_HH_

#include <vector>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

namespace gazebo{
	/// \brief A plugin to control a MODEL_NAME.
	class MODEL_NAMEPlugin : public ModelPlugin{
		/// \brief Constructor
		public: MODEL_NAMEPlugin() {}

		/// \brief The load function is called by Gazebo when the plugin is
		/// inserted into simulation
		/// \param[in] _model A pointer to the model that this plugin is
		/// attached to.
		/// \param[in] _sdf A pointer to the plugin's SDF element.
		public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
			// Safety check

			this->n_joints = _model->GetJointCount();
			std::cerr << "\n";
			std::cerr << this->n_joints;
			std::cerr << " Joints found\n\n";

			if (this->n_joints == 0){
				std::cerr << "Invalid joint count, MODEL_NAME plugin not loaded\n";
				return;
			}

			// Store the model pointer for convenience.
			this->model = _model;

			// Get the first joint. We are making an assumption about the model
			// having one joint that is the rotational joint.
			this->joints_vector = _model->GetJoints();

			for (int i = 0; i < this->n_joints; ++i){
				std::cerr << this->joints_vector[i]->GetScopedName();
				std::cerr << "\n";
				// Setup a P-controller, with a gain of 0.1.
				this->pid_vector.push_back(common::PID(0.1, 0, 0));
				// Apply the P-controller to the joint.
				this->model->GetJointController()->SetVelocityPID(this->joints_vector[i]->GetScopedName(), this->pid_vector.back());
			}
			std::cerr << "\n";

			// Default to zero velocity
			double velocity = 0;

			// Check that the velocity element exists, then read the value
			if (_sdf->HasElement("velocity"))
				velocity = _sdf->Get<double>("velocity");

			for (int i = 0; i < this->n_joints; ++i){
				this->SetVelocity(velocity, i);
			}

			// Create the node
			this->node = transport::NodePtr(new transport::Node());
			#if GAZEBO_MAJOR_VERSION < 8
			this->node->Init(this->model->GetWorld()->GetName());
			#else
			this->node->Init(this->model->GetWorld()->Name());
			#endif

			for (int i = 0; i < this->n_joints; ++i){
				// Create a topic name
				std::string topicName = "~/" + this->model->GetName() + "/joint_vel" + this->joints_vector[i]->GetScopedName();

				// TODO: use boost bind to pass the ID to the callback, in order to differentiate the joints dynamically

				// Subscribe to the topic, and register a callback
				this->sub_vector.push_back(this->node->Subscribe(topicName, &MODEL_NAMEPlugin::OnMsg, this));
			}

			// Initialize ros, if it has not already bee initialized.
			if (!ros::isInitialized()){
				int argc = 0;
				char **argv = NULL;
				ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
			}

			// Create our ROS node. This acts in a similar manner to
			// the Gazebo node
			this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

			for (int i = 0; i < this->n_joints; ++i){
				// Create a named topic, and subscribe to it.
				ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>(
					"/" + this->model->GetName() + "/jointtopic_vel" + this->joints_vector[i]->GetScopedName(),
					1, boost::bind(&MODEL_NAMEPlugin::OnRosMsg, this, _1), ros::VoidPtr(), &this->rosQueue);
				
				this->rosNode->subscribe(so); // ESSA LINHA NÃO EXISTE, APENAS DEBUG. DA PROBLEMA NESSA PARTE NA LINHA DE BAIXO
				
				// this->rosSub_vector.push_back(this->rosNode->subscribe(so));
			}

			// Spin up the queue helper thread.
			this->rosQueueThread = std::thread(std::bind(&MODEL_NAMEPlugin::QueueThread, this));
		}

		/// \brief Set the velocity of the MODEL_NAME
		/// \param[in] _vel New target velocity
		public: void SetVelocity(const double &_vel, int joint_ID){
			// Set the joint's target velocity.
			this->model->GetJointController()->SetVelocityTarget(this->joints_vector[joint_ID]->GetScopedName(), _vel);
		}

		/// \brief Handle incoming message
		/// \param[in] _msg Repurpose a vector3 message. This function will
		/// only use the x component.
		private: void OnMsg(ConstVector3dPtr &_msg){
			for (int i = 0; i < this->n_joints; ++i){
				this->SetVelocity(_msg->x(), i);
			}
		}

		/// \brief Handle an incoming message from ROS
		/// \param[in] _msg A float value that is used to set the velocity
		/// of the MODEL_NAME.
		public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg){
			for (int i = 0; i < this->n_joints; ++i){
				this->SetVelocity(_msg->data, i);
			}
		}

		/// \brief ROS helper function that processes messages
		private: void QueueThread(){
			static const double timeout = 0.01;
			while (this->rosNode->ok()){
				this->rosQueue.callAvailable(ros::WallDuration(timeout));
			}
		}

		/// \brief A node used for transport
		private: transport::NodePtr node;

		/// \brief A subscriber to a named topic.
		private: std::vector<transport::SubscriberPtr> sub_vector;

		/// \brief Pointer to the model.
		private: physics::ModelPtr model;

		/// \brief Pointer to the joints.
		private: std::vector<physics::JointPtr> joints_vector;
		// private: physics::JointPtr joint;

		private: int n_joints;

		/// \brief A PID controller for the joint.
		private: std::vector<common::PID> pid_vector;
		// private: common::PID pid;

		/// \brief A node use for ROS transport
		private: std::unique_ptr<ros::NodeHandle> rosNode;

		/// \brief A ROS subscriber
		private: std::vector<ros::Subscriber> rosSub_vector;

		/// \brief A ROS callbackqueue that helps process messages
		private: ros::CallbackQueue rosQueue;

		/// \brief A thread the keeps running the rosQueue
		private: std::thread rosQueueThread;
	};

	// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
	GZ_REGISTER_MODEL_PLUGIN(MODEL_NAMEPlugin)
}
#endif