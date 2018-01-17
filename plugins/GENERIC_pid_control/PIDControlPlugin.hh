#ifndef GAZEBO_PID_CONTROL_PLUGIN_HH_
#define GAZEBO_PID_CONTROL_PLUGIN_HH_

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include <ros/console.h>
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

#include <thread>

#include "PIDControlDataParser.hh"

namespace gazebo {
	class PIDControlPlugin : public ModelPlugin {

		public: PIDControlPlugin();
		public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
		public: void SetPIDControler(physics::ModelPtr _model);
		public: void SetVelocity(const double &_vel, int joint_ID);
		public: void SetPosition(const double &_pos, int joint_ID);
		private: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg, const int joint_ID);
		private: void QueueThread();
		private: void OnUpdate();

		private: std::unique_ptr<PIDControlDataParser> joint_data;
		private: std::unique_ptr<ros::NodeHandle> rosNode;	/// \brief A node use for ROS transport
		private: std::vector<ros::Subscriber> rosSub_vector;  /// \brief A ROS subscriber
		private: ros::CallbackQueue rosQueue; /// \brief A ROS callbackqueue that helps process messages
		private: std::thread rosQueueThread;/// \brief A thread the keeps running the rosQueue
		private: std::unique_ptr<physics::JointController> jController;
		private: event::ConnectionPtr updateConnection; // events

	};
}
#endif
