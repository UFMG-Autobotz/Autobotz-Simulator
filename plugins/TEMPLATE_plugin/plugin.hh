#ifndef _PID_CONTROL_PLUGIN_HH_
#define _PID_CONTROL_PLUGIN_HH_

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
		private: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg, const int joint_ID);
		private: void QueueThread();
		private: void OnUpdate();

		private: std::unique_ptr<ros::NodeHandle> rosNode;	// A node used for ROS transport
		private: std::vector<ros::Subscriber> rosSub;  // A ROS subscriber vector
		private: ros::CallbackQueue rosQueue; // A ROS callbackqueue that helps process messages
		private: std::thread rosQueueThread;// A thread the keeps running the rosQueue
		private: std::vector<ros::Publisher> rosPub;  // A ROS publisher vector
		private: event::ConnectionPtr updateConnection; // events

	};
}
#endif
