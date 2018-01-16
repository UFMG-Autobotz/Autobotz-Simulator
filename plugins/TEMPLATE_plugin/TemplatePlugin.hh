#ifndef _TEMPLATE_PLUGIN_HH_
#define _TEMPLATE_PLUGIN_HH_

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include <ros/console.h>
#include "ros/subscribe_options.h"

#include "std_msgs/Float32.h"
// other message types:
// http://wiki.ros.org/std_msgs
// http://wiki.ros.org/common_msgs

#include <gazebo/gazebo.hh>
// #include <gazebo/physics/physics.hh>
// #include <gazebo/math/gzmath.hh>

#include <thread>
#include <iostream>

namespace gazebo {
	// model plugin example
	// other types: https://osrf-distributions.s3.amazonaws.com/gazebo/api/7.1.0/group__gazebo__common.html#ga54e0db21fd49ff6e0178b6ebe120a6d2
	class TemplatePlugin : public ModelPlugin {
		public: TemplatePlugin();
		public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
		private: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg, const int _id); // Handle an incoming message from ROS
		private: void QueueThread(); // ROS helper function that processes messages
		private: void OnUpdate();

		private: std::unique_ptr<ros::NodeHandle> rosNode;	// A node used for ROS transport
		private: std::vector<ros::Subscriber> rosSub;  // A ROS subscriber vector
		private: ros::CallbackQueue rosQueue; // A ROS callbackqueue that helps process messages
		private: std::thread rosQueueThread;// A thread the keeps running the rosQueue
		private: std::vector<ros::Publisher> rosPub;  // A ROS publisher vector
		private: event::ConnectionPtr updateConnection; // A Gazebo event
	};
}
#endif
