#ifndef _DEBUG_LINK_PLUGIN_HH_
#define _DEBUG_LINK_PLUGIN_HH_

#include <iostream>
#include <vector>
#include <string>
#include <sstream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/math/gzmath.hh>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include <ros/console.h>
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <thread>

#include "Data.hpp"

namespace gazebo {
	class DebugLink : public ModelPlugin {

		public: DebugLink();
		public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
		private: void setTopics();
		private: void OnUpdate();

		private: std::unique_ptr<Data> link_data;
		private: std::unique_ptr<ros::NodeHandle> rosNode;	/// \brief A node use for ROS transport
		private: std::vector<ros::Publisher> rosPub_vector;  /// \brief A ROS publisher
		private: event::ConnectionPtr updateConnection; // events

	};
}

#endif
