#ifndef _DEBUG_LINK_PLUGIN_HH_
#define _DEBUG_LINK_PLUGIN_HH_

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <functional>
#include <map>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/math/gzmath.hh>

#include "ros/ros.h"
#include <ros/console.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"

#include "DebugLinkDataParser.hh"

namespace gazebo {


	class DebugLinkPlugin : public ModelPlugin {

		public: DebugLinkPlugin();
		public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
		public: void setTopics();
		public: void OnUpdate();
		public: void createMap(link_param *param, int idx);

		private: std::unique_ptr<DebugLinkDataParser> link_data;
		private: std::unique_ptr<ros::NodeHandle> rosNode; /// \brief A node use for ROS transport
		private: std::vector<ros::Publisher> rosPub_vector;  /// \brief A ROS publisher
		private: event::ConnectionPtr updateConnection; // events
		private: std::vector<std::map<std::string, std::function<math::Vector3()>>> mapsVector3;
		// private: std::vector<std::map<std::string, std::pair< gazebo::math::Vector3(physics::Link::*)() const, boost::shared_ptr<gazebo::physics::Link>*>>> mapsVector3;
		// private: std::map<std::string, std::function<gazebo::math::Vector3(physics::Link::*)()>> mapsVector3;

	};
}

#endif
