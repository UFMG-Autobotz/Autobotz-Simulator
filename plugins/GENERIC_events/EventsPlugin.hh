#ifndef GAZEBO_EVENTS_PLUGIN_HH_
#define GAZEBO_EVENTS_PLUGIN_HH_

#include <gazebo/gazebo.hh>

#include "ros/ros.h"
#include <ros/console.h>
#include "std_msgs/Int8.h"

namespace gazebo {
	class EventsPlugins : public WorldPlugin {

		public: EventsPlugins();
		public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
		public: void SetPIDControler(physics::ModelPtr _model);
		private: void onResetWorld();

		private: std::unique_ptr<ros::NodeHandle> rosNode;	/// \brief A node use for ROS transport
		private: event::ConnectionPtr ResetWorld; // events
		private: ros::Publisher Pub;

	};
}

#endif
