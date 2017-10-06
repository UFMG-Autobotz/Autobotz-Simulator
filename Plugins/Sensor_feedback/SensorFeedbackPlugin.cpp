#ifndef _SENSOR_FEEDBACK_PLUGIN_HH_
#define _SENSOR_FEEDBACK_PLUGIN_HH_

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

namespace gazebo {

	class SensorFeedbackPlugin : public ModelPlugin {

public:
	// Constructor
	SensorFeedbackPlugin() {
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

	}

	/*-------------------*/

private:
	std::unique_ptr<ros::NodeHandle> rosNode;	/// \brief A node use for ROS transport
	std::vector<ros::Publisher> rosPub_vector;  /// \brief A ROS subscriber

	};

	// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
	GZ_REGISTER_MODEL_PLUGIN(SensorFeedbackPlugin)
}
#endif
