#ifndef _SET_MODEL_POSE_PLUGIN_HH_
#define _SET_MODEL_POSE_PLUGIN_HH_

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include <ros/console.h>
#include "ros/subscribe_options.h"
#include "geometry_msgs/Pose.h"

#include <thread>

#include "SetModelPoseDataParser.hh"

namespace gazebo {
	class SetModelPosePlugin : public WorldPlugin {

		public: SetModelPosePlugin();
		public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
		public: void Config();
		public: void SetPose(math::Pose &_pose, int joint_ID);
		private: void OnRosMsg(const geometry_msgs::PoseConstPtr &_msg, const int joint_ID);
		private: void QueueThread();

		private: std::unique_ptr<SetModelPoseDataParser> model_data;
		private: std::unique_ptr<ros::NodeHandle> rosNode; /// \brief A node use for ROS transport
		private: std::vector<ros::Subscriber> rosSub_vector;  /// \brief A ROS subscriber
		private: ros::CallbackQueue rosQueue; /// \brief A ROS callbackqueue that helps process messages
		private: std::thread rosQueueThread;/// \brief A thread the keeps running the rosQueue
		private: physics::WorldPtr world;
	};
}
#endif
