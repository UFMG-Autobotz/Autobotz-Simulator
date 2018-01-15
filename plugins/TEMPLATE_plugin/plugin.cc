#include "PIDControlPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(PIDControlPlugin)

/*-------------------*/

// Constructor
PIDControlPlugin::PIDControlPlugin() {
	// Initialize ROS
	if (!ros::isInitialized()) {
		int argc = 0;
		char **argv = NULL;
		ros::init(argc, argv, "gazebo", ros::init_options::AnonymousName);
	}

	// Create ROS node
	this->rosNode.reset(new ros::NodeHandle("gazebo"));
}

/*-------------------*/

void PIDControlPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
	// error check
	if (!_model || !_sdf) {
		gzerr << "No visual or SDF element specified. Plugin won't load." << std::endl;
		return;
	}


	// Spin up the queue helper thread.
	this->rosQueueThread = std::thread(std::bind(&PIDControlPlugin::QueueThread, this));

	this->updateConnection = event::Events::ConnectWorldUpdateEnd(
		boost::bind(&PIDControlPlugin::OnUpdate, this));
}


/*-------------------*/

//  Handle an incoming message from ROS
// / \param[in] _msg A float value that is used to set the velocity
// / of the VT_sim.
void PIDControlPlugin::OnRosMsg(const std_msgs::Float32ConstPtr &_msg, const int joint_ID){

}

/*-------------------*/

// ROS helper function that processes messages
void PIDControlPlugin::QueueThread() {
	static const double timeout = 0.01;
	while (this->rosNode->ok()) {
		this->rosQueue.callAvailable(ros::WallDuration(timeout));
	}
}

/*-------------------*/

void PIDControlPlugin::OnUpdate() {
	for (int idx = 0; idx < rosPub.size(); idx++) {
		this->rosPub[idx].publish(var_ROS);
	}
}
