#include "DebugLink.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(DebugLink)


DebugLink::DebugLink() {
	// Initialize ROS
	if (!ros::isInitialized()) {
		int argc = 0;
		char **argv = NULL;
		ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
	}

	// Create ROS node
	this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
}

void DebugLink::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
	// error check
	if (!_model || !_sdf) {
		gzerr << "No visual or SDF element specified. Plugin won't load." << std::endl;
		return;
	}

}
