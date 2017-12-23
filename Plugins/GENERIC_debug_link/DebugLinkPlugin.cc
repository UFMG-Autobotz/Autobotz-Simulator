#include "DebugLinkPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(DebugLinkPlugin)

/*-------------------*/

DebugLinkPlugin::DebugLinkPlugin() {
	// Initialize ROS
	if (!ros::isInitialized()) {
		int argc = 0;
		char **argv = NULL;
		ros::init(argc, argv, "gazebo_client5", ros::init_options::NoSigintHandler);
	}

	// Create ROS node
	this->rosNode.reset(new ros::NodeHandle("gazebo_client5"));
}

/*-------------------*/

void DebugLinkPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
	// error check
	if (!_model || !_sdf) {
		gzerr << "No visual or SDF element specified. Debug Link Plugin won't load." << std::endl;
		return;
	}

	this->link_data.reset(new DebugLinkDataParser(_model, _sdf));
	this->link_data->ReadVariables();
	this->setTopics();

	this->updateConnection = event::Events::ConnectWorldUpdateEnd(
		boost::bind(&DebugLinkPlugin::OnUpdate, this));
}

/*-------------------*/

void DebugLinkPlugin::setTopics() {
	int linkCount = this->link_data->GetLinkCount();  // get number of valid links
	int variable_count;
	link_param *currentLink;
	variable_param current_variable;
	ros::Publisher pub;

	// this->rosPub_vector.resize(linkCount);
	for (int i = 0; i < linkCount; i++) {
		currentLink = this->link_data->GetLink(i);

		variable_count = currentLink->variables.size();
		for (int j = 0; j < variable_count; j++) {
			current_variable = currentLink->variables[j];

			switch(current_variable.group) {
	    	case 1 :
					std::cout << '1' << std::endl;
					pub = this->rosNode->advertise<geometry_msgs::Vector3>(current_variable.topic, 100);
					break;
	    	case 2 :
					std::cout << '2' << std::endl;
					pub = this->rosNode->advertise<geometry_msgs::Pose>(current_variable.topic, 100);
					break;
				case 3 :
					std::cout << '3' << std::endl;
					pub = this->rosNode->advertise<std_msgs::Float64>(current_variable.topic, 100);
					break;
			}

			this->rosPub_vector.push_back(pub);
		}

	}

}

/*-------------------*/

void DebugLinkPlugin::OnUpdate() {
	// int linkCount = this->link_data->GetLinkCount();  // get number of valid links
	// link_param *currentLink;
	// math::Pose linkPose;
  //
	// for (int i = 0; i < linkCount; i++) {
	// 	currentLink = this->link_data->GetLink(i);
	// 	if(currentLink->valid && currentLink->pose) {
	// 	  linkPose = currentLink->link->GetWorldCoGPose();
  //
	// 		geometry_msgs::Pose currentLink_pose;
  //
	// 		currentLink_pose.position.x = linkPose.pos.x;
	// 		currentLink_pose.position.y = linkPose.pos.y;
	// 		currentLink_pose.position.z = linkPose.pos.z;
  //
	// 		currentLink_pose.orientation.x =linkPose.rot.x;
	// 		currentLink_pose.orientation.y =linkPose.rot.y;
	// 		currentLink_pose.orientation.z =linkPose.rot.z;
	// 		currentLink_pose.orientation.w =linkPose.rot.w;
  //
	// 		this->rosPub_vector[i].publish(currentLink_pose);
	// 	}
	// }

}
