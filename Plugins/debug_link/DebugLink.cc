#include "DebugLink.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(DebugLink)


DebugLink::DebugLink() {
	// Initialize ROS
	if (!ros::isInitialized()) {
		int argc = 0;
		char **argv = NULL;
		ros::init(argc, argv, "gazebo_client5", ros::init_options::NoSigintHandler);
	}

	// Create ROS node
	this->rosNode.reset(new ros::NodeHandle("gazebo_client5"));
}

void DebugLink::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
	// error check
	if (!_model || !_sdf) {
		gzerr << "No visual or SDF element specified. Plugin won't load." << std::endl;
		return;
	}

	this->link_data.reset(new DebugDataParser(_model, _sdf));
	this->link_data->ReadVariables();
	this->setTopics();

	this->updateConnection = event::Events::ConnectWorldUpdateEnd(
		boost::bind(&DebugLink::OnUpdate, this));

}

void DebugLink::setTopics() {
	int linkCount = this->link_data->GetLinkCount();  // get number of valid links

	link_param *currentLink;

	this->rosPub_vector.resize(linkCount);
	for (int i = 0; i < linkCount; i++) {
		currentLink = this->link_data->GetLink(i);

		std::cout << currentLink->pose << std::endl;

		if(currentLink->valid) {
			if (currentLink->pose) {
			  this->rosPub_vector[i] = this->rosNode->advertise<geometry_msgs::Pose>(currentLink->postopic, 100);
			}
		}
	}

}

void DebugLink::OnUpdate() {
	int linkCount = this->link_data->GetLinkCount();  // get number of valid links
	link_param *currentLink;
	math::Pose linkPose;

	for (int i = 0; i < linkCount; i++) {
		currentLink = this->link_data->GetLink(i);
		if(currentLink->valid) {
			if (currentLink->pose) {
			  linkPose = currentLink->link->GetWorldCoGPose();

				// std::cout << linkPose.pos.x << std::endl;

				// std_msgs::Float32 teste;

				geometry_msgs::Pose currentLink_pose;

				currentLink_pose.position.x = linkPose.pos.x;
				currentLink_pose.position.y = linkPose.pos.y;
				currentLink_pose.position.z = linkPose.pos.z;

				currentLink_pose.orientation.x =linkPose.rot.x;
				currentLink_pose.orientation.y =linkPose.rot.y;
				currentLink_pose.orientation.z =linkPose.rot.z;
				currentLink_pose.orientation.w =linkPose.rot.w;

				// teste.data = 10;
				this->rosPub_vector[i].publish(currentLink_pose);
			}
		}
	}

}
