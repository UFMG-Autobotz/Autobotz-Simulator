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

  this->pub = this->rosNode->advertise<std_msgs::Float32>("lala", 100);
}

void DebugLink::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
	std::cout << "oi" << std::endl;

	std_msgs::Float32 teste;
	teste.data = 10;
	this->pub.publish(teste);

	// error check
	if (!_model || !_sdf) {
		gzerr << "No visual or SDF element specified. Plugin won't load." << std::endl;
		return;
	}

	this->link_data.reset(new Data(_model, _sdf));
	this->link_data->ReadVariables();
	this->setTopics();

	this->updateConnection = event::Events::ConnectWorldUpdateEnd(
		boost::bind(&DebugLink::OnUpdate, this));

}

void DebugLink::setTopics() {
	// int linkCount = this->link_data->test();  // get number of valid links

	// link_param *currentLink;
	//
	// this->rosPub_vector.resize(linkCount);
	// for (int i = 0; i < linkCount; i++) {
	// 	currentLink = this->link_data->GetLink(i);
	//
	// 	if(currentLink->valid) {
	// 		if (currentLink->position) {
	// 		  this->rosPub_vector[i] = this->rosNode->advertise<std_msgs::Float32>(currentLink->postopic, 100);
	// 		}
	// 	}
	// }

}

void DebugLink::OnUpdate() {
	// int linkCount = this->link_data->GetLinkCount();  // get number of valid links
	// link_param *currentLink;
	// math::Pose linkPose;
	//
	// for (int i = 0; i < linkCount; i++) {
	// 	currentLink = this->link_data->GetLink(i);
	// 	if(currentLink->valid) {
	// 		if (currentLink->position) {
	// 		  linkPose = currentLink->link->GetWorldCoGPose();
	//
	//
	// 			std_msgs::Float32 teste;
	// 			teste.data = 10;
	// 			this->rosPub_vector[i].publish(teste);
	// 		}
	// 	}
	// }

}
