#include "TemplatePlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(TemplatePlugin) // change according to plugin type

/*-------------------*/

TemplatePlugin::TemplatePlugin() {
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

void TemplatePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
	// error check
	if (!_model || !_sdf) {
		gzerr << "No model or SDF element specified. Example plugin won't load." << std::endl;
		return;
	}

	// Spin up the queue helper thread.
	this->rosQueueThread = std::thread(std::bind(&TemplatePlugin::QueueThread, this));

	// Event that happens at the end of each world update
	// Other events: https://osrf-distributions.s3.amazonaws.com/gazebo/api/7.1.0/group__gazebo__event.html
	this->updateConnection = event::Events::ConnectWorldUpdateEnd(
		boost::bind(&TemplatePlugin::OnUpdate, this));

	// create subscriber(s)
	int idx = 0;
	ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>("/subscriber_topic", 100,
			boost::bind(&TemplatePlugin::OnRosMsg, this, _1, idx), ros::VoidPtr(), &this->rosQueue);
	this->rosSub.push_back(this->rosNode->subscribe(so));

	// create publisher(s)
	ros::Publisher pub = this->rosNode->advertise<std_msgs::Float32>("/publisher_topic", 100);
	this->rosPub.push_back(pub);
}

/*-------------------*/

void TemplatePlugin::OnRosMsg(const std_msgs::Float32ConstPtr &_msg, const int _id){
	std::cout << "Received message " <<  _msg->data << " from subscriber #" << _id <<  "." << std::endl;
}

/*-------------------*/

void TemplatePlugin::QueueThread() {
	static const double timeout = 0.01;
	while (this->rosNode->ok()) {
		this->rosQueue.callAvailable(ros::WallDuration(timeout));
	}
}

/*-------------------*/

void TemplatePlugin::OnUpdate() {
	float num = 0;
	std_msgs::Float32 msg;

	for (int idx = 0; idx < rosPub.size(); idx++) {
		float num =  rand() % 1000;
		msg.data = num;
		this->rosPub[idx].publish(msg);
		std::cout << "Sending " << num <<  " with publisher #" << idx << "." << std::endl;
	}
}
