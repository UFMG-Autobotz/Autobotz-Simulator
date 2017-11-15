#include "SetModelPosePlugin.hh"

using namespace gazebo;
GZ_REGISTER_WORLD_PLUGIN(SetModelPosePlugin)

// Constructor
SetModelPosePlugin::SetModelPosePlugin() {
	// Initialize ROS
	if (!ros::isInitialized()) {
		int argc = 0;
		char **argv = NULL;
		ros::init(argc, argv, "gazebo_client6", ros::init_options::NoSigintHandler);
	}

	// Create ROS node
	this->rosNode.reset(new ros::NodeHandle("gazebo_client6"));
}

/*-------------------*/

void SetModelPosePlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
	// error check
	if (!_world || !_sdf) {
    gzerr << "erro" << std::endl;
    return;
  }

	this->world = _world;

	this->model_data.reset(new SetModelPoseDataParser(_world, _sdf));
	this->model_data->ReadVariables();
	// this->model_data->ShowJoints();

	this->Config();

	// Spin up the queue helper thread.
	this->rosQueueThread = std::thread(std::bind(&SetModelPosePlugin::QueueThread, this));
}

/*-------------------*/

void SetModelPosePlugin::Config() {

	int modelCount = this->model_data->GetModelCount();  // get number of valid joints
	model_param *currentModel;

	std::cout << std::endl << "------------------------" << std::endl;
	ROS_INFO_STREAM("Set Model Pose subscribing to ROS topics:");
	std::cout << "------------------------" << std::endl;

	for (int i = 0; i < modelCount; i++) {
		// add current joint to de controller
		currentModel = this->model_data->GetModel(i);
		if(currentModel->valid) {
			// create subscriber
			ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Pose>(currentModel->topic, 100,
					boost::bind(&SetModelPosePlugin::OnRosMsg, this, _1, i), ros::VoidPtr(), &this->rosQueue);
			this->rosSub_vector.push_back(this->rosNode->subscribe(so));

			ROS_INFO_STREAM(currentModel->topic);
		}
	}
	std::cout << "------------------------" << std::endl << std::endl;
}

/*-------------------*/

// Set the joint's target position
void SetModelPosePlugin::SetPose(math::Pose &_pose, int model_ID) {
	this->model_data->GetModel(model_ID)->model->SetWorldPose(_pose, true, true);
}

/*-------------------*/

// / \brief Handle an incoming message from ROS
// / \param[in] _msg A float value that is used to set the velocity
// / of the VT_sim.
void SetModelPosePlugin::OnRosMsg(const geometry_msgs::PoseConstPtr &_msg, const int model_ID) {
	math::Pose pose;

	pose.pos.x = _msg->position.x;
	pose.pos.y = _msg->position.y;
	pose.pos.z = _msg->position.z;

	pose.rot.x = _msg->orientation.x;
	pose.rot.y = _msg->orientation.y;
	pose.rot.z = _msg->orientation.z;
	pose.rot.w = _msg->orientation.w;

	if (world->IsPaused()) {
		this->SetPose(pose, model_ID);
	}
}

/*-------------------*/

/// \brief ROS helper function that processes messages
void SetModelPosePlugin::QueueThread() {
	static const double timeout = 0.01;
	while (this->rosNode->ok()) {
		this->rosQueue.callAvailable(ros::WallDuration(timeout));
	}
}
