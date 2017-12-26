#include "DebugLinkPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(DebugLinkPlugin)

/*-------------------*/

int add(int x, int y) {return x+y;}
int sub(int x, int y) {return x-y;}

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

void DebugLinkPlugin::createMap(link_param *param, int idx) {
	physics::LinkPtr link = param->link;

	this->mapsGroup1[idx]["GetRelativeTorque"] = boost::bind(&physics::Link::GetRelativeTorque, link);
	this->mapsGroup1[idx]["GetWorldTorque"] = boost::bind(&physics::Link::GetWorldTorque, link);
	this->mapsGroup1[idx]["GetRelativeAngularAccel"] = boost::bind(&physics::Link::GetRelativeAngularAccel, link);
	this->mapsGroup1[idx]["GetWorldAngularAccel"] = boost::bind(&physics::Link::GetWorldAngularAccel, link);
	this->mapsGroup1[idx]["GetRelativeAngularVel"] = boost::bind(&physics::Link::GetRelativeAngularVel, link);
	this->mapsGroup1[idx]["GetWorldAngularVel"] = boost::bind(&physics::Link::GetWorldAngularVel, link);
	this->mapsGroup1[idx]["GetRelativeForce"] = boost::bind(&physics::Link::GetRelativeForce, link);
	this->mapsGroup1[idx]["GetWorldForce"] = boost::bind(&physics::Link::GetWorldForce, link);
	this->mapsGroup1[idx]["GetRelativeLinearAccel"] = boost::bind(&physics::Link::GetRelativeLinearAccel, link);
	this->mapsGroup1[idx]["GetWorldLinearAccel"] = boost::bind(&physics::Link::GetWorldLinearAccel, link);
	this->mapsGroup1[idx]["GetRelativeLinearVel"] = boost::bind(&physics::Link::GetRelativeLinearVel, link);
	this->mapsGroup1[idx]["GetWorldLinearVel"] = boost::bind(&physics::Link::GetWorldLinearVel, link);
	this->mapsGroup1[idx]["GetWorldCoGLinearVel"] = boost::bind(&physics::Link::GetWorldCoGLinearVel, link);
	this->mapsGroup1[idx]["GetWorldAngularMomentum"] = boost::bind(&physics::Link::GetWorldAngularMomentum, link);

	this->mapsGroup2[idx]["GetRelativePose"] = boost::bind(&physics::Link::GetRelativePose, link);
	this->mapsGroup2[idx]["GetWorldPose"] = boost::bind(&physics::Link::GetWorldPose, link);
	this->mapsGroup2[idx]["GetWorldCoGPose"] = boost::bind(&physics::Link::GetWorldCoGPose, link);

	this->mapsGroup3[idx]["GetWorldEnergy"] = boost::bind(&physics::Link::GetWorldEnergy, link);
	this->mapsGroup3[idx]["GetWorldEnergyKinetic"] = boost::bind(&physics::Link::GetWorldEnergyKinetic, link);
	this->mapsGroup3[idx]["GetWorldEnergyPotential"] = boost::bind(&physics::Link::GetWorldEnergyPotential, link);
}


/*-------------------*/

void DebugLinkPlugin::setTopics() {
	int link_count, variable_count;
	link_param *current_link;
	variable_param *current_variable;
	ros::Publisher pub;

	link_count = this->link_data->GetLinkCount();  // get number of links
	this->mapsGroup1.resize(link_count);
	this->mapsGroup2.resize(link_count);
	this->mapsGroup3.resize(link_count);

	for (int i = 0; i < link_count; i++) {

		// create maps for this link
		current_link = this->link_data->GetLink(i);
		this->createMap(current_link, i);

		variable_count = this->link_data->GetVariableCount(i);
		for (int j = 0; j < variable_count; j++) {
			current_variable = this->link_data->GetVariable(i, j);

			switch(current_variable->group) {
	    	case 1 :
					pub = this->rosNode->advertise<geometry_msgs::Vector3>(current_variable->topic, 100);
					break;
	    	case 2 :
					pub = this->rosNode->advertise<geometry_msgs::Pose>(current_variable->topic, 100);
					break;
				case 3 :
					pub = this->rosNode->advertise<std_msgs::Float64>(current_variable->topic, 100);
					break;
			}

			this->rosPub_vector.push_back(pub);
		}

	}

}

/*-------------------*/

void DebugLinkPlugin::OnUpdate() {
	int link_count, variable_count;
	int idx = 0;
	variable_param *current_variable;
	ros::Publisher pub;

	link_count = this->link_data->GetLinkCount();  // get number of links


	for (int i = 0; i < link_count; i++) {

		variable_count = this->link_data->GetVariableCount(i);
		for (int j = 0; j < variable_count; j++) {
			current_variable = this->link_data->GetVariable(i, j);

			std::string funct = "Get" + current_variable->scope + current_variable->name;

			switch(current_variable->group) {
				case 1 :
				{
					math::Vector3 var_gz = this->mapsGroup1[i][funct]();
					geometry_msgs::Vector3 var_ROS;

					var_ROS.x = var_gz.x;
					var_ROS.y = var_gz.y;
					var_ROS.z = var_gz.z;

  				this->rosPub_vector[idx].publish(var_ROS);
					break;
				}
				case 2 :
				{
					math::Pose var_gz = this->mapsGroup2[i][funct]();
					geometry_msgs::Pose var_ROS;

					var_ROS.position.x = var_gz.pos.x;
					var_ROS.position.y = var_gz.pos.y;
					var_ROS.position.z = var_gz.pos.z;

					var_ROS.orientation.x = var_gz.rot.x;
					var_ROS.orientation.y = var_gz.rot.y;
					var_ROS.orientation.z = var_gz.rot.z;
					var_ROS.orientation.w = var_gz.rot.w;

					this->rosPub_vector[idx].publish(var_ROS);
					break;
				}
				case 3 :
				{
					float var_gz = this->mapsGroup3[i][funct]();
					std_msgs::Float64 var_ROS;

					var_ROS.data = var_gz;

					this->rosPub_vector[idx].publish(var_ROS);
					break;
				}
			}

			idx++;
		}

	}

}
