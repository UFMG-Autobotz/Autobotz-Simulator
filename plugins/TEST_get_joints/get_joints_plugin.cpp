#ifndef _JOINT_TEST_PLUGIN_HH_
#define _JOINT_TEST_PLUGIN_HH_

#include <iostream>
#include <vector>
#include <thread>
#include <string>
#include <algorithm>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// bool invalidChar (char c){
// 	return !((c>=47 && c <=57) || (c>=65 && c <=90) || (c>=97 && c <=122) );
// }
//
// void validate_str(std::string & str){
// 	std::replace_if( str.begin(), str.end(), invalidChar, '_' );
// }

namespace gazebo {

	/// \brief A plugin to control a VT_sim.
	class jointPlugin : public ModelPlugin {

	/// \brief The load function is called by Gazebo when the plugin is
	/// inserted into simulation
	/// \param[in] _model A pointer to the model that this plugin is
	/// attached to.
	/// \param[in] _sdf A pointer to the plugin's SDF element.
	public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
		// error check
		if (!_model || !_sdf) {
			gzerr << "No visual or SDF element specified. Plugin won't load." << std::endl;
			return;
		}

		this->n_joints = _model->GetJointCount();
		std::cout << "------------" << std::endl;
		gzmsg << this->n_joints << " Joints found" << std::endl;
		std::cout << "------------" << std::endl;

		if (this->n_joints == 0) {
			gzerr << "Invalid joint count, VT_sim plugin not loaded" << std::endl;
			return;
		}

		this->joints_vector = _model->GetJoints();

		for (int i = 0; i < this->n_joints; i++) {
			this->joint = this->joints_vector[i];
			gzmsg << this->joint->GetScopedName() << ": " << this->joint->GetType() << std::endl;
		}

	}

	private: std::vector<physics::JointPtr> joints_vector;
	private: physics::JointPtr joint;
	private: int n_joints;

	};

	// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
	GZ_REGISTER_MODEL_PLUGIN(jointPlugin)
}
#endif
