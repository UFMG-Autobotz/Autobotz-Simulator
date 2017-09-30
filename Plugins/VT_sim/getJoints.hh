#ifndef _VT_SIM_PLUGIN_HH_
#define _VT_SIM_PLUGIN_HH_

#include <iostream>
#include <vector>
#include <string>
#include <sstream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/math/gzmath.hh>

using namespace gazebo;

typedef struct _joint_param{
  bool valid;
  std::string name;
  physics::JointPtr joint;
  math::Vector3 vel_pid_gains;
  math::Vector3 pos_pid_gains;
  bool velocity;
  bool position;
  std::string veltopic;
  std::string postopic;
} joint_param;

class Data {
public:
  Data(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void ReadVariables();

private:
  /// \brief Pointer to the model.
	physics::ModelPtr model;

	/// \brief Pointer to the sdf.
	sdf::ElementPtr sdf;

  // pid gains
  math::Vector3 vel_pid_gains, pos_pid_gains;

  std::vector<joint_param> joints;
  bool all = false;
  int n_joints;

  /// \brief Pointer to the joint.
	private: std::vector<physics::JointPtr> joints_vector;
};

#endif
