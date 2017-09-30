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
  common::PID vel_pid;
  common::PID pos_pid;
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
	physics::ModelPtr model;
	sdf::ElementPtr sdf;

  std::vector<joint_param> joints;
  bool all = false;

};

#endif
