#ifndef _DATA_HH_
#define _DATA_HH_

#include <iostream>
#include <vector>
#include <string>
#include <sstream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/math/gzmath.hh>

namespace gazebo {

  bool invalidChar (char c);
  void validate_str(std::string & str);

  const unsigned int REVOLUTE = 576;
  const unsigned int PRISMATIC = 1088;

  typedef struct _joint_param{
    bool valid; // whether this joint is valid (only revolute and prismatic joints are valid)

    std::string name; // scoped name of the joint
    physics::JointPtr joint; // pointer to the joint

    common::PID vel_pid;  // PID gains to control the velocity of the joint
    common::PID pos_pid; // PID gains to control the velocity of the joint

    bool velocity;  // whether the velocity of the joint will be controlled
    bool position;  // whether the position of the joint will be controlled

    std::string veltopic; // name of the rostopic that will control the velocity
    std::string postopic; // name of the rostopic that will control the position
  } joint_param;

  class Data {
  public:
    Data(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    void ReadVariables();
    void ShowJoints();

    joint_param *GetJoint(int idx);
    int GetJointCount();

  private:
  	physics::ModelPtr model; // pointer to the parent model
  	sdf::ElementPtr sdf; // pointer to the sdf

    std::vector<joint_param> joints;  // vector with each valid joint data
  };

}

#endif
