#ifndef _SET_MODEL_POSE_DATA_PARSER_HH_
#define _SET_MODEL_POSE_DATA_PARSER_HH_

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

  typedef struct _model_param{
    bool valid; // whether this model's position will be set

    std::string name; // scoped name of the model
    physics::ModelPtr model; // pointer to the model

    std::string topic; // name of the rostopic that will set the pose
  } model_param;

  class SetModelPoseDataParser {

    public: SetModelPoseDataParser(physics::WorldPtr _world, sdf::ElementPtr _sdf);
    public: void ReadVariables();
    public: model_param *GetModel(int idx);
    public: int GetModelCount();

  	private: physics::WorldPtr world; // pointer to the parent world
  	private: sdf::ElementPtr sdf; // pointer to the sdf
    private: std::vector<model_param> models;  // vector with each model data
  };

}

#endif
