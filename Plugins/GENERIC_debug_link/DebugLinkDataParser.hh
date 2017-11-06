#ifndef _DEBUG_LINK_DATA_PARSER_HH_
#define _DEBUG_LINK_DATA_PARSER_HH_

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

  /*-------------------*/

  typedef struct _link_param{
    bool valid; // whether this joint is valid (only revolute and prismatic joints are valid)

    std::string name; // scoped name of the link
    physics::LinkPtr link; // pointer to the link

    // bool velocity;  // whether the velocity of the link will be controlled
    bool pose;  // whether the position of the link will be controlled

    // std::string veltopic; // name of the rostopic that will control the velocity
    std::string postopic; // name of the rostopic that will control the position
  } link_param;

  /*-------------------*/

  class DebugLinkDataParser {

    public: DebugLinkDataParser(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    public: void ReadVariables();
    public: int GetLinkCount();
    // void ShowJoints();
    public: link_param *GetLink(int idx);


  	private: physics::ModelPtr model; // pointer to the parent model
  	private: sdf::ElementPtr sdf; // pointer to the sdf
    private: std::vector<link_param> links;  // vector with each valid joint data
  };

}

#endif
