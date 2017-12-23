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

  typedef struct _variable_param{
    bool valid; // whether this variable is valid

    std::string name;  // variable being read
    std::string scope; // scope used to read the variable

    std::string topic; // name of the rostopic that will send the variable
  } variable_param;

  /*-------------------*/

  typedef struct _link_param{
    bool valid; // whether this link is valid

    std::string name; // scoped name of the link
    physics::LinkPtr link; // pointer to the link

    std::vector<variable_param> variables;  // vector with variables being read
  } link_param;

  /*-------------------*/

  class DebugLinkDataParser {

    public: DebugLinkDataParser(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    public: void ReadVariables();
    public: void parseVariables(link_param &link, sdf::ElementPtr &link_element);
    public: int GetLinkCount();
    // void ShowJoints();
    public: link_param *GetLink(int idx);


  	private: physics::ModelPtr model; // pointer to the parent model
  	private: sdf::ElementPtr sdf; // pointer to the sdf
    private: std::vector<link_param> links;  // vector with each link data
  };

}

#endif
