#include "DebugLinkDataParser.hh"

namespace gazebo {

  bool invalidChar (char c) {
  	return !((c>=47 && c <=57) || (c>=65 && c <=90) || (c>=97 && c <=122) );
  }

  /*-------------------*/

  void validate_str(std::string & str) {
  	std::replace_if(str.begin(), str.end(), invalidChar, '_');
  }

  /*-------------------*/

  DebugLinkDataParser::DebugLinkDataParser(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    this->model = _model;
    this->sdf = _sdf;
  }

  /*-------------------*/

  void DebugLinkDataParser::ReadVariables() {
    link_param current_link;

  	// read links
  	sdf::ElementPtr link_element;

  	int idx = 1;
  	std::ostringstream tag;
  	tag << "link" << idx;

  	while (this->sdf->HasElement(tag.str())) {
  		link_element = this->sdf->GetElementImpl(tag.str());
  		current_link.valid = true;  // by default the current link is valid, it's set to false if something is wrong

      // read specific link's name
  		if (link_element->HasAttribute("name")) {
  			std::string link_name = link_element->GetAttribute("name")->GetAsString(); // get links's name
  			current_link.link = this->model->GetLink(link_name); // get link from links's name

        // transform link on string to validate it
  			std::ostringstream validate_link;
  			validate_link << current_link.link;

  			if (validate_link.str() == "0") {
  				gzerr << link_name << " isn't a valid link name, " << tag.str() << " will be ignored!" << std::endl;
  				current_link.valid = false;
  			} else {
  				current_link.name = current_link.link->GetScopedName(); // save scoped name
  			}

  		} else {
  			gzerr << tag.str() << " doesn't have a name and will be ignored!" << std::endl;
  			current_link.valid = false;
  		}

      // read variables of the link
      this->parseVariables(current_link, link_element);

      // add current link to links vector
  		this->links.push_back(current_link);

      // increment index to test next link
  		idx++;
  		tag.str("");
  		tag << "link" << idx;
    }

  }

  /*-------------------*/

  void DebugLinkDataParser::parseVariables(link_param &link, sdf::ElementPtr &link_element) {
    variable_param current_variable;
  	sdf::ElementPtr variable_element;

    int idx = 1;
    std::ostringstream tag;
    tag << "variable" << idx;

    while (link_element->HasElement(tag.str())) {
  		variable_element = link_element->GetElementImpl(tag.str());
  		current_variable.valid = true;  // by default the current variable is valid, it's set to false if something is wrong

      // read variable
      current_variable.name = variable_element->Get<std::string>(tag.str());

      // read scope
      current_variable.scope = "world"; // default scope is world (all variables have this scope)
      if (variable_element->HasAttribute("scope")) {
        std::string scope = variable_element->GetAttribute("scope")->GetAsString();
        if (scope.compare("relative") == 0 || scope.compare("world") == 0 || scope.compare("worldCoG") == 0) {
          current_variable.scope = scope;
        }
      }

      // read topic
      if (variable_element->HasAttribute("topic")) {
        current_variable.topic = "/" + variable_element->GetAttribute("topic")->GetAsString();
      } else {  // if pose topic isn't given, use default name
        current_variable.topic = "/" + link.name + "/" + current_variable.scope + "_" + current_variable.name;
      }
      validate_str(current_variable.topic);

      // add current variable to variables vector
  		link.variables.push_back(current_variable);

      /*-------------------*/
      std::cout << link.name << std::endl;
      std::cout << current_variable.name << std::endl;
      std::cout << current_variable.scope << std::endl;
      std::cout << current_variable.topic << std::endl;
      /*-------------------*/

      // increment index to test next variable
  		idx++;
  		tag.str("");
  		tag << "variable" << idx;
    }


  }

  /*-------------------*/

  int DebugLinkDataParser::GetLinkCount() {
    return links.size();
  }

  /*-------------------*/

  link_param *DebugLinkDataParser::GetLink(int idx) {
    return &this->links[idx];
  }

}
