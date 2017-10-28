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
    link_param currentLink;

    // read global velocity (if false or not set, velocity won't be controlled)
    // bool global_velocity = false;
    // if (this->sdf->HasElement("velocity")) {
    //   global_velocity = this->sdf->Get<bool>("velocity");
    // }

    // read global pose (if false or not set, pose won't be controlled)
    bool global_pose = false;
    if (this->sdf->HasElement("pose")) {
      global_pose = this->sdf->Get<bool>("pose");
    }

  	// read links
  	sdf::ElementPtr linkParameter, poseParameter;

  	int idx = 1;
  	std::ostringstream tag;
  	tag << "link" << idx;

  	while (this->sdf->HasElement(tag.str())) {
  		linkParameter = this->sdf->GetElementImpl(tag.str());
  		currentLink.valid = true;  // by default the current link is valid, it's set to false if something is wrong

      // read specific link's name
  		if (linkParameter->HasAttribute("name")) {
  			std::string linkName = linkParameter->GetAttribute("name")->GetAsString(); // get links's name
  			currentLink.link = this->model->GetLink(linkName); // get joint from links's name

        // transform link on string to validate it
  			std::ostringstream validateLink;
  			validateLink << currentLink.link;

  			if (validateLink.str() == "0") {
  				gzerr << linkName << " isn't a valid link name, " << tag.str() << " will be ignored!" << std::endl;
  				currentLink.valid = false;
  			} else {
  				currentLink.name = currentLink.link->GetScopedName(); // save scoped name (will be used by PID controller)
  			}

  		} else {
  			gzerr << tag.str() << " doesn't have a name and will be ignored!" << std::endl;
  			currentLink.valid = false;
  		}

      // read specific joint's velocity (if false or not set, velocity won't be controlled)
  		// currentJoint.velocity = global_velocity;
  		// if (jointParameter->HasElement("velocity")) {
  		// 	currentJoint.velocity = jointParameter->Get<bool>("velocity");
  		// }

      // read specific link's pose (if false or not set, pose won't be controlled)
  		currentLink.pose = global_pose;
  		if (linkParameter->HasElement("pose")) {
  			currentLink.pose = linkParameter->Get<bool>("pose");
  		}

      // read specific joint's velocity topic name
  		// if (currentJoint.velocity) {
      //   if (jointParameter->HasAttribute("vel_topic")) {
  		// 	  currentJoint.veltopic = "/" + jointParameter->GetAttribute("vel_topic")->GetAsString();
  		//   } else {  // if velocity topic isn't given, use default name
  		// 	  currentJoint.veltopic = "/" + currentJoint.joint->GetScopedName() + "/joint_vel";
      //   }
      //   validate_str(currentJoint.veltopic);
  		// }

      // read specific joint's pose topic name
  		if (currentLink.pose) {
        poseParameter = linkParameter->GetElementImpl("pose");
        if (poseParameter->HasAttribute("topic")) {
  			  currentLink.postopic = "/" + poseParameter->GetAttribute("topic")->GetAsString();
  		  } else {  // if pose topic isn't given, use default name
  			  currentLink.postopic = "/" + currentLink.link->GetScopedName() + "/worldcog_pose" ;
        }
        validate_str(currentLink.postopic);
  		}

      // add curent link to linkss vector
  		this->links.push_back(currentLink);

      // increment index to test next link
  		idx++;
  		tag.str("");
  		tag << "link" << idx;
    }

  }

  /*-------------------*/

  // void DebugLinkDataParser::ShowJoints() {
  //   int n_joints = this->joints.size();
  //   std::cout << std::endl << "------------------------" << std::endl;
  //   gzmsg << "PID Control found "<< n_joints << " joints:" << std::endl;
  //   std::cout << "------------------------" << std::endl;
  //
  //   for (int i = 0; i < n_joints; i++) {
  //
  //     std::string control = " (not controlled)";
  //     if (this->joints[i].valid) {
  //       if (this->joints[i].velocity && this->joints[i].pose) {
  //         control = " (controlling velocity and pose)";
  //       } else if (this->joints[i].velocity) {
  //         control = " (controlling velocity)";
  //       } else if (this->joints[i].pose) {
  //         control = " (controlling pose)";
  //       }
  //     }
  //     gzmsg << this->joints[i].name << control << std::endl;
  //   }
  //   std::cout << "------------------------" << std::endl << std::endl;
  // }

  /*-------------------*/

  int DebugLinkDataParser::GetLinkCount() {
    return links.size();
  }

  /*-------------------*/

  link_param *DebugLinkDataParser::GetLink(int idx) {
    return &this->links[idx];
  }

}
