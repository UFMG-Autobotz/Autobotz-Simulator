#include "Data.hh"

namespace gazebo {

  bool invalidChar (char c) {
  	return !((c>=47 && c <=57) || (c>=65 && c <=90) || (c>=97 && c <=122) );
  }

  /*-------------------*/

  void validate_str(std::string & str) {
  	std::replace_if(str.begin(), str.end(), invalidChar, '_');
  }

  /*-------------------*/

  Data::Data(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    this->model = _model;
    this->sdf = _sdf;
  }

  /*-------------------*/

  void Data::ReadVariables() {
    link_param currentLink;

    gzmsg << std::endl << "TESTE" << std::endl;

    // read global velocity (if false or not set, velocity won't be controlled)
    // bool global_velocity = false;
    // if (this->sdf->HasElement("velocity")) {
    //   global_velocity = this->sdf->Get<bool>("velocity");
    // }

    // read global position (if false or not set, position won't be controlled)
    bool global_position = false;
    if (this->sdf->HasElement("position")) {
      global_position = this->sdf->Get<bool>("position");
    }

  	// read links
  	sdf::ElementPtr linkParameter, posParameter;

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

      // read specific link's position (if false or not set, position won't be controlled)
  		currentLink.position = global_position;
  		if (linkParameter->HasElement("position")) {
  			currentLink.position = linkParameter->Get<bool>("position");
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

      // read specific joint's position topic name
  		if (currentLink.position) {
        posParameter = linkParameter->GetElementImpl("position");
        if (posParameter->HasAttribute("topic")) {
  			  currentLink.postopic = "/" + posParameter->GetAttribute("topic")->GetAsString();
  		  } else {  // if position topic isn't given, use default name
  			  currentLink.postopic = "/" + currentLink.link->GetScopedName() + "/worldcog_pos";
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

  // void Data::ShowJoints() {
  //   int n_joints = this->joints.size();
  //   std::cout << std::endl << "------------------------" << std::endl;
  //   gzmsg << "PID Control found "<< n_joints << " joints:" << std::endl;
  //   std::cout << "------------------------" << std::endl;
  //
  //   for (int i = 0; i < n_joints; i++) {
  //
  //     std::string control = " (not controlled)";
  //     if (this->joints[i].valid) {
  //       if (this->joints[i].velocity && this->joints[i].position) {
  //         control = " (controlling velocity and position)";
  //       } else if (this->joints[i].velocity) {
  //         control = " (controlling velocity)";
  //       } else if (this->joints[i].position) {
  //         control = " (controlling position)";
  //       }
  //     }
  //     gzmsg << this->joints[i].name << control << std::endl;
  //   }
  //   std::cout << "------------------------" << std::endl << std::endl;
  // }

  /*-------------------*/

  int Data::GetLinkCount() {
    return 0;
  }

  /*-------------------*/

  link_param *Data::GetLink(int idx) {
    return &this->links[idx];
  }

}
