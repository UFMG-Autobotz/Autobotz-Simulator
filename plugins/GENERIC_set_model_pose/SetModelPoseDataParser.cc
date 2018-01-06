#include "SetModelPoseDataParser.hh"

namespace gazebo {

  bool invalidChar (char c) {
  	return !((c>=47 && c <=57) || (c>=65 && c <=90) || (c>=97 && c <=122) );
  }

  /*-------------------*/

  void validate_str(std::string & str) {
  	std::replace_if(str.begin(), str.end(), invalidChar, '_');
  }

  /*-------------------*/

  SetModelPoseDataParser::SetModelPoseDataParser(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
    this->world = _world;
    this->sdf = _sdf;
  }

  /*-------------------*/

  void SetModelPoseDataParser::ReadVariables() {
    model_param currentModel;

  	// read joints
  	sdf::ElementPtr modelParameter;

  	int idx = 1;
  	std::ostringstream tag;
  	tag << "model" << idx;

  	while (this->sdf->HasElement(tag.str())) {
  		modelParameter = this->sdf->GetElementImpl(tag.str());
  		currentModel.valid = true;  // by default the current model is valid, it's set to false if something is wrong

      // read specific model's name
  		if (modelParameter->HasAttribute("name")) {
  			std::string modelName = modelParameter->GetAttribute("name")->GetAsString(); // get models's name
  			currentModel.model = this->world->GetModel(modelName); // get model from models's name

        // transform model on string to validate it
  			std::ostringstream validateModel;
  			validateModel << currentModel.model;

  			if (validateModel.str() == "0") {
  				gzerr << modelName << " isn't a valid model name, " << tag.str() << " will be ignored!" << std::endl;
  				currentModel.valid = false;
  			} else {
  				currentModel.name = currentModel.model->GetScopedName(); // save scoped name
  			}

  		} else {
  			gzerr << tag.str() << " doesn't have a name and will be ignored!" << std::endl;
  			currentModel.valid = false;
  		}

      // read specific models's topic name
      if (modelParameter->HasAttribute("topic")) {
			  currentModel.topic = "/" + modelParameter->GetAttribute("topic")->GetAsString();
		  } else {  // if topic isn't given, use default name
			  currentModel.topic = "/" + currentModel.model->GetScopedName() + "/pose";
      }
      validate_str(currentModel.topic);

      // add curent models to models vector
  		this->models.push_back(currentModel);

      // increment index to test next model
  		idx++;
  		tag.str("");
  		tag << "model" << idx;

    }

    // if no model is specified (idx not incremented)
  	if (idx == 1) {
      // save all valid models on the models vector
      int total_n_models = this->world->GetModelCount();
    	std::vector<physics::ModelPtr> allModels = this->world->GetModels();
      for (int i = 0; i < total_n_models; i++) {
        // read model
        currentModel.model = allModels[i];
        currentModel.name = allModels[i]->GetScopedName();

        // create topic names
    		currentModel.topic = "/" + currentModel.model->GetScopedName() + "/pose";
        validate_str(currentModel.topic);

        // add curent model to models vector
    		this->models.push_back(currentModel);
      }

  	}
  }

  /*-------------------*/

  // void SetModelPoseDataParser::ShowJoints() {
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

  int SetModelPoseDataParser::GetModelCount() {
    return this->models.size();
  }

  /*-------------------*/

  model_param *SetModelPoseDataParser::GetModel(int idx) {
    return &this->models[idx];
  }

}
