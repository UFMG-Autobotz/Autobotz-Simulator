/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "ChangeTexture.hh"

using namespace gazebo;

GZ_REGISTER_VISUAL_PLUGIN(ChangeTexturePlugin)



ChangeTexturePlugin::ChangeTexturePlugin() {
  // Initialize ROS
  if (!ros::isInitialized()) {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_client3", ros::init_options::NoSigintHandler);
  }

  // Create ROS node
  this->rosNode.reset(new ros::NodeHandle("gazebo_client3"));

}

/*-------------------*/

void ChangeTexturePlugin::Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
{
  if (!_visual || !_sdf) {
    gzerr << "No visual or SDF element specified. Change Texture Plugin won't load." << std::endl;
    return;
  }

  // get model name
  std::string visual_name = _visual->GetName();
  std::string param = "/" + visual_name.substr(0, visual_name.find(':'));

  // if ros parameter found with the name of the model, use it to change material of the visual
  std::string newMaterial;
  if (this->rosNode->getParam(param, newMaterial)) {
    _visual->SetMaterial(newMaterial);
    gzmsg << "Changed material of " << visual_name << " to " << newMaterial << std::endl;
  } else {
    gzerr << "Please set the material" << std::endl;
  }

}
