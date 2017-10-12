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

#include "PlayersConfig.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(PlayersConfigPlugin)



PlayersConfigPlugin::PlayersConfigPlugin() {
  // Initialize ROS
  if (!ros::isInitialized()) {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_client4", ros::init_options::NoSigintHandler);
  }

  // Create ROS node
  this->rosNode.reset(new ros::NodeHandle("gazebo_client4"));


}

/*-------------------*/

void PlayersConfigPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) {
  if (!_parent || !_sdf) {
    gzerr << "erro" << std::endl;
    return;
  }

  int idx = 1;
  sdf::ElementPtr paramElement;
  std::string param, material;

  std::ostringstream tag;
  tag << "param" << idx;

  while (_sdf->HasElement(tag.str())) {
    paramElement = _sdf->GetElementImpl(tag.str());

    if (paramElement->HasAttribute("model")) {
      param = "/" + paramElement->GetAttribute("model")->GetAsString();
      material = _sdf->Get<std::string>(tag.str());
      std::cout << material << std::endl;
      this->rosNode->setParam(param, material);
    } else {
      gzerr << "Please select a model to change the texture" << std::endl;
    }

    idx++;
    tag.str("");
    tag << "param" << idx;
  }

}
