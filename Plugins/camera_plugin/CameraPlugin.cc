/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include "gazebo/sensors/DepthCameraSensor.hh"
#include "CameraPlugin.hh"

bool invalidChar (char c) {
  return !((c>=47 && c <=57) || (c>=65 && c <=90) || (c>=97 && c <=122) );
}

/*-------------------*/

void validate_str(std::string & str) {
  std::replace_if(str.begin(), str.end(), invalidChar, '/');
}


using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(CameraPlugin)

/////////////////////////////////////////////////
CameraPlugin::CameraPlugin()
: SensorPlugin(), width(0), height(0), depth(0)
{
  // Initialize ROS
  if (!ros::isInitialized()) {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_client2", ros::init_options::NoSigintHandler);
  }

  // Create ROS node
  this->rosNode.reset(new ros::NodeHandle("gazebo_client2"));
}

/////////////////////////////////////////////////
CameraPlugin::~CameraPlugin()
{
  this->parentSensor.reset();
  this->camera.reset();
}

/////////////////////////////////////////////////
void CameraPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  if (!_sensor)
    gzerr << "Invalid sensor pointer.\n";

  this->parentSensor =
    std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);

  if (!this->parentSensor)
  {
    gzerr << "CameraPlugin requires a CameraSensor.\n";
    if (std::dynamic_pointer_cast<sensors::DepthCameraSensor>(_sensor))
      gzmsg << "It is a depth camera sensor\n";
  }

  this->camera = this->parentSensor->Camera();

  if (!this->parentSensor)
  {
    gzerr << "CameraPlugin not attached to a camera sensor\n";
    return;
  }

  this->width = this->camera->ImageWidth();
  this->height = this->camera->ImageHeight();
  this->depth = this->camera->ImageDepth();
  this->format = this->camera->ImageFormat();

  this->newFrameConnection = this->camera->ConnectNewImageFrame(
      std::bind(&CameraPlugin::OnNewFrame, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));

  this->parentSensor->SetActive(true);

  std::string topic_name = _sensor->GetTopic();
  // validate_str(topic_name);
  this->rosPub = this->rosNode->advertise<sensor_msgs::Image>(topic_name.erase(0, 1), 1000);
}

/////////////////////////////////////////////////
void CameraPlugin::OnNewFrame(const unsigned char * _image,
                              unsigned int _width,
                              unsigned int _height,
                              unsigned int _depth,
                              const std::string &_format)
{
    sensor_msgs::Image frame;
    frame.height = _height;
    frame.width = _width;
    frame.encoding = "rgb8";
    frame.step = 3 * _width;

    int size = frame.height * frame.step;
    for (int i = 0; i < size; i += 1) {
      frame.data.push_back(_image[i]);
    }

    this->rosPub.publish(frame);
}
