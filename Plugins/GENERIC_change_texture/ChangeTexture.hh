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
#ifndef GAZEBO_CHANGETEXTUREPLUGIN_HH_
#define GAZEBO_CHANGETEXTUREPLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/Visual.hh>

#include "ros/ros.h"
#include "ros/callback_queue.h"
#include <ros/console.h>
#include "ros/subscribe_options.h"
#include "std_msgs/String.h"
#include <thread>

namespace gazebo
{
  class GAZEBO_VISIBLE ChangeTexturePlugin : public VisualPlugin {
    // methods
    public: ChangeTexturePlugin();
    public: virtual void Load(rendering::VisualPtr _visual,
        sdf::ElementPtr _sdf);

    // attributes
    private: rendering::VisualPtr visual;
    private: std::unique_ptr<ros::NodeHandle> rosNode;	/// \brief A node use for ROS transport

  };
}
#endif
