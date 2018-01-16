# Camera ROS plugin
> The *Camera ROS* is a Gazebo *sensor* plugin used to send Gazebo camera output via ROS messages. The plugin is a modification of the camera plugin that comes with Gazebo.

## How to use
### Compile
On the parent directory of the repository run on terminal:

```
script/compile GENERIC_camera_ros
```

### Add plugin
Use the `plugin` SDF tag to add the Camera ROS plugin to a sensor.
This tag has two required attributes:
* **name:** A unique name, scoped to its parent.
* **filename:** lib_camera_ros_plugin.so

###### Example:

```xml
<plugin name='camera_plugin' filename='lib_camera_ros_plugin.so' />
```
For more information about adding plugins, see the [Gazebo tutorials](http://gazebosim.org/tutorials?tut=plugins_model&cat=running_the_plugin#RunningthePlugin.)

### Run World
Before running the world, add the lib directory to the GAZEBO_PLUGIN_PATH so that Gazebo can find the plugin.

```
export GAZEBO_PLUGIN_PATH=$(pwd)/lib/plugins:${GAZEBO_PLUGIN_PATH}
```

### ROS Message
The plugin publishes ROS messages of type `sensor_msgs/Image` to send the camera output.
