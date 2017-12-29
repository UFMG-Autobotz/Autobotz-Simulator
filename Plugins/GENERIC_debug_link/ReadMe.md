# Debug Link plugin
> The *Debug Link* is a Gazebo *model* plugin used to send ROS messages with information about the links of a model. The plugin is not a replacement for sensors, it is intended to be used for debug purposes only.

## How to use
### Compile
On the `GENERIC_debug_link` directory run on terminal:

```
bash compile.sh
```

### Add plugin
Use the `plugin` SDF tag to add the debug link plugin to a model.
This tag has two required attributes:
* **name:** A unique name, scoped to its parent.
* **filename:** lib_pid_control_plugin.so

In order to configure the plugin, other tags can be added inside of the `plugin` tag (more information on the [settings section](#settings)).


For more information about adding plugins, see the [Gazebo tutorials](http://gazebosim.org/tutorials?tut=plugins_model&cat=running_the_plugin#RunningthePlugin.)

### Run World
Before running the world, set the import directory to the GAZEBO_PLUGIN_PATH so that Gazebo can find the plugin.

```
export GAZEBO_PLUGIN_PATH=$(pwd)/Plugins/GENERIC_debug_link/import:${GAZEBO_PLUGIN_PATH}
```

## Settings


# Link

Variable | Relative | World | World CoG | Type | Message Type
---| --- | --- | --- | --- |---
Torque | x | x | | math::Vector3 | geometry_msgs::Vector3
Angular Acceleration | x | x | | math::Vector3 | geometry_msgs::Vector3
Angular Velocity | x | x | | math::Vector3 | geometry_msgs::Vector3
Force | x | x | | math::Vector3 | geometry_msgs::Vector3
Linear Acceleration | x | x | | math::Vector3 | geometry_msgs::Vector3
Linear Velocity | x | x | x | math::Vector3 | geometry_msgs::Vector3
Pose | x | x | x | math::Pose | geometry_msgs::Pose
Angular Momentum | | x | | math::Vector3 | geometry_msgs::Vector3
Kinetic Energy | | x | | double | std_msgs::Float64
Potential Energy | | x | | double | std_msgs::Float64



to

```xml
<plugin name="player 1 debug" filename="lib_debug_link_plugin.so">
  <link1 name="player::chassi">
    <variable1 scope="WorldCoG" topic="player1/pose">Pose</variable1>
  </link1>
</plugin>
```
