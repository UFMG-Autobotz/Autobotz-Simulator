# Debug Link plugin
> The *Debug Link* is a Gazebo *model* plugin used to send ROS messages with information about the links of a model. The plugin is not a replacement for sensors, it is intended to be used for debug purposes only.

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
