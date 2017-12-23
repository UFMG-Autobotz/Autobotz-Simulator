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

```xml
<link1>
  <pose scope="worldCG"/>
</link1>
```

```xml
<plugin name="player 1 debug" filename="lib_debug_link_plugin.so">
  <link1 name="player::chassi">
    <pose scope="worldCG" topic="player1/pose">true</pose>
  </link1>
</plugin>
```

to

```xml
<plugin name="player 1 debug" filename="lib_debug_link_plugin.so">
  <link1 name="player::chassi">
    <variable1 scope="worldCG" topic="player1/pose">pose</variable1>
  </link1>
</plugin>
```
