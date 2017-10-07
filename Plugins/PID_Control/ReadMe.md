*Read-me in progress...*

# PID Joint control
> The *PID_Control* is a Gazebo model plugin used to receive ROS messages to control the velocity and/or position of a joint via PID controller. The plugin can be used with revolute and prismatic joints.

## How to use
### Compile
On the PID_Control directory run on terminal:

```
bash compile.sh
```

### Add plugin
Use the `plugin` SDF tag to add the PID control to a model.
This tag has two required attributes:
* **name:** A unique name, scoped to its parent.
* **filename:** libpid_control_plugin.so

In order to configure the plugin, other tags can be added (more information on the [settings section](#settings)).

###### Example:

```xml
<plugin name="joint control plugin" filename="libpid_control_plugin.so"/>
```
For more information about adding plugins, see the [Gazebo tutorials](http://gazebosim.org/tutorials?tut=plugins_model&cat=running_the_plugin#RunningthePlugin.)

### Run World
Before running the world, set the import directory to the GAZEBO_PLUGIN_PATH so that Gazebo can find the plugin.

```
export GAZEBO_PLUGIN_PATH=$(pwd)/Plugins/PID_Control/import:${GAZEBO_PLUGIN_PATH}
```

## Settings

### Joints
To control a joint use the `jointN tag`, where N is the number of the joint.
The plugin will look for joint tags starting from joint1 and will stop searching when a joint is not found, so always start from 1 and don't skip a number.

**Note:** The joint number is used by the parser to differentiate joints and is not related to the actual joints on the model (the name attribute is used to select a joint).

The jointN tag has 3 attributes:
* **name** *(required)*: The name of the joint as defined on the model
* **vel_topic**: Name of the ROS topic that send the desired velocity to the plugin
  * *default:* `/modelName/joint_vel_jointName`
* **pos_topic**: Name of the ROS topic that send the desired position to the plugin
  * *default:* `/modelName/joint_pos_jointName`

###### Example:
```xml
<plugin name="joint control plugin" filename="libpid_control_plugin.so">
  <joint1 name=""></joint1>
  <joint2 name=""></joint2>
</plugin>
```

### Velocity and Position
The `velocity` and `position` tags receive a boolean that says whether the velocity and position will be controlled, `false` by default.

If added to a joint it will set the property of this joint. If added to the plugin, it will set the property of any joint where this property isn't set.

###### Example:
```xml
<plugin name="joint control plugin" filename="libpid_control_plugin.so">
  <velocity>true</velocity>
  <joint1 name="">
    <position>true</position>
  </joint1>
  <joint2 name="">
    <velocity>false</velocity>
  </joint2>
</plugin>
```
