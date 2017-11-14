# PID Joint control
> The *PID_Control* is a Gazebo model plugin used to receive ROS messages to control the velocity and/or position of a joint via Gazebo PID controller. The plugin can be used with revolute and prismatic joints.

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
* **filename:** lib_pid_control_plugin.so

In order to configure the plugin, other tags can be added inside of the `plugin` tag (more information on the [settings section](#settings)).

###### Example:

```xml
<plugin name="joint control plugin" filename="lib_pid_control_plugin.so"/>
```
For more information about adding plugins, see the [Gazebo tutorials](http://gazebosim.org/tutorials?tut=plugins_model&cat=running_the_plugin#RunningthePlugin.)

### Run World
Before running the world, set the import directory to the GAZEBO_PLUGIN_PATH so that Gazebo can find the plugin.

```
export GAZEBO_PLUGIN_PATH=$(pwd)/Plugins/GENERIC_pid_control/import:${GAZEBO_PLUGIN_PATH}
```

## Settings

### Joints
To control a joint use the `jointN tag`, where N is the number of the joint.
The plugin will look for joint tags starting from `joint1` and will stop searching when a joint is not found, so always start from 1 and don't skip a number.

**Note:** The joint number is used by the parser to differentiate joints and is not related to the actual joints on the model (the name attribute is used to select a joint).

The jointN tag has 3 attributes:
* **name** *(required)*: The name of the joint as defined on the model
  * follows the format: modelName::jointName
* **vel_topic**: Name of the ROS topic that send the desired velocity to the plugin
  * *default:* `/modelInstanceName__modelName__jointName/joint_vel`
* **pos_topic**: Name of the ROS topic that send the desired position to the plugin
  * *default:* `/modelInstanceName__modelName__jointName/joint_pos`

###### Example:
```xml
<plugin name="joint control plugin" filename="libpid_control_plugin.so">
  <!-- set ROS topic: /vel_left_wheel to control the
  velocity of the joint chassi_left from model player -->
  <joint1 name='player::chassi_left_wheel' vel_topic='vel_left_wheel'/>

  <!-- set ROS topic: /vel_right_wheel to control the
  velocity of the joint chassi_right from model player -->
  <joint2 name='player::chassi_right_wheel' vel_topic='vel_right_wheel'/>
</plugin>
```

### Velocity and Position
The `velocity` and `position` tags receive a boolean that says whether the velocity and position will be controlled, they are `false` by default.

If added to a joint it will set the property of this joint. If added to the plugin, it will set the property of any joint where this property isn't set.

###### Example:
```xml
<plugin name="joint control plugin" filename="libpid_control_plugin.so">
  <!-- for joints where velocity is not specified, it will be controlled, position won't -->
  <velocity>true</velocity>

  <joint1 name="model::chassi_left_wheel">
    <!-- position and velocity will be controlled for the joint chassi_left_wheel -->
    <position>true</position>
  </joint1>

  <joint2 name="player::chassi_right_wheel">
    <!-- nether position nor velocity will be controlled for the joint chassi_right_wheel -->
    <velocity>false</velocity>
  </joint2>
</plugin>
```

### PID Gains
The `pid`, `vel_pid` and `pos_pid` tags set the pid gains for the variables controlled. These tags can be added to the plugin or to a specific joint, the gains are chosen using the following priority order:

* **fist**
* specific pid (vel_pid or pos_pid) for specific joint
* generic pid (pid) for specific joint
* specific pid
* generic pid
* default value: kp = 1,  ki = 0 kd = 0
* **last**

###### Example:
```xml
###### Example:
```xml
<plugin name="joint control plugin" filename="libpid_control_plugin.so">
  <velocity>true</velocity>
  <position>true </position>

  <!-- when pid is not defined, these gains will be used for velocity or position -->
  <pid>0.5 0 0</pid>

  <joint1 name="player::chassi_left_wheel">
    <!-- joint chassi_left_wheel will use (0.1, 0, 0) for velocity
    and (0.5, 0, 0) for position -->
    <vel_pid>0.1 0 0</vel_pid>
  </joint1>

  <joint2 name="player::chassi_right_wheel">
    <!-- joint chassi_right_wheel will use (0.5, 0, 0) for velocity
    and (2, 0, 0) for position -->
    <pos_pid>2 0 0</pos_pid>
  </joint2>
</plugin>
```
