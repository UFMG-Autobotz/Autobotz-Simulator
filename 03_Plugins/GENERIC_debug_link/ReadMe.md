# Debug Link plugin
> The *Debug Link* is a Gazebo *model* plugin used to send ROS messages with information about the links of a model. The plugin is not a replacement for sensors, it is intended to be used for debug purposes.

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
* **filename:** lib_debug_link_plugin.so

In order to configure the plugin, other tags can be added inside of the `plugin` tag (more information on the [settings section](#settings)).

###### Example:

```xml
<plugin name="debug" filename="lib_debug_link_plugin.so">
  <!-- tags to set plugin up -->
</plugin>
```

For more information about adding plugins, see the [Gazebo tutorials](http://gazebosim.org/tutorials?tut=plugins_model&cat=running_the_plugin#RunningthePlugin.)

### Run World
Before running the world, set the import directory to the GAZEBO_PLUGIN_PATH so that Gazebo can find the plugin.

```
export GAZEBO_PLUGIN_PATH=$(pwd)/Plugins/GENERIC_debug_link/import:${GAZEBO_PLUGIN_PATH}
```

## Settings

### Links
To choose a link to get information from, use the `linkN` tag, where N is the number of the link.
The plugin will look for link tags starting from `link1` and will stop searching when a link is not found, so always start from 1 and don't skip a number.

The `linkN` tag has 1 required attribute, **name**, which is the name of the link as defined on the model.

**Note:** The link number is used only by the parser, the name attribute is what is used to identify a link on the model.

###### Example:
```xml
<plugin name="debug" filename="lib_debug_link_plugin.so">
  <link1 name="player::chassi_left_wheel">
    <!-- tags to choose variables for link 1-->
  </link1>

  <link2 name="player::chassi_right_wheel">
    <!-- tags to choose variables for link 2-->
  </link2>
</plugin>
```

### Variables
To choose what information you want to get form a link, use the `variableN` tag, where N is the number of the variable. Again, the plugin will look for variable tags starting from `variable1` and will stop searching when a variable is not found, so always start from 1 and don't skip a number.

The `variableN` tag has 2 attributes:
* **scope**: The scope of the variable:
  * *Relative*: get the value of the variable relative to the model's own frame
  * *World* (default): get the value of the variable relative to the world's frame
  * *WordlCoG*: get the value of the variable on the center of gravity relative to the world's frame
* **topic**: Name of the ROS topic that will send the variable value
  * *default:* `/modelInstanceName__modelName__linkName/variableScope_variableName`

The value inside of the `variableN` tag is the name of the desired variable.

Here is a list of variables that can be read with the debug link plugin with its available scopes and ROS messages types:

Variable | Name | Relative | World | WorldCoG | Message Type
---| --- | --- | --- | --- | ---
Torque | Torque | x | x | |  geometry_msgs::Vector3
Angular Acceleration |  AngularAccel | x | x | | geometry_msgs::Vector3
Angular Velocity | AngularVel | x | x | |  geometry_msgs::Vector3
Force | Force | x | x | |  geometry_msgs::Vector3
Linear Acceleration | LinearAccel | x | x | | geometry_msgs::Vector3
Linear Velocity | LinearVel | x | x | x |  geometry_msgs::Vector3
Pose | Pose | x | x | x |  geometry_msgs::Pose
Angular Momentum | AngularMomentum | | x | |  geometry_msgs::Vector3
Total Energy | Energy | | x | |  std_msgs::Float64
Kinetic Energy | EnergyKinetic | | x | |  std_msgs::Float64
Potential Energy | EnergyPotential  | | x | |  std_msgs::Float64

###### Example:
```xml
<plugin name="player 1 debug" filename="lib_debug_link_plugin.so">
  <link1 name="player::chassi">
    <!-- get pose of the chassi scoped WorldCoG on topic /player1/pose -->
    <variable1 scope="WorldCoG" topic="player1/pose">Pose</variable1>

    <!-- get linear velocity of the chassi scoped World on default topic -->
    <variable2>LinearVel</variable2>
  </link1>
</plugin>
```
