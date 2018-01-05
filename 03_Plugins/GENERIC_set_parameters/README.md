# Set Parameters plugin
> The *Set Parameters* is a Gazebo *world* plugin used to set ROS parameters when a world is loaded. The parameters can then be used by other plugins that are not able to receive attributes from the .world file, like sensor and visual plugins

## How to use
### Compile
On the `GENERIC_set_parameters` directory run on terminal:

```
bash compile.sh
```

### Add plugin
Use the `plugin` SDF tag to add the set parameters plugin to a world.
This tag has two required attributes:
* **name:** A unique name, scoped to its parent.
* **filename:** lib_pid_control_plugin.so

In order to configure the plugin, other tags can be added inside of the `plugin` tag (more information on the [settings section](#settings)).

For more information about adding plugins, see the [Gazebo tutorials](http://gazebosim.org/tutorials?tut=plugins_model&cat=running_the_plugin#RunningthePlugin.)

### Run World
Before running the world, set the import directory to the GAZEBO_PLUGIN_PATH so that Gazebo can find the plugin.

```
export GAZEBO_PLUGIN_PATH=$(pwd)/03_Plugins/GENERIC_set_parameters/import:${GAZEBO_PLUGIN_PATH}
```

## Settings
To set a parameter use the `paramN tag`, where N is the number of the parameter.
The plugin will look for params tags starting from `param1` and will stop searching when a param is not found, so always start from 1 and don't skip a number.

**Note:** The param number is used by the parser to differentiate parameters and is not related to the actual parameters created.

The jointN tag has 1 required attribute: `parameter` wich is the name of the ROS parameter setted. The value goes inside of the tag.

###### Example:
```xml

<plugin name="config" filename="lib_set_parameters.so">
  <!-- set ROS parameter: player1 with value "VSS_player/yellow1" -->
  <param1 parameter="player1">VSS_player/yellow1</param1>

  <!-- set ROS parameter: player2 with value "VSS_player/blue2" -->
  <param2 parameter="player2">VSS_player/blue2</param2>
</plugin>
```
