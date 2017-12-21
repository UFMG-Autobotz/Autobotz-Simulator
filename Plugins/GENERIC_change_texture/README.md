# Change Texture plugin
> The *Change Texture* is a Gazebo *visual* plugin used to change the material of a visual

## How to use
### Compile
On the `GENERIC_set_parameters` directory run on terminal:

```
bash compile.sh
```

### Add plugin
Use the `plugin` SDF tag inside of a `visual` tag to add the change texture plugin to a world.
This tag has two required attributes:
* **name:** A unique name, scoped to its parent.
* **filename:** lib_change_texture_plugin.so

###### Example:
```xml
<plugin name="texture" filename="lib_change_texture.so"> </plugin>
```

For more information about adding plugins, see the [Gazebo tutorials](http://gazebosim.org/tutorials?tut=plugins_model&cat=running_the_plugin#RunningthePlugin.)

### Run World
Before running the world, set the import directory to the GAZEBO_PLUGIN_PATH so that Gazebo can find the plugin.

```
export GAZEBO_PLUGIN_PATH=$(pwd)/Plugins/GENERIC_change_texture/import:${GAZEBO_PLUGIN_PATH}
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
