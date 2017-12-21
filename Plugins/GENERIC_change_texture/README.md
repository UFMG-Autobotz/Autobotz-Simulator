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

## Set Parameters
To be able to have multiple instances of the same model with different materials this plugin receive a ROS parameter. The parameter name is the name of the model on the world and the its value is the name of the chosen material.

A tipical way to set the parameter is using the `Set Parameter` plugin.
