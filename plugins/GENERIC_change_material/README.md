# Change Material plugin
> The *Change Material* is a Gazebo *visual* plugin used to change the material of a visual

## How to use
### Compile

On the parent directory of the repository run on terminal:

```
script/compile GENERIC_change_material
```

### Add plugin
Use the `plugin` SDF tag inside of a `visual` tag to add the change material plugin to a world.
This tag has two required attributes:
* **name:** A unique name, scoped to its parent.
* **filename:** lib_change_material_plugin.so

###### Example:
```xml
<plugin name="texture" filename="lib_change_material.so"> </plugin>
```

For more information about adding plugins, see the [Gazebo tutorials](http://gazebosim.org/tutorials?tut=plugins_model&cat=running_the_plugin#RunningthePlugin.)

### Run World
Before running the world, add the lib directory to the GAZEBO_PLUGIN_PATH so that Gazebo can find the plugin.

```
export GAZEBO_PLUGIN_PATH=$(pwd)/lib/plugins:${GAZEBO_PLUGIN_PATH}
```

If you use the `run` script to run your simulations, this command is already executed for you.

## Set Parameters
To be able to have multiple instances of the same model with different materials this plugin receive a ROS parameter. The parameter name is the name of the model on the world and the its value is the name of the chosen material.

A typical way to set the parameter is using the `Set Parameter` plugin.
