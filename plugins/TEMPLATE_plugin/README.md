# Template plugin
> The *Template* is a Gazebo *model* plugin used as an example. It sends and receives Float32 ROS messages.

## How to use
### Compile
On the parent directory of the repository run on terminal:

```
script/compile TEMPLATE_plugin
```

### Add plugin
Use the `plugin` SDF tag to add the set parameters plugin to a world.
This tag has two required attributes:
* **name:** A unique name, scoped to its parent.
* **filename:** lib_template_plugin.so

<!-- In order to configure the plugin, other tags can be added inside of the `plugin` tag (more information on the [settings section](#settings)). -->

###### Example:
```xml

<plugin name="example" filename="lib_template_plugin.so">
  <!-- plugin settings -->
</plugin>
```

For more information about adding plugins, see the [Gazebo tutorials](http://gazebosim.org/tutorials?tut=plugins_model&cat=running_the_plugin#RunningthePlugin.)

### Run World
Before running the world, add the lib directory to the GAZEBO_PLUGIN_PATH so that Gazebo can find the plugin.

```
export GAZEBO_PLUGIN_PATH=$(pwd)/lib/plugins:${GAZEBO_PLUGIN_PATH}
```

<!-- ## Settings -->

<!-- ### ROS Message
The plugin publishes ROS messages of type `{{type}}` to send {{information}}.
The plugin subscribes ROS messages of type `{{type}}` to receive {{information}}. -->
