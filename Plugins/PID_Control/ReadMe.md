# PID Joint control
The *PID_Control* is a Gazebo model plugin used to control the velocity and/or position of a joint via PID controller. The plugin can be used with revolute and prismatic joints.

## How to use
### Compile
On the PID_Control directory run on terminal:

```
bash compile.sh
```

### Add plugin
Use the plugin SDF tag to add the PID control to a model.
This tag has two required attributes:
* **name:** A unique name, scoped to its parent.
* **filename:** libpid_control_plugin.so

In order to configure the plugin, other tags can be added. More on configuration on the [settings section](#settings).

###### Example:

```xml
<plugin name="vt_sim plugin" filename="libpid_control_plugin.so">
  <velocity>true</velocity>
  <pid>5 0 0</pid>
</plugin>
```
For more information about adding plugins, see the [Gazebo tutorials](http://gazebosim.org/tutorials?tut=plugins_model&cat=running_the_plugin#RunningthePlugin.)

### Run World
Before running the world, set the import directory to the GAZEBO_PLUGIN_PATH so that Gazebo can find the plugin.

```
export GAZEBO_PLUGIN_PATH=$(pwd)/Plugins/PID_Control/import:${GAZEBO_PLUGIN_PATH}
```

## Settings
