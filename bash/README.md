# Bash

> Scripts used to run the Gazebo simulations.


The scripts in this folder include the Gazebo initialization as well as the commands that have to be run before initializing Gazebo.

## How to use

On terminal, inside this folder, type `bash` and the name of the desired script.

###### Example:
```
bash GENERIC_gazebo.sh
```

## bash/compile
> compile Gazebo plugins

Use it after `git pull` or after changing a plugin to update the plugins binaries.


#### Example
```
# compile only one plugin
bash/compile GENERIC_camera_ros

# compile multiple plugins
bash/compile GENERIC_camera_ros GENERIC_change_material GENERIC_debug_link

# compile all plugins
bash/compile all # or simply bash/compile
```
