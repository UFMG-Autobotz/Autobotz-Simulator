# Scripts to rule them all

These scripts are inspired by the [normalized script pattern that GitHub uses in its projects](https://githubengineering.com/scripts-to-rule-them-all/).
Here's a description of what each script used in this project does.

# script/run
>  run Gazebo simulations

Use it to run the Autobotz-Simulator worlds

#### Usage
`script/run [project] [world] [options]`

#### Projects
* **VSS:** Very Small Size Soccer, used with the AutoGoal project
* **VT:** Land vehicle
* **GENERIC:** Not related to any project
* **TEST:** Used for testes, use it on the test/* branchs

#### Options
* **-q:** Run Gazebo on quiet mode (without the graphic interface)
* **-u:** Start world paused

###### Example:
```
# Open world VSS_1on1.world on quiet mode
script/run vss 1on1 -q

# Open Gazebo without any world paused
script/run generic none -u
```

## script/compile
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
