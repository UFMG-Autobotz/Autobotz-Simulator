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

## script/create
> create new elements (Worls, Models or Plugins)

Use it to create new elements instead of creating them from scratch to have a copy of the templates with relevant parts already renamed according to the repository conventions.

#### Usage
`script/create [type of element] [options]`

#### Element
`world`, `model` or `plugin`.

#### Options
* **-p:** Project (VSS, VT, GENERIC or TEST), default is GENERIC
* **-n:** Name, *obligatory*

###### Example:
```
# Create new world called VSS_game.world
script/create world -p vss -n game

# Create new plugin caolled GENERIC_stop_simulation.world
script/create plugin -n stop_simulation
```

> **Note:**

> The name option doesn't accepts spaces, it recognizes snake_case, camelCase and PascalCase.
