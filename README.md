# Autobotz simulator

> The Autobotz Simulator repository centralizes Autobotz Gazebo simulations and resources. It also facilitates the use of Gazebo for Autobotz members through utilities such as scripts, plugins, etc.

## Projects
Some files and folders are named with tags according to the project it belongs to, which indicates their scope of usage:

* **GENERIC:** can be used on multiple applications
* **TEST:** used to test some properties and learn to use Gazebo
* **VSS:** Very Small Size Soccer competition
* **VT:** Land Vehicle

## How to use

### Compiling plugins

Before using the plugins it is necessary to compile them. This has to be done after cloning the repository and each time a plugin is modified (it's recommended to recompile the plugin after using `git pull`). To compile the plugins, run on terminal:

```
script/compile all
```

For more information about the `compile` script usage, see the `README.md` of the `script` directory.

### Run simulations

To run a simulation, run on the terminal `script/run` followed by the **project** the simulation belongs to and the **name** of the simulation. For example, to run the `penalty` simulation which is part of the `VSS` project, run:

```
script/run VSS penalty
```

To learn more about the `run` script, see the `README.md` of the `script` directory. To see the list of available simulations, see the `README.md` of the `worlds` directory.

### Other uses
To see other things you can do on the Autobotz Simulator, see the `README.md` of the `script` directory.

## Learn more

To learn more about the use of the Autobotz Simulator you can:

* Read the `README.md` and the `CONTRIBUTING.md` of the subdirectories;
* Read the [repository's wiki](https://github.com/UFMG-Autobotz/Autobotz-Simulator/wiki);
* Follow the [Gazebo and Autobotz Simulator course](https://gitlab.com/Autobotz-UFMG/COURSE_Gazebo).
