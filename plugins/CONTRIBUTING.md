# Create new plugins
The best way to create a plugin is to use the `create`  script, it will create an example plugin that already follow the conventions.
Sometimes, though, it is easier to copy an existent plugin (one that comes with Gazebo, for example) and modify it. So it's important to know the name conventions.

## Name conventions for plugins

* **Directory:** UPPERCASEPROJECT_snake_case_name
* **Files:** PascalCaseNamePlugin.cc, PascalCaseNamePlugin.hh (other files: PascalCaseNamePascalCaseFunction.cc, PascalCaseNamePascalCaseFunction.hh)
* **Class name:** PascalCaseNamePlugin (other classes: PascalCaseNameFunction)
* **Identifier on #ifndef directive:** GAZEBO_UPPERCASE_WITH_UNDERSCORES_NAME_PLUGIN_HH
* **CMake library name:**  _snake_case_name_plugin (to generate lib_snake_case_name_plugin.so) (other libraries _snake_cane_name_snake_case_function)

In case of doubt see the existent plugins.

> **Note:**

> Update the `create` plugin and the existent plugins any time the name conventions change.


## Message conventions for plugins

The conventions used to print information or erros on the console are described bellow. In case of doubt see the existent plugins.

### Function used for messages

* If the message is related to Gazebo elements, use `gzmsg` and `gzerr` ([reference](https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/group__gazebo__common.html)).
* If the message if related to ROS, use `ROS_INFO_STREAM` and `ROS_ERROR_STREAM` ([reference](http://wiki.ros.org/roscpp/Overview/Logging)).

### Load error
On the beginning of the `Load` function, check if its parameters were correctly received:

```
if (!_{{type}} || !_sdf) {
  gzerr << "No {{type}} or SDF element specified. {{name}} Plugin won't load." << std::endl;
}
```

### ROS Messages info
When a plugin publishes or subscribes to a ROS topic, print the topics with the following pattern:

```
std::cout << std::endl << "------------------------" << std::endl;
ROS_INFO_STREAM("On {{Element using plugin in bold}}, {{Name}} Plugin publishing/subscribing to ROS topic(s):");
std::cout << "------------------------" << std::endl;
ROS_INFO_STREAM({{topic_name}});
std::cout << "------------------------" << std::endl << std::endl;
```

### Other messages

Inform which plugin is sending the message beginning it with the tag [{{Name}} Plugin]

## Binary files adress

The compiled binaries are saved on the `lib/plugins` directory. To do so, the `CMakeLists.txt` must contain the following lines:

```CMake
# Choose where the plugin will be built
set(dir ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/plugins)
set(LIBRARY_OUTPUT_PATH ${dir} CACHE PATH "Build directory" FORCE)
```
