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


## Error message conventions for plugins

```
if (!_{{type}} || !_sdf) {
  gzerr << "No {{type}} or SDF element specified. {{name}} Plugin won't load." << std::endl;
}
```

```
std::cout << std::endl << "------------------------" << std::endl;
ROS_INFO_STREAM("Camera Plugin publishing to ROS topic:");
std::cout << "------------------------" << std::endl;
ROS_INFO_STREAM(topic_name);
std::cout << "------------------------" << std::endl << std::endl;
```

## Binary files adress

The compiled binaries are saved on the `lib/plugins` directory. To do so, the `CMakeLists.txt` must contain the following lines:

```CMake
# Choose where the plugin will be built
set(dir ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/plugins)
set(LIBRARY_OUTPUT_PATH ${dir} CACHE PATH "Build directory" FORCE)
```
