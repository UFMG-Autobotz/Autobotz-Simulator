# Create new plugins
The best way to create a plugin is to use the `create`  script, it will create an example plugin that already follow the name conventions.
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
