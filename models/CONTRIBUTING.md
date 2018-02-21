# Create new models
The recommended way to create a model is to use the `create`  script, it will create an example model that already contains some example links and joints and follow the conventions.

Models created using the `create` script use embedded ruby to make it easier to edit it (properties are automatically recalculated when dimentions are changed).

Another way to create a model is to export it from Sollidworks.

## Name conventions for models

* **Directory:** UPPERCASEPROJECT_snake_case_name
* **Files:**
  * **model.config:** configurations file _(required)_
  * **model.sdf:** model definition fle _(required)_
  * **model.sdf.erb:** template file (used on models that use embedded ruby)

In case of doubt see the existent models.

## Embedded Ruby

to do
