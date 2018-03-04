# Create new models
The recommended way to create a model is to use the `create`  script, it will create an example model that already contains some example links and joints and follow the conventions.

Models created using the `create` script use embedded ruby to make it easier to edit it (properties are automatically recalculated when dimentions are changed).

Another way to create a model is to export it from Solidworks.

## Name conventions for models

* **Directory:** UPPERCASEPROJECT_snake_case_name
* **Files:**
  * **model.config:** configurations file _(required)_
  * **model.sdf:** model definition file _(required)_
  * **model.sdf.erb:** template file (used on models that use embedded ruby)

In case of doubt see the existent models.

## Embedded Ruby

Embedded Ruby is a templating system that embeds Ruby code into a text document.
It's used on Gazebo models to embed code that calculates physic properties of the model into the SDF, so they are automatically recalculated when a geometric property is changed.

### Ruby Classes

Ruby classes were created to facilitate the use of embedded ruby to automate changes.
They are saved on the `lib/models/` directory. When you create a model from the template there will be instructions on how to use them.

You can also create your own Ruby classes for a spectific project, see the `VSS_player` model for an example.

### Generate SDF from templates

To generate a `model.sdf` file from a `model.sdf.erb` template, use the following command:

``` shell
erb model.sdf.erb > model.sdf
```

If you use the `run` script to run your simulations, this command is already executed for you.
