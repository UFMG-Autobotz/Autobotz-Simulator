# Create new world
To create a new world use the `create`  script, it will create a world that already follows the name conventions and has a sun and a floor.

**Never** create worlds using the Gazebo GUI, it copies the entire content of the models into the world file, making it unnecessarily big and complex. More importantly, if you edit a model added via GUI, the model will not be changed on the world, so, always add models via text using the `include` tag.

In case of doubt see the existent worlds.
