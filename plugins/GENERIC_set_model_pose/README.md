# :heavy_exclamation_mark: *Incomplete plugin* :heavy_exclamation_mark:

This plugin is intended to be used to set the position and orientations of a model during the simulation.

Currently, it's set to be used only when the world is paused, because it causes the model to move unexpectly (it flies imediatly) otherwise.

But even with this workaround, the plugin is still buggy, if `reset pose` is used when the world is paused, the models move to unexpected positions.

Maybe ROS Params or ROS Services should be used instead of ROS Messages.
