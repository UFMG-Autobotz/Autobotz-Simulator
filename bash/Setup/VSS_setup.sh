cd ..

# salvar .sdf a partir do .rsdf parametrizavel
erb models/VSS_player/model.rsdf > models/VSS_player/model.sdf
erb models/VSS_ball/model.rsdf > models/VSS_ball/model.sdf

# necessario antes de modificar o GAZEBO_RESOURCE_PATH
source /usr/share/gazebo/setup.sh

# adicionar caminho para a materiais
export GAZEBO_RESOURCE_PATH=$(pwd)/materials:$GAZEBO_RESOURCE_PATH

# adicionar caminho para o stl do campo
export GAZEBO_RESOURCE_PATH=$(pwd)/meshs:$GAZEBO_RESOURCE_PATH

# adicionar caminho para os modelos
export GAZEBO_MODEL_PATH=$(pwd)/models:$GAZEBO_MODEL_PATH

# adicionar caminho para os plugins
export GAZEBO_PLUGIN_PATH=$(pwd)/plugins/GENERIC_pid_control/lib:${GAZEBO_PLUGIN_PATH}
export GAZEBO_PLUGIN_PATH=$(pwd)/plugins/GENERIC_debug_link/lib:${GAZEBO_PLUGIN_PATH}
export GAZEBO_PLUGIN_PATH=$(pwd)/plugins/GENERIC_camera_ros/lib:${GAZEBO_PLUGIN_PATH}
export GAZEBO_PLUGIN_PATH=$(pwd)/plugins/GENERIC_change_material/lib:${GAZEBO_PLUGIN_PATH}
export GAZEBO_PLUGIN_PATH=$(pwd)/plugins/GENERIC_set_parameters/lib:${GAZEBO_PLUGIN_PATH}
export GAZEBO_PLUGIN_PATH=$(pwd)/plugins/GENERIC_set_model_pose/lib:${GAZEBO_PLUGIN_PATH}
export GAZEBO_PLUGIN_PATH=$(pwd)/plugins/GENERIC_events/lib:${GAZEBO_PLUGIN_PATH}
