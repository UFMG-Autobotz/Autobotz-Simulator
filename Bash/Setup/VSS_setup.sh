cd ..

# salvar .sdf a partir do .rsdf parametrizavel
erb Models/VSS_player/model.rsdf > Models/VSS_player/model.sdf
erb Models/VSS_ball/model.rsdf > Models/VSS_ball/model.sdf

# necessario antes de modificar o GAZEBO_RESOURCE_PATH
source /usr/share/gazebo/setup.sh

# adicionar caminho para a materiais
export GAZEBO_RESOURCE_PATH=$(pwd)/Materials:$GAZEBO_RESOURCE_PATH

# adicionar caminho para o stl do campo
export GAZEBO_RESOURCE_PATH=$(pwd)/Meshs:$GAZEBO_RESOURCE_PATH

# adicionar caminho para os modelos
export GAZEBO_MODEL_PATH=$(pwd)/Models:$GAZEBO_MODEL_PATH

# adicionar caminho para os plugins
export GAZEBO_PLUGIN_PATH=$(pwd)/Plugins/GENERIC_pid_control/import:${GAZEBO_PLUGIN_PATH}
export GAZEBO_PLUGIN_PATH=$(pwd)/Plugins/GENERIC_debug_link/import:${GAZEBO_PLUGIN_PATH}
export GAZEBO_PLUGIN_PATH=$(pwd)/Plugins/GENERIC_camera_ros/import:${GAZEBO_PLUGIN_PATH}
export GAZEBO_PLUGIN_PATH=$(pwd)/Plugins/GENERIC_change_texture/import:${GAZEBO_PLUGIN_PATH}
export GAZEBO_PLUGIN_PATH=$(pwd)/Plugins/VSS_players_config/import:${GAZEBO_PLUGIN_PATH}
export GAZEBO_PLUGIN_PATH=$(pwd)/Plugins/GENERIC_set_model_pose/import:${GAZEBO_PLUGIN_PATH}
export GAZEBO_PLUGIN_PATH=$(pwd)/Plugins/GENERIC_events/import:${GAZEBO_PLUGIN_PATH}
