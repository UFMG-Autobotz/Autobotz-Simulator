cd ..

# salvar .sdf a partir do .rsdf parametrizavel
erb Models/VSS_Player/model.rsdf > Models/VSS_Player/model.sdf
erb Models/VSS_Ball/model.rsdf > Models/VSS_Ball/model.sdf

# necessario antes de modificar o GAZEBO_RESOURCE_PATH
source /usr/share/gazebo/setup.sh

# adicionar caminho para a materiais
export GAZEBO_RESOURCE_PATH=$(pwd)/Materials:$GAZEBO_RESOURCE_PATH

# adicionar caminho para o stl do campo
export GAZEBO_RESOURCE_PATH=$(pwd)/Meshs:$GAZEBO_RESOURCE_PATH

# adicionar caminho para os modelos
export GAZEBO_MODEL_PATH=$(pwd)/Models:$GAZEBO_MODEL_PATH

# adicionar caminho para os plugins
export GAZEBO_PLUGIN_PATH=$(pwd)/Plugins/pid_control/import:${GAZEBO_PLUGIN_PATH}
export GAZEBO_PLUGIN_PATH=$(pwd)/Plugins/debug_link/import:${GAZEBO_PLUGIN_PATH}
export GAZEBO_PLUGIN_PATH=$(pwd)/Plugins/camera_ros/import:${GAZEBO_PLUGIN_PATH}
export GAZEBO_PLUGIN_PATH=$(pwd)/Plugins/change_texture/import:${GAZEBO_PLUGIN_PATH}
export GAZEBO_PLUGIN_PATH=$(pwd)/Plugins/vss_players_config/import:${GAZEBO_PLUGIN_PATH}

# iniciar gazebo com mundo parado
gazebo Worlds/vss.world -u --verbose
