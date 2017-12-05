cd ..

# salvar .sdf a partir do .rsdf parametrizavel
erb Models/VT_vt_simplificado/model.rsdf > Models/VT_vt_simplificado/model.sdf

# adicionar caminho para as texturas
source /usr/share/gazebo/setup.sh
export GAZEBO_RESOURCE_PATH=$(pwd)/Materials/Textures:$GAZEBO_RESOURCE_PATH

# adicionar caminho para os modelos
export GAZEBO_MODEL_PATH=$(pwd)/Models:$GAZEBO_MODEL_PATH

# adicionar caminho para os plugins
export GAZEBO_PLUGIN_PATH=$(pwd)/Plugins/GENERIC_pid_control/import:${GAZEBO_PLUGIN_PATH}
export GAZEBO_PLUGIN_PATH=$(pwd)/Plugins/GENERIC_camera_ros/import:${GAZEBO_PLUGIN_PATH}
