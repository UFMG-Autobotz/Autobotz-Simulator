cd ..

# salvar .sdf a partir do .rsdf parametrizavel
erb 01_Models/VT_vt_simplificado/model.rsdf > 01_Models/VT_vt_simplificado/model.sdf

# adicionar caminho para as texturas
source /usr/share/gazebo/setup.sh
export GAZEBO_RESOURCE_PATH=$(pwd)/05_Materials/Textures:$GAZEBO_RESOURCE_PATH

# adicionar caminho para os modelos
export GAZEBO_MODEL_PATH=$(pwd)/01_Models:$GAZEBO_MODEL_PATH

# adicionar caminho para os plugins
export GAZEBO_PLUGIN_PATH=$(pwd)/03_Plugins/GENERIC_pid_control/import:${GAZEBO_PLUGIN_PATH}
export GAZEBO_PLUGIN_PATH=$(pwd)/03_Plugins/GENERIC_camera_ros/import:${GAZEBO_PLUGIN_PATH}
