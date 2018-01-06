cd ..

# salvar .sdf a partir do .rsdf parametrizavel
erb models/VT_vt_simplificado/model.rsdf > models/VT_vt_simplificado/model.sdf

# adicionar caminho para as texturas
source /usr/share/gazebo/setup.sh
export GAZEBO_RESOURCE_PATH=$(pwd)/materials/Textures:$GAZEBO_RESOURCE_PATH

# adicionar caminho para os modelos
export GAZEBO_MODEL_PATH=$(pwd)/models:$GAZEBO_MODEL_PATH

# adicionar caminho para os plugins
export GAZEBO_PLUGIN_PATH=$(pwd)/plugins/GENERIC_pid_control/lib:${GAZEBO_PLUGIN_PATH}
export GAZEBO_PLUGIN_PATH=$(pwd)/plugins/GENERIC_camera_ros/lib:${GAZEBO_PLUGIN_PATH}
