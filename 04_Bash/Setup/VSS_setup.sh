cd ..

# salvar .sdf a partir do .rsdf parametrizavel
erb 01_Models/VSS_player/model.rsdf > 01_Models/VSS_player/model.sdf
erb 01_Models/VSS_ball/model.rsdf > 01_Models/VSS_ball/model.sdf

# necessario antes de modificar o GAZEBO_RESOURCE_PATH
source /usr/share/gazebo/setup.sh

# adicionar caminho para a materiais
export GAZEBO_RESOURCE_PATH=$(pwd)/05_Materials:$GAZEBO_RESOURCE_PATH

# adicionar caminho para o stl do campo
export GAZEBO_RESOURCE_PATH=$(pwd)/06_Meshs:$GAZEBO_RESOURCE_PATH

# adicionar caminho para os modelos
export GAZEBO_MODEL_PATH=$(pwd)/01_Models:$GAZEBO_MODEL_PATH

# adicionar caminho para os plugins
export GAZEBO_PLUGIN_PATH=$(pwd)/03_Plugins/GENERIC_pid_control/import:${GAZEBO_PLUGIN_PATH}
export GAZEBO_PLUGIN_PATH=$(pwd)/03_Plugins/GENERIC_debug_link/import:${GAZEBO_PLUGIN_PATH}
export GAZEBO_PLUGIN_PATH=$(pwd)/03_Plugins/GENERIC_camera_ros/import:${GAZEBO_PLUGIN_PATH}
export GAZEBO_PLUGIN_PATH=$(pwd)/03_Plugins/GENERIC_change_material/import:${GAZEBO_PLUGIN_PATH}
export GAZEBO_PLUGIN_PATH=$(pwd)/03_Plugins/GENERIC_set_parameters/import:${GAZEBO_PLUGIN_PATH}
export GAZEBO_PLUGIN_PATH=$(pwd)/03_Plugins/GENERIC_set_model_pose/import:${GAZEBO_PLUGIN_PATH}
export GAZEBO_PLUGIN_PATH=$(pwd)/03_Plugins/GENERIC_events/import:${GAZEBO_PLUGIN_PATH}
