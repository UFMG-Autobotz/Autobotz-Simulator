cd ..

# salvar .sdf a partir do .rsdf parametrizavel
erb Models/VSS_Player/model.rsdf > Models/VSS_Player/model.sdf
erb Models/VSS_Ball/model.rsdf > Models/VSS_Ball/model.sdf

# adicionar caminho para a materiais
source /usr/share/gazebo/setup.sh
export GAZEBO_RESOURCE_PATH=$(pwd)/Materials:$GAZEBO_RESOURCE_PATH

# adicionar caminho para os modelos
export GAZEBO_MODEL_PATH=$(pwd)/Models:$GAZEBO_MODEL_PATH

# adicionar caminho para os plugins
export GAZEBO_PLUGIN_PATH=$(pwd)/Plugins/PID_Control/import:${GAZEBO_PLUGIN_PATH}
export GAZEBO_PLUGIN_PATH=$(pwd)/Plugins/camera_plugin/import:${GAZEBO_PLUGIN_PATH}

# iniciar gazebo com mundo parado
gazebo Worlds/vss_test.world -u --verbose
