cd ..

# salvar .sdf a partir do .rsdf parametrizavel
erb Models/TEST_box/model.rsdf > Models/TEST_box/model.sdf

# adicionar caminho para o modelo
export GAZEBO_MODEL_PATH=$(pwd)/Models:$GAZEBO_MODEL_PATH

# adicionar caminho para o plugin
export GAZEBO_PLUGIN_PATH=$(pwd)/Plugins/TEST_ros_connection/build:${GAZEBO_PLUGIN_PATH}

# iniciar gazebo com mundo parado
gazebo Worlds/TEST_cubo_ros.world -u --verbose
