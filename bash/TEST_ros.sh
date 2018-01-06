cd ..

# salvar .sdf a partir do .rsdf parametrizavel
erb models/TEST_box/model.rsdf > models/TEST_box/model.sdf

# adicionar caminho para o modelo
export GAZEBO_MODEL_PATH=$(pwd)/models:$GAZEBO_MODEL_PATH

# adicionar caminho para o plugin
export GAZEBO_PLUGIN_PATH=$(pwd)/plugins/TEST_ros_connection/build:${GAZEBO_PLUGIN_PATH}

# iniciar gazebo com mundo parado
gazebo worlds/TEST_cubo_ros.world -u --verbose
