cd ..

# salvar .sdf a partir do .rsdf parametrizavel
erb Models/box/model.rsdf > Models/box/model.sdf

# adicionar caminho para o modelo
export GAZEBO_MODEL_PATH=$(pwd)/Models:$GAZEBO_MODEL_PATH

# adicionar caminho para o plugin
export GAZEBO_PLUGIN_PATH=$(pwd)/Plugins/ros_connection/build:${GAZEBO_PLUGIN_PATH}

# iniciar gazebo com mundo parado
gazebo Worlds/cubo_ros.world -u
