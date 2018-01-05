cd ..

# salvar .sdf a partir do .rsdf parametrizavel
erb 01_Models/TEST_box/model.rsdf > 01_Models/TEST_box/model.sdf

# adicionar caminho para o modelo
export GAZEBO_MODEL_PATH=$(pwd)/01_Models:$GAZEBO_MODEL_PATH

# adicionar caminho para o plugin
export GAZEBO_PLUGIN_PATH=$(pwd)/03_Plugins/TEST_push_model/build:${GAZEBO_PLUGIN_PATH}

# iniciar gazebo com mundo parado
gazebo 02_Worlds/TEST_cubo_velocity.world -u --verbose
