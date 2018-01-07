cd ..

# adicionar caminho para o plugin
export GAZEBO_PLUGIN_PATH=$(pwd)/plugins/TEST_get_joints/import:${GAZEBO_PLUGIN_PATH}

# iniciar gazebo com mundo parado
gazebo worlds/TEST_test_joints.world -u --verbose
