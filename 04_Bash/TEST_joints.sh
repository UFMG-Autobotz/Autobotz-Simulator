cd ..

# adicionar caminho para o plugin
export GAZEBO_PLUGIN_PATH=$(pwd)/03_Plugins/TEST_get_joints/import:${GAZEBO_PLUGIN_PATH}

# iniciar gazebo com mundo parado
gazebo 02_Worlds/TEST_test_joints.world -u --verbose
