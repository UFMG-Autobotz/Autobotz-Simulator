cd ..

# adicionar caminho para o plugin
export GAZEBO_PLUGIN_PATH=$(pwd)/Plugins/get_joints/import:${GAZEBO_PLUGIN_PATH}

# iniciar gazebo com mundo parado
gazebo Worlds/test_joints.world -u --verbose
