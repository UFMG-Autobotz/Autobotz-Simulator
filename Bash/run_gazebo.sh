cd ..

#source /usr/share/gazebo/setup.sh
#export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:$(pwd)

export GAZEBO_MODEL_PATH=$(pwd)/Models:$GAZEBO_MODEL_PATH
gazebo
