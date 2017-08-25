source /opt/ros/kinetic/setup.bash
cd build
cmake ..
make
cp libMODEL_NAME_plugin.so ../plugins/
