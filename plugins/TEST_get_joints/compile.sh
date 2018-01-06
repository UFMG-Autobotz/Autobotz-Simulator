mkdir build
mkdir lib
cd build
cmake ..
make
cp libget_joints_plugin.so ../lib/
rm -rf *
