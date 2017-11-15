mkdir build
mkdir import
cd build
cmake ..
make
cp libget_joints_plugin.so ../import/
rm -rf *
