mkdir build
mkdir lib
cd build
cmake ..
make
cp libmodelname_plugin.so ../lib/
rm -rf *
